//! Sampling PC profiler.
//!
//! Every `interval` emulated cycles the profiler records:
//! - each Cortex-M33 core's PC (where the firmware spends its time), and
//! - the guest 6809 PC (where the CoCo program spends its time).
//!
//! The 6809 is not modelled by this emulator; it is XRoar's `struct MC6809`
//! living in firmware SRAM. Its PC is found by pointer chase:
//! `g_m` (static `CocoMachine` in coco_machine.cpp) -> `.cpu` -> `.reg_pc`.
//! The address of `g_m` comes from the firmware ELF symbol table. The two
//! field offsets are not in the ELF's debug info, so they were read off the
//! firmware disassembly (`mc6809_get_pc` is `ldrh r0, [r0, #88]`; loads of
//! `g_m.cpu` use `#340`) and can be overridden if the firmware layout moves.

use std::collections::HashMap;
use std::io::Write;
use std::path::Path;

use rp2350_emu::Emulator;

/// Offset of `struct MC6809 *cpu` inside `CocoMachine`.
pub const DEFAULT_CPU_PTR_OFFSET: u32 = 340;
/// Offset of `uint16_t reg_pc` inside `struct MC6809`.
pub const DEFAULT_REG_PC_OFFSET: u32 = 88;

/// Symbol names `g_m` may appear under (plain C, or C++ internal linkage).
const MACHINE_SYMBOLS: [&str; 2] = ["_ZL3g_m", "g_m"];

const TOP_N: usize = 50;

/// A function symbol from the firmware ELF.
#[derive(Debug, Clone)]
pub struct FuncSym {
    pub addr: u32,
    pub size: u32,
    pub name: String,
}

/// Symbols needed by the profiler, read from an ELF32 little-endian file.
#[derive(Debug, Default)]
pub struct ElfSymbols {
    /// Function symbols sorted by address (Thumb bit cleared).
    pub funcs: Vec<FuncSym>,
    /// Data object symbols by name.
    pub objects: HashMap<String, u32>,
}

impl ElfSymbols {
    pub fn load(path: &Path) -> Result<Self, String> {
        let data = std::fs::read(path).map_err(|e| format!("{}: {}", path.display(), e))?;
        Self::parse(&data)
    }

    pub fn parse(d: &[u8]) -> Result<Self, String> {
        let u16_at = |o: usize| -> Result<u16, String> {
            d.get(o..o + 2)
                .map(|b| u16::from_le_bytes([b[0], b[1]]))
                .ok_or_else(|| "truncated ELF".to_string())
        };
        let u32_at = |o: usize| -> Result<u32, String> {
            d.get(o..o + 4)
                .map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
                .ok_or_else(|| "truncated ELF".to_string())
        };

        if d.len() < 52 || &d[0..4] != b"\x7fELF" || d[4] != 1 || d[5] != 1 {
            return Err("not a 32-bit little-endian ELF".to_string());
        }
        let shoff = u32_at(0x20)? as usize;
        let shentsize = u16_at(0x2E)? as usize;
        let shnum = u16_at(0x30)? as usize;

        let mut syms = Self::default();
        for i in 0..shnum {
            let sh = shoff + i * shentsize;
            const SHT_SYMTAB: u32 = 2;
            if u32_at(sh + 4)? != SHT_SYMTAB {
                continue;
            }
            let off = u32_at(sh + 0x10)? as usize;
            let size = u32_at(sh + 0x14)? as usize;
            let link = u32_at(sh + 0x18)? as usize;
            let entsize = (u32_at(sh + 0x24)? as usize).max(16);
            let strtab = shoff + link * shentsize;
            let str_off = u32_at(strtab + 0x10)? as usize;

            for e in (off..off + size).step_by(entsize) {
                let name_off = u32_at(e)? as usize;
                let value = u32_at(e + 4)?;
                let sym_size = u32_at(e + 8)?;
                let info = *d.get(e + 12).ok_or("truncated ELF")?;
                let start = str_off + name_off;
                let end = d[start..].iter().position(|&b| b == 0).map(|n| start + n);
                let Some(end) = end else { continue };
                let name = String::from_utf8_lossy(&d[start..end]).into_owned();
                if name.is_empty() {
                    continue;
                }
                match info & 0xF {
                    2 => syms.funcs.push(FuncSym { addr: value & !1, size: sym_size, name }),
                    1 => {
                        syms.objects.insert(name, value);
                    }
                    _ => {}
                }
            }
        }
        syms.funcs.sort_by_key(|f| f.addr);
        Ok(syms)
    }

    /// Function containing `addr`, if any.
    pub fn func_at(&self, addr: u32) -> Option<&FuncSym> {
        let i = self.funcs.partition_point(|f| f.addr <= addr);
        let f = self.funcs.get(i.checked_sub(1)?)?;
        // Zero-size symbols (hand-written asm) claim up to the next symbol.
        (f.size == 0 || addr < f.addr + f.size).then_some(f)
    }

    /// Address of the firmware's `CocoMachine g_m`.
    pub fn machine_addr(&self) -> Option<u32> {
        MACHINE_SYMBOLS.iter().find_map(|n| self.objects.get(*n).copied())
    }
}

/// Where to find the guest 6809 PC in firmware RAM.
#[derive(Debug, Clone, Copy)]
pub struct M6809Locator {
    pub machine_addr: u32,
    pub cpu_ptr_offset: u32,
    pub reg_pc_offset: u32,
}

impl M6809Locator {
    /// Read the guest PC, or None while `g_m.cpu` is not set up yet.
    pub fn read_pc(&self, emu: &Emulator) -> Option<u16> {
        let cpu = peek_u32(emu, self.machine_addr + self.cpu_ptr_offset);
        if !(0x2000_0000..0x2008_2000).contains(&cpu) {
            return None;
        }
        Some(peek_u16(emu, cpu + self.reg_pc_offset))
    }
}

fn peek_u32(emu: &Emulator, addr: u32) -> u32 {
    let base = addr & !3;
    let shift = (addr & 3) * 8;
    if shift == 0 {
        emu.peek(base)
    } else {
        let lo = emu.peek(base) as u64;
        let hi = emu.peek(base + 4) as u64;
        (((hi << 32) | lo) >> shift) as u32
    }
}

fn peek_u16(emu: &Emulator, addr: u32) -> u16 {
    peek_u32(emu, addr) as u16
}

pub struct Profiler {
    pub interval: u64,
    arm: [HashMap<u32, u64>; 2],
    m6809: HashMap<u16, u64>,
    m6809_idle: u64,
    samples: u64,
    locator: Option<M6809Locator>,
    symbols: Option<ElfSymbols>,
}

impl Profiler {
    pub fn new(interval: u64, symbols: Option<ElfSymbols>, locator: Option<M6809Locator>) -> Self {
        Self {
            interval: interval.max(1),
            arm: [HashMap::new(), HashMap::new()],
            m6809: HashMap::new(),
            m6809_idle: 0,
            samples: 0,
            locator,
            symbols,
        }
    }

    pub fn sample(&mut self, emu: &Emulator) {
        self.samples += 1;
        for core in 0..2 {
            *self.arm[core].entry(emu.core(core).regs.pc()).or_default() += 1;
        }
        if let Some(loc) = &self.locator {
            match loc.read_pc(emu) {
                Some(pc) => *self.m6809.entry(pc).or_default() += 1,
                None => self.m6809_idle += 1,
            }
        }
    }

    pub fn write_report(&self, path: &Path) -> std::io::Result<()> {
        let mut out = std::io::BufWriter::new(std::fs::File::create(path)?);
        self.report(&mut out)?;
        out.flush()
    }

    pub fn report(&self, out: &mut impl Write) -> std::io::Result<()> {
        writeln!(out, "# CoCo Zero PC profile")?;
        writeln!(out, "# {} samples, one every {} cycles", self.samples, self.interval)?;
        writeln!(out)?;

        for core in 0..2 {
            writeln!(out, "## ARM core {} (by function)", core)?;
            let mut by_func: HashMap<String, u64> = HashMap::new();
            for (&pc, &n) in &self.arm[core] {
                let key = match self.symbols.as_ref().and_then(|s| s.func_at(pc)) {
                    Some(f) => f.name.clone(),
                    None => format!("0x{:08X}", pc),
                };
                *by_func.entry(key).or_default() += n;
            }
            write_top(out, by_func.into_iter().collect(), self.samples)?;
            writeln!(out)?;
        }

        writeln!(out, "## 6809 guest (by PC)")?;
        match &self.locator {
            None => writeln!(out, "(not sampled: pass --elf so g_m can be located)")?,
            Some(loc) => {
                writeln!(
                    out,
                    "# g_m=0x{:08X} cpu@+{} reg_pc@+{}; {} samples before the 6809 existed",
                    loc.machine_addr, loc.cpu_ptr_offset, loc.reg_pc_offset, self.m6809_idle
                )?;
                let rows = self.m6809.iter().map(|(&pc, &n)| (format!("${:04X}", pc), n)).collect();
                let live = self.samples - self.m6809_idle;
                write_top(out, rows, live)?;
            }
        }
        Ok(())
    }
}

fn write_top(out: &mut impl Write, mut rows: Vec<(String, u64)>, total: u64) -> std::io::Result<()> {
    rows.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
    writeln!(out, "{:>7} {:>10}  location", "pct", "samples")?;
    for (name, n) in rows.iter().take(TOP_N) {
        let pct = 100.0 * *n as f64 / total.max(1) as f64;
        writeln!(out, "{:>6.2}% {:>10}  {}", pct, n, name)?;
    }
    if rows.len() > TOP_N {
        let rest: u64 = rows[TOP_N..].iter().map(|r| r.1).sum();
        writeln!(out, "{:>6.2}% {:>10}  ({} more)", 100.0 * rest as f64 / total.max(1) as f64, rest, rows.len() - TOP_N)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn func_at_respects_size_and_zero_size() {
        let s = ElfSymbols {
            funcs: vec![
                FuncSym { addr: 0x100, size: 0x10, name: "a".into() },
                FuncSym { addr: 0x200, size: 0, name: "b".into() },
            ],
            objects: HashMap::new(),
        };
        assert_eq!(s.func_at(0x108).unwrap().name, "a");
        assert!(s.func_at(0x110).is_none());
        assert!(s.func_at(0xFF).is_none());
        assert_eq!(s.func_at(0x2FF).unwrap().name, "b");
    }

    #[test]
    fn rejects_non_elf() {
        assert!(ElfSymbols::parse(b"not an elf file at all, definitely not one....").is_err());
    }
}
