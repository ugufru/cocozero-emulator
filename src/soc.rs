//! CoCo Zero RP2350B System-on-a-Chip Emulator Engine.
//!
//! Ties together:
//! - Dual ARM Cortex-M33 cores (252 MHz sys_clk)
//! - Hardware TMDS encoder in SIO
//! - SPI1 MicroSD card controller with software CS on GPIO 43
//! - UART0 serial console telemetry
//! - 520 KB SRAM with 320×240 RGB565 Framebuffer extraction

use std::collections::VecDeque;
use std::io::Write;
use std::sync::{Arc, Mutex};

use rp2350_emu::{Config, Emulator, EmulatorBuilder};

use crate::keyboard::KeyboardMatrix;
use crate::profiler::Profiler;
use crate::spi_sd::{SectorStorage, SpiSdCard};
use crate::symbols::FirmwareSymbols;
use crate::uf2::FlashImage;

pub const SYS_CLK_HZ: u32 = 252_000_000;
pub const FB_WIDTH: usize = 320;
pub const FB_HEIGHT: usize = 240;
pub const FB_PIXELS: usize = FB_WIDTH * FB_HEIGHT;

pub struct CoCoZeroSoC {
    pub emu: Emulator,
    pub sd_card: Arc<Mutex<SpiSdCard>>,
    pub serial_tx_log: Arc<Mutex<Vec<u8>>>,
    pub serial_rx_queue: VecDeque<u8>,
    pub fb_addr: Option<u32>,
    pub frame_counter: u64,
    pub total_cycles: u64,
    pub keyboard_ready_forced: bool,
    pub keyboard: KeyboardMatrix,
    pub symbols: FirmwareSymbols,
    pub profiler: Option<Profiler>,
}

impl CoCoZeroSoC {
    pub fn new(sd_storage: Box<dyn SectorStorage + Send>) -> Result<Self, String> {
        let config = Config {
            sys_clk_hz: SYS_CLK_HZ,
        };
        let mut emu = EmulatorBuilder::new(config)
            .step_quantum(512)
            .build()
            .map_err(|e| format!("Failed to build RP2350 emulator: {:?}", e))?;

        let sd_card = Arc::new(Mutex::new(SpiSdCard::new(sd_storage)));
        let serial_tx_log = Arc::new(Mutex::new(Vec::new()));

        // Wire SPI1 to SD card
        let sd_clone = Arc::clone(&sd_card);
        emu.bus.spi[1].slave_handler = Some(Box::new(move |mosi| {
            let mut card = sd_clone.lock().unwrap();
            card.transfer_byte(mosi)
        }));

        // Wire UART0 TX to serial telemetry log + stdout
        let tx_clone = Arc::clone(&serial_tx_log);
        emu.bus.uart[0].tx_sink = Some(Box::new(move |byte| {
            tx_clone.lock().unwrap().push(byte);
            print!("{}", byte as char);
            let _ = std::io::stdout().flush();
        }));

        let symbols = FirmwareSymbols::default();
        Ok(Self {
            emu,
            sd_card,
            serial_tx_log,
            serial_rx_queue: VecDeque::new(),
            fb_addr: Some(symbols.fb_addr),
            frame_counter: 0,
            total_cycles: 0,
            keyboard_ready_forced: false,
            keyboard: KeyboardMatrix::new(),
            symbols,
            profiler: None,
        })
    }

    /// Load Flash memory image (from UF2 or raw binary) into XIP Flash space.
    pub fn load_firmware(&mut self, flash: &FlashImage) {
        self.load_firmware_with_elf(flash, None);
    }

    /// Load Flash memory image with an optional companion ELF file for symbol extraction.
    pub fn load_firmware_with_elf(&mut self, flash: &FlashImage, elf_path: Option<&std::path::Path>) {
        self.emu.load_flash(&flash.memory);

        // Auto-detect firmware symbols from ELF or in-flash signatures
        let syms = FirmwareSymbols::auto_detect(elf_path, &flash.memory, crate::uf2::FLASH_BASE);
        self.symbols = syms;
        self.fb_addr = Some(syms.fb_addr);
        self.keyboard.matrix_addr = syms.kb_matrix_addr;

        // If bootrom is not loaded, initialize core 0 registers directly from flash vector table
        if let (Some(sp), Some(pc)) = (flash.initial_sp, flash.entry_point) {
            println!("Initializing Core 0 vectors directly: SP=0x{:08X}, PC=0x{:08X}", sp, pc);
            self.emu.core_mut(0).regs.msp = sp;
            self.emu.core_mut(0).regs.r[13] = sp;
            self.emu.core_mut(0).regs.set_pc(pc & !1);
            self.emu.core_mut(0).regs.xpsr = 1 << 24; // Thumb mode
            // Core 1 waits in WFE until awakened by multicore launch handshake
            self.emu.bus.atomics.set_wfe_waiting(1);

            // When Core 0 bypasses Boot ROM directly into Flash, Boot ROM's
            // security initialization (which sets RCP salt on both cores)
            // was skipped. Initialize RCP salt so Core 1's Boot ROM monitor
            // validates the canary check upon waking and handles the launch handshake.
            self.emu.bus.atomics.rcp_salt_set(0, 0xCAFE_BABE);
            self.emu.bus.atomics.rcp_salt_set(1, 0xDEAD_BEEF);
        }
    }

    /// Load RP2350 Boot ROM binary into 0x00000000..0x00007FFF.
    pub fn load_bootrom(&mut self, bootrom_bytes: &[u8]) {
        self.emu.load_bootrom(bootrom_bytes);
        self.emu.reset();
    }

    /// Update MicroSD Chip Select from SIO GPIO 43.
    fn update_sd_cs(&mut self) {
        let gpio_hi = self.emu.bus.sio.gpio_hi_out;
        // GPIO 43 is bit 11 in GPIO_HI_OUT (active low)
        let cs_asserted = (gpio_hi & (1 << (43 - 32))) == 0;
        self.sd_card.lock().unwrap().set_cs(cs_asserted);
    }

    /// Inject a character into UART0 serial console.
    pub fn push_serial_char(&mut self, ch: u8) {
        self.emu.bus.uart[0].push_rx_byte(ch);
    }

    /// Type an ASCII character into the Color Computer keyboard matrix.
    pub fn type_char(&mut self, ch: char) {
        self.keyboard.type_char(ch);
    }

    /// Type a string into the Color Computer keyboard matrix.
    pub fn type_str(&mut self, s: &str) {
        self.keyboard.type_str(s);
    }

    /// Press a physical key on the keyboard matrix.
    pub fn key_down(&mut self, key: sdl2::keyboard::Keycode) {
        self.keyboard.key_down(key);
    }

    /// Release a physical key on the keyboard matrix.
    pub fn key_up(&mut self, key: sdl2::keyboard::Keycode) {
        self.keyboard.key_up(key);
    }

    /// Advance the SoC simulation by virtual cycles.
    pub fn step_cycles(&mut self, cycles: u64) -> Result<(), String> {
        self.update_sd_cs();

        // Feed any pending serial input
        if let Some(ch) = self.serial_rx_queue.pop_front() {
            self.emu.bus.uart[0].push_rx_byte(ch);
        }

        match self.profiler.as_mut() {
            None => {
                self.emu
                    .run(cycles)
                    .map_err(|e| format!("Emulator run error: {:?}", e))?;
            }
            Some(prof) => {
                // Run in profiler-sized slices, sampling PCs between them.
                let mut left = cycles;
                while left > 0 {
                    let n = left.min(prof.interval);
                    self.emu
                        .run(n)
                        .map_err(|e| format!("Emulator run error: {:?}", e))?;
                    prof.sample(&self.emu);
                    left -= n;
                }
            }
        }

        self.total_cycles += cycles;
        self.update_sd_cs();
        Ok(())
    }

    /// Check whether Color BASIC has completed boot and is ready for keyboard input.
    pub fn is_keyboard_ready(&self) -> bool {
        self.keyboard_ready_forced || self.total_cycles >= 630_000_000
    }

    /// Step by one frame quantum (~252 MHz / 60 Hz = 4,200,000 cycles).
    pub fn step_frame(&mut self) -> Result<(), String> {
        self.step_frame_turbo(1)
    }

    /// Step by one frame quantum divided by turbo factor.
    pub fn step_frame_turbo(&mut self, turbo: u32) -> Result<(), String> {
        let ready = self.is_keyboard_ready();
        self.keyboard.sync_frame(&mut self.emu.bus, turbo, ready);
        let t = turbo.max(1) as u64;
        let cycles_per_frame = ((SYS_CLK_HZ / 60) as u64) / t;
        self.step_cycles(cycles_per_frame)?;
        self.frame_counter += 1;
        Ok(())
    }

    /// Scan SRAM to discover the active 320×240 RGB565 framebuffer.
    pub fn detect_framebuffer(&mut self) -> Option<u32> {
        if let Some(addr) = self.fb_addr {
            return Some(addr);
        }

        // Look through SRAM (0x20000000..0x20080000) in 4KB steps
        // Look for regions with non-zero color data
        let sram_base = 0x2000_0000u32;
        let sram_size = 520 * 1024;
        let fb_byte_size = (FB_PIXELS * 2) as u32;

        for offset in (0..(sram_size - fb_byte_size)).step_by(1024) {
            let addr = sram_base + offset;
            let mut nonzero_count = 0;
            for i in 0..100 {
                let pixel = self.emu.bus.memory.sram_read16(offset + i * 2);
                if pixel != 0 {
                    nonzero_count += 1;
                }
            }
            if nonzero_count > 20 {
                println!("Detected active framebuffer at SRAM 0x{:08X}", addr);
                self.fb_addr = Some(addr);
                return Some(addr);
            }
        }

        None
    }

    /// Extract the 320×240 RGB565 frame from SRAM for rendering.
    pub fn extract_frame(&mut self, out: &mut [u16; FB_PIXELS]) -> bool {
        let addr = self.fb_addr.unwrap_or(self.symbols.fb_addr);
        let offset = addr.saturating_sub(0x2000_0000);
        let mem = &self.emu.bus.memory;
        for i in 0..(FB_PIXELS / 2) {
            let val = mem.sram_read32(offset + (i as u32) * 4);
            out[i * 2] = val as u16;
            out[i * 2 + 1] = (val >> 16) as u16;
        }
        true
    }

    /// Check whether a firmware menu overlay (Disks, Programs, Carts, Files, Info) is active.
    pub fn is_menu_active(&self) -> bool {
        self.emu.bus.memory.sram_read8(self.symbols.menu_active_addr - 0x2000_0000) != 0
    }

    /// Get current active menu mode (0 = Disks, 1 = Programs, 2 = Carts, 3 = Files, 4 = Info).
    pub fn get_menu_mode(&self) -> u32 {
        self.emu.bus.memory.sram_read32(self.symbols.menu_mode_addr - 0x2000_0000)
    }

    /// Inject a USB HID keycode report into the firmware's TinyUSB host handler on Core 0.
    pub fn inject_hid_keycode(&mut self, keycode: u8) {
        // Trampoline at TRAMP_ADDR: 'b .' (0xE7FE)
        self.emu.bus.write16(TRAMP_ADDR, 0xE7FE, 0);

        // Dedicated scratch HID report buffer located right below trampoline in upper SRAM
        let report_addr = TRAMP_ADDR - 0x10;

        // 1. Send Key Press report [modifier, reserved, keycode, 0, 0, 0, 0, 0]
        self.emu.bus.write8(report_addr, 0, 0);     // modifier
        self.emu.bus.write8(report_addr + 1, 0, 0); // reserved
        self.emu.bus.write8(report_addr + 2, keycode, 0); // keycode[0]
        for i in 3..8 {
            self.emu.bus.write8(report_addr + i, 0, 0);
        }

        let saved_r = self.emu.core(0).regs.r;
        let saved_xpsr = self.emu.core(0).regs.xpsr;
        let saved_msp = self.emu.core(0).regs.msp;

        self.emu.core_mut(0).regs.r[0] = 1; // dev_addr = 1
        self.emu.core_mut(0).regs.r[1] = 0; // instance = 0
        self.emu.core_mut(0).regs.r[2] = report_addr;
        self.emu.core_mut(0).regs.r[3] = 8; // len = 8
        self.emu.core_mut(0).regs.r[14] = TRAMP_ADDR | 1;
        self.emu.core_mut(0).regs.set_pc(self.symbols.hid_cb_addr);

        for _ in 0..10_000 {
            let _ = self.emu.run(100);
            if self.emu.core(0).regs.pc() == TRAMP_ADDR {
                break;
            }
        }

        // 2. Send Key Release report [0, 0, 0, 0, 0, 0, 0, 0]
        self.emu.bus.write8(report_addr + 2, 0, 0);
        self.emu.core_mut(0).regs.r[0] = 1;
        self.emu.core_mut(0).regs.r[1] = 0;
        self.emu.core_mut(0).regs.r[2] = report_addr;
        self.emu.core_mut(0).regs.r[3] = 8;
        self.emu.core_mut(0).regs.r[14] = TRAMP_ADDR | 1;
        self.emu.core_mut(0).regs.set_pc(self.symbols.hid_cb_addr);

        for _ in 0..10_000 {
            let _ = self.emu.run(100);
            if self.emu.core(0).regs.pc() == TRAMP_ADDR {
                break;
            }
        }

        // Restore Core 0 registers so it resumes previous execution cleanly
        self.emu.core_mut(0).regs.r = saved_r;
        self.emu.core_mut(0).regs.xpsr = saved_xpsr;
        self.emu.core_mut(0).regs.msp = saved_msp;
    }

    /// Trigger a menu mode directly (0=Disks, 1=Programs, 2=Carts, 3=Files, 4=Info).
    pub fn trigger_menu(&mut self, mode: u32) {
        let keycode = match mode {
            0 => 0x45, // F12 Disks
            1 => 0x42, // F9 Programs
            2 => 0x43, // F10 Cartridges
            3 => 0x44, // F11 Files
            4 => 0x3A, // F1 Info
            _ => 0x45,
        };
        self.inject_hid_keycode(keycode);
    }
}

/// Physical SRAM address of menu active flag (1 = active, 0 = inactive).
pub const MENU_ACTIVE_ADDR: u32 = 0x2000_c8d0;
/// Physical SRAM address of active menu mode (0 = Disks, 1 = Programs, 2 = Carts, 3 = Files, 4 = Info).
pub const MENU_MODE_ADDR: u32 = 0x2000_c8d4;
/// Physical SRAM address of TinyUSB HID keyboard report buffer.
pub const HID_REPORT_ADDR: u32 = 0x2001_c18c;
/// Entry point of TinyUSB host HID report callback in firmware flash.
pub const HID_CB_ADDR: u32 = 0x1001_f460;
/// Scratch trampoline location in upper SRAM.
pub const TRAMP_ADDR: u32 = 0x2007_ffe0;
