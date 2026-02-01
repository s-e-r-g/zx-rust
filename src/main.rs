mod cpu;
mod debugger;
mod emulator;
mod screen;

use eframe::egui::{self, CentralPanel};
use emulator::{Key, MachineZxSpectrum48};
use screen::draw_screen;
use std::collections::VecDeque;

use clap::Parser;

/// ZX Spectrum Emulator
#[derive(Parser)]
#[command(version, about, long_about = None)]
struct Args {
    /// Enable instruction disassembly tracing
    #[arg(long, default_value_t = false)]
    disasm: bool,

    /// Enable interrupt tracing
    #[arg(long, default_value_t = false)]
    trace_int: bool,

    /// Load specified ROM file
    #[arg(long, default_value = "roms/48.rom")]
    rom: std::path::PathBuf,

    /// Load specified tap file
    #[arg(long)]
    tap: Option<std::path::PathBuf>,

    /// Run ZEXALL instruction set exerciser test
    #[arg(long, default_value_t = false)]
    run_zexall: bool,

    /// Run ZEXDOC instruction set exerciser test
    #[arg(long, default_value_t = false)]
    run_zexdoc: bool,

    /// Display current FPS in the top-left corner
    #[arg(long, default_value_t = false)]
    show_fps: bool,

    /// Run in headless mode for maximum performance (e.g., for tests)
    #[arg(long, default_value_t = false)]
    no_ui: bool,
}

fn main() -> Result<(), eframe::Error> {
    // Parse command line arguments
    let args = Args::parse();

    // Parse ROM filename
    let rom_filename: std::path::PathBuf;
    if args.run_zexall {
        rom_filename = "roms/zexall-0x0100.rom".into();
    } else if args.run_zexdoc {
        rom_filename = "roms/zexdoc-0x0100.rom".into();
    } else {
        rom_filename = args.rom;
    }

    if args.no_ui {
        let mut emulator = MachineZxSpectrum48::new_with_options(
            args.disasm,
            args.trace_int,
            rom_filename,
            args.run_zexall || args.run_zexdoc,
            args.tap,
        );
        loop {
            emulator.run_until_frame_without_ula();
        }
    } else {
        let options = eframe::NativeOptions::default();
        eframe::run_native(
            "ZX Spectrum Emulator",
            options,
            Box::new(move |_cc| {
                Box::new(MyApp::new(
                    args.disasm,
                    args.trace_int,
                    rom_filename,
                    args.run_zexall || args.run_zexdoc,
                    args.show_fps,
                    args.tap,
                ))
            }),
        )
    }
}

struct MyApp {
    emulator: MachineZxSpectrum48,
    last_frame_generation_start_time: std::time::Instant,
    show_fps: bool,
    frame_times: std::collections::VecDeque<std::time::Instant>,
    emulated_frames_this_second: u32,
    last_emulated_fps_calc: std::time::Instant,
    emulated_fps: u32,
}

impl MyApp {
    fn handle_input(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            // CAPS SHIFT: either Shift or Ctrl
            let caps_shift = i.modifiers.shift || i.modifiers.ctrl;
            // SYMBOL SHIFT: Alt (or keep as Ctrl if you prefer)
            let sym_shift = i.modifiers.alt;

            self.emulator.set_key_state(Key::Shift, caps_shift);
            self.emulator.set_key_state(Key::Sym, sym_shift);

            let keys = [
                (egui::Key::Z, Key::Z),
                (egui::Key::X, Key::X),
                (egui::Key::C, Key::C),
                (egui::Key::V, Key::V),
                (egui::Key::A, Key::A),
                (egui::Key::S, Key::S),
                (egui::Key::D, Key::D),
                (egui::Key::F, Key::F),
                (egui::Key::G, Key::G),
                (egui::Key::Q, Key::Q),
                (egui::Key::W, Key::W),
                (egui::Key::E, Key::E),
                (egui::Key::R, Key::R),
                (egui::Key::T, Key::T),
                (egui::Key::Num1, Key::Num1),
                (egui::Key::Num2, Key::Num2),
                (egui::Key::Num3, Key::Num3),
                (egui::Key::Num4, Key::Num4),
                (egui::Key::Num5, Key::Num5),
                (egui::Key::Num0, Key::Num0),
                (egui::Key::Num9, Key::Num9),
                (egui::Key::Num8, Key::Num8),
                (egui::Key::Num7, Key::Num7),
                (egui::Key::Num6, Key::Num6),
                (egui::Key::P, Key::P),
                (egui::Key::O, Key::O),
                (egui::Key::I, Key::I),
                (egui::Key::U, Key::U),
                (egui::Key::Y, Key::Y),
                (egui::Key::Enter, Key::Enter),
                (egui::Key::L, Key::L),
                (egui::Key::K, Key::K),
                (egui::Key::J, Key::J),
                (egui::Key::H, Key::H),
                (egui::Key::Space, Key::Space),
                (egui::Key::M, Key::M),
                (egui::Key::N, Key::N),
                (egui::Key::B, Key::B),
            ];

            for (egui_key, zx_key) in keys.iter() {
                self.emulator.set_key_state(*zx_key, i.key_down(*egui_key));
            }
        });
    }
}

impl MyApp {
    fn new(
        enable_disassembler: bool,
        enable_trace_interrupts: bool,
        rom_filename: std::path::PathBuf,
        run_zexall: bool,
        show_fps: bool,
        tap_filename: Option<std::path::PathBuf>,
    ) -> Self {
        let now = std::time::Instant::now();
        Self {
            emulator: MachineZxSpectrum48::new_with_options(
                enable_disassembler,
                enable_trace_interrupts,
                rom_filename,
                run_zexall,
                tap_filename,
            ),
            last_frame_generation_start_time: now,
            show_fps,
            frame_times: VecDeque::new(),
            emulated_frames_this_second: 0,
            last_emulated_fps_calc: now,
            emulated_fps: 0,
        }
    }
}

impl Default for MyApp {
    fn default() -> Self {
        Self::new(false, false, "roms/48.rom".into(), false, false, None)
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_input(ctx);

        let now = std::time::Instant::now();
        self.last_frame_generation_start_time = now;
        self.emulated_frames_this_second += self.emulator.run_until_time(now);

        CentralPanel::default().show(ctx, |ui| {
            draw_screen(
                ui,
                &self.emulator.screen_buffer,
                emulator::FULL_WIDTH,
                emulator::FULL_HEIGHT,
            );
        });

        if self.show_fps {
            self.frame_times.push_back(now);
            // Keep only frame times within the last second
            let one_second_ago = now - std::time::Duration::from_secs(1);
            while self
                .frame_times
                .front()
                .map_or(false, |&t| t < one_second_ago)
            {
                self.frame_times.pop_front();
            }
            let fps = self.frame_times.len();

            if now.duration_since(self.last_emulated_fps_calc).as_secs() >= 1 {
                self.emulated_fps = self.emulated_frames_this_second;
                self.emulated_frames_this_second = 0;
                self.last_emulated_fps_calc = now;
            }

            egui::Area::new("fps_display".into())
                .fixed_pos(egui::pos2(10.0, 10.0))
                .show(ctx, |ui| {
                    egui::Frame::default()
                        .fill(egui::Color32::BLACK)
                        .inner_margin(egui::Margin::symmetric(5.0, 2.0))
                        .show(ui, |ui| {
                            ui.label(
                                egui::RichText::new(format!("FPS: {}", fps))
                                    .color(egui::Color32::WHITE),
                            );
                            ui.label(
                                egui::RichText::new(format!("Emulated FPS: {}", self.emulated_fps))
                                    .color(egui::Color32::WHITE),
                            );
                        });
                });
        }
        ctx.request_repaint();
    }
}
