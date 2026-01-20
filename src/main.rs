mod cpu;
mod debugger;
mod emulator;
mod screen;

use eframe::egui::{self, CentralPanel};
use emulator::{MachineZxSpectrum48, Key};
use screen::draw_screen;
use std::collections::VecDeque;

fn main() -> Result<(), eframe::Error> {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    
    // Check for help flag
    if args.contains(&"--help".to_string()) || args.contains(&"-h".to_string()) {
        println!("ZX Spectrum Emulator");
        println!("Usage: {} [OPTIONS]", args[0]);
        println!();
        println!("Options:");
        println!("  --disasm          Enable instruction disassembly tracing");
        println!("  --trace-int       Enable interrupt tracing");
        println!("  --rom=<file>      Load specified ROM file (default: roms/48.rom)");
        println!("  --run-zexall      Run ZEXALL instruction set exerciser test");
        println!("  --max-speed       Disable frame rate limiting for maximum speed");
        println!("  --show-fps        Display current FPS in the top-left corner");
        println!("  --help, -h        Show this help message");
        println!();
        println!("Examples:");
        println!("  {}                Run emulator normally", args[0]);
        println!("  {} --disasm       Run with disassembly tracing", args[0]);
        println!("  {} --trace-int    Run with interrupt tracing", args[0]);
        println!("  {} --rom=roms/Robik48.rom  Load alternative ROM", args[0]);
        println!("  {} --run-zexall   Run ZEXALL test", args[0]);
        println!("  {} --max-speed    Run at maximum speed", args[0]);
        println!("  {} --show-fps     Show FPS counter", args[0]);
        std::process::exit(0);
    }
    
    let enable_disassembler = args.contains(&"--disasm".to_string());
    let enable_trace_interrupts = args.contains(&"--trace-int".to_string());
    let run_zexall = args.contains(&"--run-zexall".to_string());
    let max_speed = args.contains(&"--max-speed".to_string());
    let show_fps = args.contains(&"--show-fps".to_string());
    
    // Parse ROM filename
    let mut rom_filename = "roms/48.rom".to_string();
    for arg in &args {
        if arg.starts_with("--rom=") {
            rom_filename = arg[6..].to_string();
        }
    }

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "ZX Spectrum Emulator",
        options,
        Box::new(move |_cc| Box::new(MyApp::new(enable_disassembler, enable_trace_interrupts, rom_filename, run_zexall, max_speed, show_fps))),
    )
}

struct MyApp {
    emulator: MachineZxSpectrum48,
    last_frame_generation_start_time: std::time::Instant,
    max_speed: bool,
    show_fps: bool,
    frame_times: std::collections::VecDeque<std::time::Instant>,
}

impl MyApp {
    fn handle_input(&mut self, ctx: &egui::Context) {
        ctx.input(|i| {
            self.emulator.set_key_state(Key::Shift, i.modifiers.shift);
            self.emulator.set_key_state(Key::Sym, i.modifiers.ctrl);

            let keys = [
                (egui::Key::Z, Key::Z), (egui::Key::X, Key::X), (egui::Key::C, Key::C), (egui::Key::V, Key::V),
                (egui::Key::A, Key::A), (egui::Key::S, Key::S), (egui::Key::D, Key::D), (egui::Key::F, Key::F), (egui::Key::G, Key::G),
                (egui::Key::Q, Key::Q), (egui::Key::W, Key::W), (egui::Key::E, Key::E), (egui::Key::R, Key::R), (egui::Key::T, Key::T),
                (egui::Key::Num1, Key::Num1), (egui::Key::Num2, Key::Num2), (egui::Key::Num3, Key::Num3), (egui::Key::Num4, Key::Num4), (egui::Key::Num5, Key::Num5),
                (egui::Key::Num0, Key::Num0), (egui::Key::Num9, Key::Num9), (egui::Key::Num8, Key::Num8), (egui::Key::Num7, Key::Num7), (egui::Key::Num6, Key::Num6),
                (egui::Key::P, Key::P), (egui::Key::O, Key::O), (egui::Key::I, Key::I), (egui::Key::U, Key::U), (egui::Key::Y, Key::Y),
                (egui::Key::Enter, Key::Enter), (egui::Key::L, Key::L), (egui::Key::K, Key::K), (egui::Key::J, Key::J), (egui::Key::H, Key::H),
                (egui::Key::Space, Key::Space), (egui::Key::M, Key::M), (egui::Key::N, Key::N), (egui::Key::B, Key::B),
            ];

            for (egui_key, zx_key) in keys.iter() {
                self.emulator.set_key_state(*zx_key, i.key_down(*egui_key));
            }
        });
    }
}

impl MyApp {
    fn new(enable_disassembler: bool, enable_trace_interrupts: bool, rom_filename: String, run_zexall: bool, max_speed: bool, show_fps: bool) -> Self {
        Self {
            emulator: MachineZxSpectrum48::new_with_options(enable_disassembler, enable_trace_interrupts, rom_filename, run_zexall),
            last_frame_generation_start_time: std::time::Instant::now(),
            max_speed,
            show_fps,
            frame_times: VecDeque::new(),
        }
    }
}

impl Default for MyApp {
    fn default() -> Self {
        Self::new(false, false, "roms/48.rom".to_string(), false, false, false)
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_input(ctx);
        let now = std::time::Instant::now();
        if self.max_speed || now.duration_since(self.last_frame_generation_start_time).as_millis() >= 20 { // ~50Hz (20ms)
            self.last_frame_generation_start_time = now;
            self.emulator.run_until_frame();
        }

        CentralPanel::default().show(ctx, |ui| {
            draw_screen(ui, &self.emulator.screen_buffer, emulator::FULL_WIDTH, emulator::FULL_HEIGHT);
        });

        if self.show_fps {
            self.frame_times.push_back(now);
            // Keep only frame times within the last second
            let one_second_ago = now - std::time::Duration::from_secs(1);
            while self.frame_times.front().map_or(false, |&t| t < one_second_ago) {
                self.frame_times.pop_front();
            }
            let fps = self.frame_times.len();

            egui::Area::new("fps_display".into())
                .fixed_pos(egui::pos2(10.0, 10.0))
                .show(ctx, |ui| {
                    ui.label(egui::RichText::new(format!("FPS: {}", fps)).color(egui::Color32::WHITE));
                });
        }
        ctx.request_repaint();
    }
}
