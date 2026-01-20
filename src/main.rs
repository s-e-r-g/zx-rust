mod cpu;
mod disassembler;
mod emulator;
mod screen;

use eframe::egui::{self, CentralPanel};
use emulator::{MachineZxSpectrum48, Key};
use screen::draw_screen;

fn main() -> Result<(), eframe::Error> {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    let enable_disassembler = args.contains(&"--disasm".to_string());

    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "ZX Spectrum Emulator",
        options,
        Box::new(move |_cc| Box::new(MyApp::new(enable_disassembler))),
    )
}

struct MyApp {
    emulator: MachineZxSpectrum48,
    last_frame_generation_start_time: std::time::Instant,
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
    fn new(enable_disassembler: bool) -> Self {
        Self {
            emulator: MachineZxSpectrum48::new_with_disassembler(enable_disassembler),
            last_frame_generation_start_time: std::time::Instant::now(),
        }
    }
}

impl Default for MyApp {
    fn default() -> Self {
        Self::new(false)
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.handle_input(ctx);
        let now = std::time::Instant::now();
        if now.duration_since(self.last_frame_generation_start_time).as_millis() >= 20 { // ~50Hz (20ms)
            self.last_frame_generation_start_time = now;
            self.emulator.run_until_frame();
        }

        CentralPanel::default().show(ctx, |ui| {
            draw_screen(ui, &self.emulator.screen_buffer, emulator::FULL_WIDTH, emulator::FULL_HEIGHT);
        });
        ctx.request_repaint();
    }
}
