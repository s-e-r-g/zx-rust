mod emulator;
mod screen;

use eframe::egui::{self, CentralPanel};
use emulator::Emulator;
use screen::draw_screen;

fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "ZX Spectrum Emulator",
        options,
        Box::new(|_cc| Box::new(MyApp::default())),
    )
}

struct MyApp {
    emulator: Emulator,
}

impl Default for MyApp {
    fn default() -> Self {
        Self {
            emulator: Emulator::new(),
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.emulator.step(); // виконати кілька інструкцій
        self.emulator.render();
        CentralPanel::default().show(ctx, |ui| {
            draw_screen(ui, &self.emulator.screen_buffer, emulator::FULL_WIDTH, emulator::FULL_HEIGHT);
        });
        ctx.request_repaint();
    }
}
