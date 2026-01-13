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
    last_frame_time: std::time::Instant,
}

impl Default for MyApp {
    fn default() -> Self {
        let mut emu = Emulator::new();
        if let Err(e) = emu.load_tap("test.tap") {
            println!("Error loading tap file: {}", e);
        }
        Self {
            emulator: emu,
            last_frame_time: std::time::Instant::now(),
        }
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = std::time::Instant::now();
        if now.duration_since(self.last_frame_time).as_millis() >= 20 { // ~50Hz (20ms)
            self.last_frame_time = now;
            self.emulator.step();
            self.emulator.render();
        }

        CentralPanel::default().show(ctx, |ui| {
            draw_screen(ui, &self.emulator.screen_buffer, emulator::FULL_WIDTH, emulator::FULL_HEIGHT);
        });
        ctx.request_repaint();
    }
}
