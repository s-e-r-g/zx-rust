use eframe::egui::{ColorImage, TextureHandle, TextureOptions, Ui};

pub fn draw_screen(ui: &mut Ui, pixels: &[u8], width: usize, height: usize) {
    let image = ColorImage::from_rgba_unmultiplied([width, height], pixels);
    let texture: TextureHandle = ui.ctx().load_texture("screen", image, TextureOptions::NEAREST);

    ui.image(&texture);
}
