#![feature(catch_expr)]

extern crate apodize;
extern crate flow_synth;
extern crate jack;
extern crate modular_flow;
extern crate palette;
extern crate rustfft;
extern crate sdl2;

mod audio_io;
mod stft;
mod pixel_scroller;

use flow_synth::control::NodeDescriptor;

#[no_mangle]
pub fn get_name() -> String {
    "essentials".into()
}

#[no_mangle]
pub fn get_descriptors() -> Vec<NodeDescriptor> {
    vec![
        audio_io::audio_io(),
        stft::stft(),
        stft::istft(),
        stft::spectrogram_render(),
        pixel_scroller::pixel_scroller(),
    ]
}
