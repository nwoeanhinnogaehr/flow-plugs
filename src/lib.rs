#![feature(catch_expr)]

extern crate modular_flow;
extern crate flow_synth;
extern crate apodize;
extern crate jack;
extern crate palette;
extern crate rustfft;
extern crate sdl2;

mod audio_io;
mod stft;
mod pixel_scroller;

use flow_synth::control::NodeDescriptor;

#[no_mangle]
pub extern "Rust" fn get_name() -> &'static str {
    "essentials"
}

#[no_mangle]
pub extern "Rust" fn get_descriptors() -> Vec<NodeDescriptor> {
    vec![
        audio_io::AUDIO_IO,
        stft::STFT,
        stft::ISTFT,
        stft::SPECTROGRAM_RENDER,
        pixel_scroller::PIXEL_SCROLLER,
    ]
}

