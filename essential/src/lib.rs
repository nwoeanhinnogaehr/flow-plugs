#![feature(catch_expr)]
#![feature(core_intrinsics)]
#![feature(duration_from_micros)]

extern crate apodize;
#[macro_use]
extern crate flow_synth;
extern crate jack;
extern crate modular_flow;
extern crate palette;
extern crate rustfft;
extern crate sdl2;
#[macro_use]
extern crate serde_derive;

mod audio_io;
mod stft;
mod pixel_scroller;
mod basics;
mod synth;

use flow_synth::control::{NodeDescriptor, message};

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
        basics::splitter(),
        basics::mixer::<f32>(),
        basics::clock(),
        basics::debug::<usize>(),
        basics::debug::<f32>(),
        basics::constant(message::Type::F32),
        basics::constant(message::Type::Usize),
        basics::repeater::<f32>(),
        synth::square(),
        synth::saw(),
        synth::sin(),
    ]
}
