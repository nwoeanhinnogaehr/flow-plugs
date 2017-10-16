#![feature(catch_expr)]
#![feature(core_intrinsics)]
#![feature(duration_from_micros)]

extern crate apodize;
extern crate flow_synth;
extern crate jack;
extern crate modular_flow;
extern crate palette;
extern crate rustfft;
extern crate sdl2;

#[macro_use]
mod macros;
mod audio_io;
mod stft;
mod pixel_scroller;
mod basics;
mod synth;
mod bytebeat;

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
        basics::splitter(),
        basics::mixer::<f32>(),
        basics::clock(),
        basics::debug::<usize>(),
        basics::debug::<f32>(),
        synth::square(),
        synth::saw(),
        synth::sin(),
        bytebeat::beat("asketch", |t| {
            let x = t as f32;
            (x / (1.0 + x % (1 + (t >> 9 | t >> 13)) as f32) / 64.0).sin()
        }),
        bytebeat::beat("ksketch", |t| {
            let x = t as f32;
            let s = x / (1.0 + x % (1 + (t >> 5 | t >> 7)) as f32);
            s % 22050.0
        }),
        bytebeat::beat("asimple", |t| {
            let wrap = 64.0;
            (t & t >> 9 | t >> 13) as f32 % wrap / wrap * 2.0 - 1.0
        }),
        bytebeat::beat("ksimple", |t| {
            let s = t & t >> 5 | t >> 7;
            (1 << (s * s / 666) % 16) as f32 % 22050.0
        }),
    ]
}
