#![feature(catch_expr)]
#![feature(core_intrinsics)]
#![feature(duration_from_micros)]

#[macro_use]
extern crate flow_synth;
extern crate modular_flow;
extern crate rustfft;

mod bytebeat;
mod specfx;

use flow_synth::control::NodeDescriptor;

#[no_mangle]
pub fn get_name() -> String {
    "experimental".into()
}

#[no_mangle]
pub fn get_descriptors() -> Vec<NodeDescriptor> {
    vec![
        bytebeat::beat("asketch", |t| {
            let x = t as f32;
            (x / (1.0 + x % (1 + (t >> 9 | t >> 13)) as f32) / 64.0).sin()
        }),
        bytebeat::beat("asketch2", |t| {
            let x = t as f32;
            let z = (1 + (t >> 9 | t >> 13)) as f32;
            (x / (1.0 + (x % z - z / 2.0).abs()) / 256.0).sin()
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
        bytebeat::beat("kadv", |t| {
            let s = t & t >> 5 | t >> 7;
            (1.3f32).powf((s * s / 666) as f32  % 32.0) % 22050.0
        }),
        bytebeat::beat("ksimplei", |t| {
            let s = t & t >> 5 | t >> 7;
            (s % 256) as usize
        }),
        specfx::const_phase_mul(),
    ]
}
