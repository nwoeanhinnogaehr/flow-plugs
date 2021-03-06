#![feature(catch_expr)]
#![feature(core_intrinsics)]
#![feature(duration_from_micros)]

extern crate arrayfire;
#[macro_use]
extern crate flow_synth;
extern crate modular_flow;
extern crate rustfft;
#[macro_use]
extern crate serde_derive;
use arrayfire as af;

mod bytebeat;
mod specfx;

use flow_synth::control::NodeDescriptor;

#[no_mangle]
pub fn get_name() -> String {
    "experimental".into()
}

#[no_mangle]
pub fn get_descriptors() -> Vec<NodeDescriptor> {
    af::init();
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
        bytebeat::beat("asketchi", |t| {
            let x = t as f32 * 9.5734857;
            (x / (1.0 + x % (1 + (t >> 9 | t >> 13)) as f32)) as usize
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
        bytebeat::beat("ksimple12t", |t| {
            let s = t & t >> 8 | t >> 10;
            (((s * s / 222) % 256) as f32 / 12.0).exp2() % 22050.0
        }),
        bytebeat::beat("ksimple12t-alt", |t| {
            let s = t & t >> 8 | t >> 10;
            (((s * s / 288) % 256) as f32 / 12.0).exp2() % 22050.0
        }),
        bytebeat::beat("kadv", |t| {
            let s = t & t >> 5 | t >> 7;
            (1.3f32).powf((s * s / 666) as f32 % 32.0) % 22050.0
        }),
        bytebeat::beat("kadv2", |t| {
            let s = t & t >> 5 | t >> 7;
            (1.2f32).powf((s * s / 431) as f32 % 64.0) % 22050.0
        }),
        bytebeat::beat("kforfft", |t| {
            let s = t & t >> 3 | t >> 5 ^ t >> 8;
            (1.5f32).powf((s * s / 666) as f32 % 32.0) % 22050.0
        }),
        bytebeat::beat("kforfftp100", |t| {
            let s = t & t >> 3 | t >> 5 ^ t >> 8;
            (1.5f32).powf((s * s / 666) as f32 % 32.0) % 22050.0 + 100.0
        }),
        bytebeat::beat("kforfftp110", |t| {
            let s = t & t >> 3 | t >> 5 ^ t >> 8;
            (1.5f32).powf((s * s / 666) as f32 % 32.0) % 22050.0 + 102.0
        }),
        bytebeat::beat("ksimplei", |t| {
            let s = t & t >> 5 | t >> 7;
            (s % 256) as usize
        }),
        bytebeat::beat("nov1", |t| {
            let s = t/(1+(t*((t>>15|t>>17^t>>13)&31)&t>>5^t>>9)%(1+(t>>4|t>>12)));
            (s as f32 / 32.0).sin()*0.25
        }),
        bytebeat::beat("nov1-2", |t| {
            let s = t/(1+(t*((t>>15|t>>17^t>>13)&31)&t>>5^t>>8)%(2+(t>>4|t>>12)));
            (s as f32 / 32.0).sin()*0.25
        }),
        bytebeat::beat("nov1-unpitch", |t| {
            let s = t/(1+(t*(((t&t>>14|t>>17^t>>12)&127) as f32 /12.0).exp2() as usize&t>>5^t>>8)%(1+(t>>6|t>>12)));
            let s2 = t/(1+(t*(((t>>14^t>>17^t>>12)&127) as f32 /12.0).exp2() as usize&t>>5^t>>8)%(1+(t>>6|t>>12)));
            (s as f32 / 32.0).sin()*0.125 +
            (s2 as f32 / 32.0).sin()*0.125
        }),
        bytebeat::beat("nov1-2-unpitch", |t| {
            let s = t/(1+(t*(((t&t>>14|t>>17^t>>12)&127) as f32 /12.0).exp2() as usize&t>>5^t>>8)%(1+(t>>7|t>>12)));
            let s2 = t/(1+(t*(((t>>14^t>>17^t>>12)&127) as f32 /12.0).exp2() as usize&t>>5^t>>8)%(1+(t>>7|t>>12)));
            (s as f32 / 32.0).sin()*0.125 +
            (s2 as f32 / 32.0).sin()*0.125
        }),
        bytebeat::beat("nov1-unpitch-old", |t| {
            let s = t/(1+(t*((t&t>>15|t>>17^t>>13))&t>>5^t>>9)%(1+(t>>4|t>>12)));
            (s as f32 / 32.0).sin()*0.25
        }),
        bytebeat::beat("nov1-2-unpitch-old", |t| {
            let s = t/(1+(t*((t&t>>15|t>>17^t>>13))&t>>5^t>>8)%(2+(t>>4|t>>12)));
            (s as f32 / 32.0).sin()*0.25
        }),
        bytebeat::beat("nov2", |t| {
            let t = t % 65536 + 1243937 + (t >> 16 ^ t >> 18)*4096;
            let x = t as f32 * 0.5;
            (x / (1.0 + x % (1 + (t >> 9 | t >> 13)) as f32) / 256.0).sin()*0.5
        }),

        bytebeat::beat("counter", |t| t as usize),
        specfx::const_phase_mul(),
        specfx::hold(),
        specfx::to_polar(),
        specfx::from_polar(),
        specfx::to_phase_diff(),
        specfx::from_phase_diff(),
        specfx::backbuffer(),
        specfx::resize(),
        specfx::rotate(),
        specfx::mix(),
        specfx::bin_min(),
        specfx::bin_max(),
        specfx::freq_split(),
        specfx::fft_sin1_synth(),
        specfx::fft_sin2_synth(),
        specfx::fft_sin3_synth(),
        specfx::fft_driver(),
    ]
}
