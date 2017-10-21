use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;

#[derive(Debug, Copy, Clone)]
struct STFTHeader {
    size: usize,
    hop: usize,
}
unsafe impl TransmuteByteConvertible for STFTHeader {}

pub fn hold() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.hold", new_hold)
}
const MAX_SIZE: usize = 1 << 28;

fn new_hold(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let spectrum_in_port = InPortID(0);
    let spectrum_out_port = OutPortID(0);
    let mut size = 0;
    let mut time = 0;
    let mut buffer = vec![Complex::<f32>::zero(); size];
    let ctl = macros::simple_node(
        ctx,
        cfg,
        (1, 1),
        vec![],
        move |node_ctx, _| {
            let lock = node_ctx.lock_all();
            lock.wait(|lock| {
                Ok(lock.available::<STFTHeader>(spectrum_in_port)? >= 1)
            })?;
            let header = lock.read_n::<STFTHeader>(spectrum_in_port, 1)?[0];
            if size != header.size {
                if header.size >= MAX_SIZE {
                    println!("specfx.hold got header too large {:?}", header);
                    return Ok(());
                }
                println!("specfx.hold size change {:?}", header);
                size = header.size;
                buffer.resize(size, Complex::<f32>::zero());
            }
            lock.wait(|lock| {
                Ok(lock.available::<Complex<f32>>(spectrum_in_port)? >= size)
            })?;
            let mut frame = lock.read_n::<Complex<f32>>(spectrum_in_port, size)?;

            for (idx, bin) in frame.iter_mut().enumerate() {
                let mut phase = bin.arg();
                let amp = bin.norm();
                //phase -= ((idx+time)%size+1) as f32;
                phase *= -1.0;
                *bin = Complex::<f32>::from_polar(&amp, &phase);
            }

            lock.write(spectrum_out_port, &[header])?;
            lock.write(spectrum_out_port, &frame)?;
            time += 1;
            Ok(())
        },
    );
    ctl.node()
        .in_port(spectrum_in_port)
        .unwrap()
        .set_name("spec in");
    ctl.node()
        .out_port(spectrum_out_port)
        .unwrap()
        .set_name("spec out");
    ctl
}
