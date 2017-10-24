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

const MAX_SIZE: usize = 1 << 28;

// need: process fn
// port resize fn
// buf resize fn
// num aux in
//
fn new_stftfx<ProcessFn: FnMut(&RemoteControl, &NodeGuard, usize, &mut [Complex<f32>]) + Send + 'static>(
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
    num_aux_in: usize,
    num_aux_out: usize,
    mut process: ProcessFn,
) -> Arc<RemoteControl> {
    let mut size = 0;
    let ctl = macros::simple_node(
        ctx,
        cfg,
        (1 + num_aux_in, 1 + num_aux_out),
        vec![
            message::Desc {
                name: "Add port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove port".into(),
                args: vec![],
            },
        ],
        move |node_ctx, ctl| {
            let node = node_ctx.node();
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node.push_in_port();
                        node.push_out_port();
                    }
                    "Remove port" => if node.in_ports().len() > num_aux_in
                        && node.out_ports().len() > num_aux_out
                    {
                        node.pop_in_port(ctl.context().graph());
                        node.pop_out_port(ctl.context().graph());
                    },
                    _ => panic!(),
                }
            }
            let lock = node_ctx.lock_all();

            for (in_port, out_port) in node.in_ports()[num_aux_in..]
                .iter()
                .zip(&node.out_ports()[num_aux_out..])
            {
                lock.wait(|lock| Ok(lock.available::<STFTHeader>(in_port.id())? >= 1))?;
                let header = lock.read_n::<STFTHeader>(in_port.id(), 1)?[0];
                if size != header.size {
                    if header.size >= MAX_SIZE {
                        println!("specfx.hold got header too large {:?}", header);
                        return Ok(());
                    }
                    println!("specfx.hold size change {:?}", header);
                    size = header.size;
                }
                lock.wait(|lock| {
                    Ok(lock.available::<Complex<f32>>(in_port.id())? >= size)
                })?;
                let mut frame = lock.read_n::<Complex<f32>>(in_port.id(), size)?;

                process(ctl, &lock, header.hop, &mut frame);

                lock.write(out_port.id(), &[header])?;
                lock.write(out_port.id(), &frame)?;
            }
            Ok(())
        },
    );
    ctl
}

pub fn const_phase_mul() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.const-phase-mul", move |ctx, cfg| {
        let mut mul = 1.0;
        let mut init = false;
        let ctl = new_stftfx(ctx, cfg, 1, 0, move |ctl, lock, _, frame| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_mul| mul = saved_mul);
                init = true;
            }
            if let Ok(new_mul) = lock.read_n::<f32>(InPortID(0), 1).map(|data| data[0]) {
                mul = new_mul;
                ctl.save(mul).unwrap();
            }
            for (idx, bin) in frame.iter_mut().enumerate() {
                let mut phase = bin.arg();
                let amp = bin.norm();
                phase *= mul;
                *bin = Complex::<f32>::from_polar(&amp, &phase);
            }
        });
        ctl
    })
}
