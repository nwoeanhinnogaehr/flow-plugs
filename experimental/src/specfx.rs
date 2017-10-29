use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;
use rustfft::num_complex::Complex;
use std::f32;
use std::collections::VecDeque;
use arrayfire as af;

#[derive(Debug, Copy, Clone)]
struct STFTHeader {
    size: usize,
    hop: usize,
}
unsafe impl TransmuteByteConvertible for STFTHeader {}

const MAX_SIZE: usize = 1 << 28;

type STFTFrame = (STFTHeader, Vec<Complex<f32>>);

fn new_stftfx<ProcessFn: FnMut(&RemoteControl, &NodeGuard, &mut [STFTFrame]) -> Result<()> + Send + 'static>(
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

            let frames: Result<Vec<STFTFrame>> = node.in_ports()[num_aux_in..].iter().map(|in_port| {
                lock.wait(|lock| Ok(lock.available::<STFTHeader>(in_port.id())? >= 1))?;
                let header = lock.read_n::<STFTHeader>(in_port.id(), 1)?[0];
                if size != header.size {
                    if header.size >= MAX_SIZE {
                        println!("specfx got header too large {:?}", header);
                        return Err(Error::Unavailable);
                    }
                    println!("specfx size change {:?}", header);
                    size = header.size;
                }
                lock.wait(|lock| {
                    Ok(lock.available::<Complex<f32>>(in_port.id())? >= size)
                })?;
                Ok((header, lock.read_n::<Complex<f32>>(in_port.id(), size)?))
            }).collect();
            let mut frames = frames?;

            process(ctl, &lock, &mut frames)?;

            for (&(header, ref frame), out_port) in frames.iter()
                .zip(&node.out_ports()[num_aux_out..])
            {
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
        let ctl = new_stftfx(ctx, cfg, 1, 0, move |ctl, lock, frames| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_mul| mul = saved_mul);
                init = true;
            }
            if let Ok(new_mul) = lock.read_n::<f32>(InPortID(0), 1).map(|data| data[0]) {
                mul = new_mul;
                ctl.save(mul).unwrap();
            }
            for &mut (_, ref mut frame) in frames {
                for bin in frame {
                    let mut phase = bin.arg();
                    let amp = bin.norm();
                    phase *= mul;
                    *bin = Complex::<f32>::from_polar(&amp, &phase);
                }
            }
            Ok(())
        });
        ctl
    })
}

pub fn hold() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.hold", move |ctx, cfg| {
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            a_mix: f32,
            b_mix: f32,
        }
        let mut model = Model::default();
        let mut init = false;
        let mut prev_frames = vec![];
        let ctl = new_stftfx(ctx, cfg, 2, 0, move |ctl, lock, frames| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_model| model = saved_model);
                init = true;
            }
            if let Ok(new_a_mix) = lock.read_n::<f32>(InPortID(0), 1).map(|data| data[0]) {
                model.a_mix = new_a_mix.min(1.0).max(0.0);
                ctl.save(&model).unwrap();
            }
            if let Ok(new_b_mix) = lock.read_n::<f32>(InPortID(1), 1).map(|data| data[0]) {
                model.b_mix = new_b_mix.min(1.0).max(0.0);
                ctl.save(&model).unwrap();
            }
            prev_frames.resize(frames.len(), Vec::new());
            for (&mut (_, ref mut frame), ref mut prev_frame) in frames.iter_mut().zip(prev_frames.iter_mut()) {
                prev_frame.resize(frame.len(), Complex::<f32>::default());
                for (bin, prev) in frame.iter_mut().zip(prev_frame.iter()) {
                    let mut a = bin.re;
                    let mut b = bin.im;
                    let mut prev_a = prev.re;
                    let mut prev_b = prev.im;
                    a = (1.0 - model.a_mix) * a + model.a_mix * prev_a;
                    b = (1.0 - model.b_mix) * b + model.b_mix * prev_b;
                    *bin = Complex::<f32>::new(a, b);
                }
                prev_frame.clone_from_slice(&frame);
            }
            Ok(())
        });
        ctl
    })
}

pub fn resize() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.resize", move |ctx, cfg| {
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            size: usize,
            hop: usize,
        }
        let mut model = Model {
            size: 2048,
            hop: 512,
        };
        let mut init = false;
        let ctl = new_stftfx(ctx, cfg, 2, 0, move |ctl, lock, frames| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_model| model = saved_model);
                init = true;
            }
            if let Ok(new_size) = lock.read_n::<usize>(InPortID(0), 1).map(|data| data[0]) {
                model.size = new_size.max(1).min(1<<20);
                ctl.save(&model).unwrap();
            }
            if let Ok(new_hop) = lock.read_n::<usize>(InPortID(1), 1).map(|data| data[0]) {
                model.hop = new_hop.max(1).min(model.size);
                ctl.save(&model).unwrap();
            }
            for &mut (ref mut header, ref mut frame) in frames {
                let arr = af::Array::new(&frame, af::Dim4::new(&[frame.len() as u64, 1, 1, 1]));
                let out_arr = af::resize(&arr, model.size as i64, 1, af::InterpType::BILINEAR);
                frame.resize(model.size, Complex::<f32>::default());
                out_arr.host(frame);
                header.size = model.size;
                header.hop = model.hop;
            }
            Ok(())
        });
        ctl
    })
}

pub fn rotate() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.rotate", move |ctx, cfg| {
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            a_rot: usize,
            b_rot: usize,
        }
        let buffer_size = 128;
        let mut a_q = VecDeque::new();
        let mut b_q = VecDeque::new();
        let mut model = Model::default();
        let mut init = false;
        let ctl = new_stftfx(ctx, cfg, 2, 2, move |ctl, lock, frames| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_model| model = saved_model);
                init = true;
            }
            if a_q.is_empty() {
                let write_req_result = lock.write(OutPortID(0), &[buffer_size]);
                let ok = write_req_result.is_ok();
                ignore_nonfatal!({write_req_result?});
                if ok {
                    lock.wait(|lock| Ok(lock.available::<usize>(InPortID(0))? >= buffer_size))?;
                }
                ignore_nonfatal!({
                    let reps = lock.read::<usize>(InPortID(0))?;
                    a_q.extend(&reps);
                });
            }
            if let Some(rot) = a_q.front().cloned() {
                model.a_rot = rot;
                a_q.pop_front();
            }
            if b_q.is_empty() {
                let write_req_result = lock.write(OutPortID(1), &[buffer_size]);
                let ok = write_req_result.is_ok();
                ignore_nonfatal!({write_req_result?});
                if ok {
                    lock.wait(|lock| Ok(lock.available::<usize>(InPortID(1))? >= buffer_size))?;
                }
                ignore_nonfatal!({
                    let reps = lock.read::<usize>(InPortID(1))?;
                    b_q.extend(&reps);
                });
            }
            if let Some(rot) = b_q.front().cloned() {
                model.b_rot = rot;
                b_q.pop_front();
            }

            for &mut (_, ref mut frame) in frames {
                let arr = af::Array::new(&frame, af::Dim4::new(&[frame.len() as u64, 1, 1, 1]));
                let a = af::real(&arr);
                let b = af::imag(&arr);
                let a = af::shift(&a, &[model.a_rot as i32, 1, 1, 1]);
                let b = af::shift(&b, &[model.b_rot as i32, 1, 1, 1]);
                let out_arr = af::cplx2(&a, &b, false);
                out_arr.host(frame);
            }
            Ok(())
        });
        ctl.node().in_port(InPortID(0)).unwrap().set_name("a rot");
        ctl.node().in_port(InPortID(1)).unwrap().set_name("b rot");
        ctl.node().out_port(OutPortID(0)).unwrap().set_name("a rot req");
        ctl.node().out_port(OutPortID(1)).unwrap().set_name("b rot req");
        ctl
    })
}


pub fn backbuffer() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.backbuffer", move |ctx, cfg| {
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            size: usize,
        }
        let buffer_size = 128;
        let mut write_head = 0;
        let mut read_head = 0;
        let mut read_q = VecDeque::new();
        let mut model = Model::default();
        let mut init = false;
        let mut buffers: Vec<Vec<_>> = Vec::new();
        let ctl = new_stftfx(ctx, cfg, 2, 1, move |ctl, lock, frames| {
            if !init { // ugh
                let _ = ctl.restore().map(|saved_model| model = saved_model);
                init = true;
            }
            if let Ok(new_size) = lock.read_n::<usize>(InPortID(0), 1).map(|data| data[0]) {
                model.size = new_size.max(1).min(1<<16);
                println!("bbuf set size {}", model.size);
                ctl.save(&model).unwrap();
            }
            buffers.resize(frames.len(), Vec::new());
            for buffer in &mut buffers {
                buffer.resize(model.size, Vec::new());
            }
            if read_q.is_empty() {
                let write_req_result = lock.write(OutPortID(0), &[buffer_size]);
                let ok = write_req_result.is_ok();
                ignore_nonfatal!({write_req_result?});
                if ok {
                    lock.wait(|lock| Ok(lock.available::<usize>(InPortID(1))? >= buffer_size))?;
                }
                ignore_nonfatal!({
                    let reps = lock.read::<usize>(InPortID(1))?;
                    read_q.extend(&reps);
                });
            }
            if let Some(read_pos) = read_q.front().cloned() {
                read_head = read_pos;
                read_q.pop_front();
            }

            for (&mut (_, ref mut frame), ref mut buffer) in frames.iter_mut().zip(buffers.iter_mut()) {
                let len = buffer.len();
                if len == 0 {
                    continue;
                }
                // write to buffer
                {
                    let write_buffer = &mut buffer[write_head % len];
                    write_buffer.resize(frame.len(), Complex::<f32>::default());
                    write_buffer.clone_from_slice(&frame);
                }

                // read from buffer
                {
                    let read_buffer = &mut buffer[(write_head + read_head) % len];
                    read_buffer.resize(frame.len(), Complex::<f32>::default());
                    frame.clone_from_slice(read_buffer);
                }
            }
            write_head += 1;
            Ok(())
        });
        ctl.node().in_port(InPortID(0)).unwrap().set_name("size");
        ctl.node().in_port(InPortID(1)).unwrap().set_name("read head");
        ctl.node().out_port(OutPortID(0)).unwrap().set_name("read head req");
        ctl
    })
}

pub fn to_polar() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.to-polar", move |ctx, cfg| {
        let ctl = new_stftfx(ctx, cfg, 0, 0, move |_, _, frames| {
            for &mut (_, ref mut frame) in frames {
                for bin in frame {
                    let norm = bin.norm();
                    let arg = bin.arg();
                    *bin = Complex::<f32>::new(norm, arg);
                }
            }
            Ok(())
        });
        ctl
    })
}
pub fn from_polar() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.from-polar", move |ctx, cfg| {
        let ctl = new_stftfx(ctx, cfg, 0, 0, move |_, _, frames| {
            for &mut (_, ref mut frame) in frames {
                for bin in frame {
                    let norm = bin.re;
                    let arg = bin.im;
                    *bin = Complex::<f32>::from_polar(&norm, &arg);
                }
            }
            Ok(())
        });
        ctl
    })
}


pub fn to_phase_diff() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.to-phase-diff", move |ctx, cfg| {
        let mut prev_phases = Vec::new();
        let ctl = new_stftfx(ctx, cfg, 0, 0, move |_, _, frames| {
            prev_phases.resize(frames.len(), Vec::new());
            for (&mut (_, ref mut frame), ref mut prev_phase) in frames.iter_mut().zip(prev_phases.iter_mut()) {
                prev_phase.resize(frame.len(), 0.0);
                for (bin, prev) in frame.iter_mut().zip(prev_phase.iter_mut()) {
                    let norm = bin.norm();
                    let arg = bin.arg();
                    let diff = arg - *prev;
                    *prev = arg;
                    *bin = Complex::<f32>::new(norm, diff);
                }
            }
            Ok(())
        });
        ctl
    })
}
pub fn from_phase_diff() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.from-phase-diff", move |ctx, cfg| {
        let mut accum_phases = Vec::new();
        let ctl = new_stftfx(ctx, cfg, 0, 0, move |_, _, frames| {
            accum_phases.resize(frames.len(), Vec::new());
            for (&mut (_, ref mut frame), ref mut accum_phase) in frames.iter_mut().zip(accum_phases.iter_mut()) {
                accum_phase.resize(frame.len(), 0.0);
                for (bin, accum) in frame.iter_mut().zip(accum_phase.iter_mut()) {
                    let norm = bin.re;
                    let diff = bin.im;
                    *accum = (*accum + diff) % (f32::consts::PI * 2.0);
                    *bin = Complex::<f32>::from_polar(&norm, accum);
                }
            }
            Ok(())
        });
        ctl
    })
}
