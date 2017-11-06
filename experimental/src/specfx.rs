use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;
use rustfft::num_complex::Complex;
use std::{f32, f64};
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

struct ReqQueue<T: ByteConvertible> {
    q: VecDeque<T>,
    in_port: InPortID,
    out_port: OutPortID,
    buffer_size: usize,
}

impl<T: ByteConvertible> ReqQueue<T> {
    fn new(in_port: InPortID, out_port: OutPortID, buffer_size: usize) -> ReqQueue<T> {
        ReqQueue {
            q: VecDeque::new(),
            in_port,
            out_port,
            buffer_size,
        }
    }
    fn next(&mut self, lock: &NodeGuard) -> Result<Option<T>> {
        if self.q.is_empty() {
            let write_result = lock.write(self.out_port, &[self.buffer_size]);
            let ok = write_result.is_ok();
            ignore_nonfatal!({ write_result? });
            if ok {
                lock.wait(|lock| {
                    Ok(lock.available::<T>(self.in_port)? >= self.buffer_size)
                })?;
            }
            ignore_nonfatal!({
                let vals = lock.read::<T>(self.in_port)?;
                self.q.extend(&vals);
            });
        }
        Ok(self.q.pop_front())
    }
}

enum PortConfig {
    InOneOutOne,
    InManyOutOne,
    InOneOutMany,
    InManyOutMany,
}

fn new_stftfx<
    ProcessFn: FnMut(&RemoteControl, &NodeGuard, Vec<STFTFrame>)
        -> Result<Vec<STFTFrame>>
        + Send
        + 'static,
>(
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
    num_aux_in: usize,
    num_aux_out: usize,
    port_config: PortConfig,
    mut process: ProcessFn,
) -> Arc<RemoteControl> {
    let mut size = 0;
    let mut messages = Vec::new();
    match port_config {
        PortConfig::InManyOutMany => messages.extend_from_slice(&[
            message::Desc {
                name: "Add in port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove in port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Add out port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove out port".into(),
                args: vec![],
            },
        ]),
        PortConfig::InManyOutOne => messages.extend_from_slice(&[
            message::Desc {
                name: "Add in port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove in port".into(),
                args: vec![],
            },
        ]),
        PortConfig::InOneOutMany => messages.extend_from_slice(&[
            message::Desc {
                name: "Add out port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove out port".into(),
                args: vec![],
            },
        ]),
        PortConfig::InOneOutOne => messages.extend_from_slice(&[
            message::Desc {
                name: "Add port".into(),
                args: vec![],
            },
            message::Desc {
                name: "Remove port".into(),
                args: vec![],
            },
        ]),
    };
    let ctl = macros::simple_node(
        ctx,
        cfg,
        (1 + num_aux_in, 1 + num_aux_out),
        messages,
        move |node_ctx, ctl| {
            let node = node_ctx.node();
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node.push_in_port();
                        node.push_out_port();
                    }
                    "Remove port" => if node.in_ports().len() > num_aux_in + 1
                        && node.out_ports().len() > num_aux_out + 1
                    {
                        node.pop_in_port(ctl.context().graph());
                        node.pop_out_port(ctl.context().graph());
                    },
                    "Add in port" => {
                        node.push_in_port();
                    }
                    "Add out port" => {
                        node.push_out_port();
                    }
                    "Remove in port" => if node.in_ports().len() > num_aux_in + 1 {
                        node.pop_in_port(ctl.context().graph());
                    },
                    "Remove out port" => if node.out_ports().len() > num_aux_out + 1 {
                        node.pop_out_port(ctl.context().graph());
                    },
                    _ => panic!(),
                }
            }
            let lock = node_ctx.lock_all();

            let frames: Result<Vec<STFTFrame>> = node.in_ports()[num_aux_in..]
                .iter()
                .map(|in_port| {
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
                })
                .collect();

            let frames = process(ctl, &lock, frames?)?;
            assert_eq!(frames.len(), node.out_ports().len() - num_aux_out);
            for (&(header, ref frame), out_port) in
                frames.iter().zip(&node.out_ports()[num_aux_out..])
            {
                lock.write(out_port.id(), &[header])?;
                lock.write(out_port.id(), &frame)?;
            }
            Ok(())
        },
    );
    ctl
}

pub fn freq_split() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.freq-split", move |ctx, cfg| {
        new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InOneOutMany,
            move |ctl, _, mut frames| {
                assert_eq!(frames.len(), 1);
                let (header, input) = frames.pop().unwrap();
                let num_splits = ctl.node().out_ports().len();
                let mut output = vec![(header, vec![Complex::<f32>::default(); input.len()]); num_splits];
                for (idx, bin) in input.iter().enumerate() {
                    output[(num_splits as f32 * (idx as f32 + 1.0).log2() / (input.len() as f32 + 1.0).log2()) as usize].1[idx] = *bin;
                }
                Ok(output)
            },
        )
    })
}

pub fn mix() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.mix", move |ctx, cfg| {
        let ctl = new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InManyOutOne,
            move |_, _, frames| {
                let (header, mut out) = frames[0].clone();
                for &(_, ref frame) in &frames[1..] {
                    for (in_bin, out_bin) in frame.iter().zip(out.iter_mut()) {
                        out_bin.re += in_bin.re;
                        out_bin.im += in_bin.im;
                    }
                }
                Ok(vec![(header, out)])
            },
        );
        ctl
    })
}

pub fn bin_max() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.bin-max", move |ctx, cfg| {
        bin_choose(|x, y| if x.norm() > y.norm() { x } else { y }, ctx, cfg)
    })
}
pub fn bin_min() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.bin-min", move |ctx, cfg| {
        bin_choose(|x, y| if x.norm() < y.norm() { x } else { y }, ctx, cfg)
    })
}

pub fn bin_choose<F: Fn(Complex<f32>, Complex<f32>) -> Complex<f32> + Send + 'static>(
    f: F,
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
) -> Arc<RemoteControl> {
    let ctl = new_stftfx(
        ctx,
        cfg,
        0,
        0,
        PortConfig::InManyOutOne,
        move |_, _, frames| {
            let (header, mut out) = frames[0].clone();
            for &(_, ref frame) in &frames[1..] {
                for (in_bin, out_bin) in frame.iter().zip(out.iter_mut()) {
                    *out_bin = f(*out_bin, *in_bin);
                }
            }
            Ok(vec![(header, out)])
        },
    );
    ctl
}

pub fn const_phase_mul() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.const-phase-mul", move |ctx, cfg| {
        let mut mul = 1.0;
        let mut init = false;
        let ctl = new_stftfx(
            ctx,
            cfg,
            1,
            0,
            PortConfig::InOneOutOne,
            move |ctl, lock, mut frames| {
                if !init {
                    // ugh
                    let _ = ctl.restore().map(|saved_mul| mul = saved_mul);
                    init = true;
                }
                if let Ok(new_mul) = lock.read_n::<f32>(InPortID(0), 1).map(|data| data[0]) {
                    mul = new_mul;
                    ctl.save(mul).unwrap();
                }
                for &mut (_, ref mut frame) in &mut frames {
                    for bin in frame {
                        let mut phase = bin.arg();
                        let amp = bin.norm();
                        phase *= mul;
                        *bin = Complex::<f32>::from_polar(&amp, &phase);
                    }
                }
                Ok(frames)
            },
        );
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
        let ctl = new_stftfx(
            ctx,
            cfg,
            2,
            0,
            PortConfig::InOneOutOne,
            move |ctl, lock, mut frames| {
                if !init {
                    // ugh
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
                for (&mut (_, ref mut frame), ref mut prev_frame) in
                    frames.iter_mut().zip(prev_frames.iter_mut())
                {
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
                Ok(frames)
            },
        );
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
        let ctl = new_stftfx(
            ctx,
            cfg,
            2,
            0,
            PortConfig::InOneOutOne,
            move |ctl, lock, mut frames| {
                if !init {
                    // ugh
                    let _ = ctl.restore().map(|saved_model| model = saved_model);
                    init = true;
                }
                if let Ok(new_size) = lock.read_n::<usize>(InPortID(0), 1).map(|data| data[0]) {
                    model.size = new_size.max(1).min(1 << 20);
                    ctl.save(&model).unwrap();
                }
                if let Ok(new_hop) = lock.read_n::<usize>(InPortID(1), 1).map(|data| data[0]) {
                    model.hop = new_hop.max(1).min(model.size);
                    ctl.save(&model).unwrap();
                }
                for &mut (ref mut header, ref mut frame) in &mut frames {
                    let arr = af::Array::new(&frame, af::Dim4::new(&[frame.len() as u64, 1, 1, 1]));
                    let out_arr = af::resize(&arr, model.size as i64, 1, af::InterpType::BILINEAR);
                    frame.resize(model.size, Complex::<f32>::default());
                    out_arr.host(frame);
                    header.size = model.size;
                    header.hop = model.hop;
                }
                Ok(frames)
            },
        );
        ctl
    })
}

pub fn rotate() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.rotate", move |ctx, cfg| {
        let mut a_rot = 0;
        let mut b_rot = 0;
        let mut a_q = ReqQueue::new(InPortID(0), OutPortID(0), 128);
        let mut b_q = ReqQueue::new(InPortID(1), OutPortID(1), 128);
        let ctl = new_stftfx(
            ctx,
            cfg,
            2,
            2,
            PortConfig::InOneOutOne,
            move |_, lock, mut frames| {
                if let Some(rot) = a_q.next(&lock)? {
                    a_rot = rot;
                }
                if let Some(rot) = b_q.next(&lock)? {
                    b_rot = rot;
                }

                for &mut (_, ref mut frame) in &mut frames {
                    let arr = af::Array::new(&frame, af::Dim4::new(&[frame.len() as u64, 1, 1, 1]));
                    let a = af::real(&arr);
                    let b = af::imag(&arr);
                    let a = af::shift(&a, &[a_rot as i32, 1, 1, 1]);
                    let b = af::shift(&b, &[b_rot as i32, 1, 1, 1]);
                    let out_arr = af::cplx2(&a, &b, false);
                    out_arr.host(frame);
                }
                Ok(frames)
            },
        );
        ctl.node().in_port(InPortID(0)).unwrap().set_name("a rot");
        ctl.node().in_port(InPortID(1)).unwrap().set_name("b rot");
        ctl.node()
            .out_port(OutPortID(0))
            .unwrap()
            .set_name("a rot req");
        ctl.node()
            .out_port(OutPortID(1))
            .unwrap()
            .set_name("b rot req");
        ctl
    })
}

pub fn backbuffer() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.backbuffer", move |ctx, cfg| {
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            size: usize,
        }
        let mut write_head = 0;
        let mut read_head = 0;
        let mut read_q = ReqQueue::new(InPortID(1), OutPortID(0), 128);
        let mut model = Model::default();
        let mut init = false;
        let mut buffers: Vec<Vec<_>> = Vec::new();
        let ctl = new_stftfx(
            ctx,
            cfg,
            2,
            1,
            PortConfig::InOneOutOne,
            move |ctl, lock, mut frames| {
                if !init {
                    // ugh
                    let _ = ctl.restore().map(|saved_model| model = saved_model);
                    init = true;
                }
                if let Ok(new_size) = lock.read_n::<usize>(InPortID(0), 1).map(|data| data[0]) {
                    model.size = new_size.max(1).min(1 << 16);
                    println!("bbuf set size {}", model.size);
                    ctl.save(&model).unwrap();
                }
                buffers.resize(frames.len(), Vec::new());
                for buffer in &mut buffers {
                    buffer.resize(model.size, Vec::new());
                }
                if let Some(read_pos) = read_q.next(&lock)? {
                    read_head = read_pos;
                }

                for (&mut (_, ref mut frame), ref mut buffer) in
                    frames.iter_mut().zip(buffers.iter_mut())
                {
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
                Ok(frames)
            },
        );
        ctl.node().in_port(InPortID(0)).unwrap().set_name("size");
        ctl.node()
            .in_port(InPortID(1))
            .unwrap()
            .set_name("read head");
        ctl.node()
            .out_port(OutPortID(0))
            .unwrap()
            .set_name("read head req");
        ctl
    })
}

pub fn to_polar() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.to-polar", move |ctx, cfg| {
        let ctl = new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InOneOutOne,
            move |_, _, mut frames| {
                for &mut (_, ref mut frame) in &mut frames {
                    for bin in frame {
                        let norm = bin.norm();
                        let arg = bin.arg();
                        *bin = Complex::<f32>::new(norm, arg);
                    }
                }
                Ok(frames)
            },
        );
        ctl
    })
}
pub fn from_polar() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.from-polar", move |ctx, cfg| {
        let ctl = new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InOneOutOne,
            move |_, _, mut frames| {
                for &mut (_, ref mut frame) in &mut frames {
                    for bin in frame {
                        let norm = bin.re;
                        let arg = bin.im;
                        *bin = Complex::<f32>::from_polar(&norm, &arg);
                    }
                }
                Ok(frames)
            },
        );
        ctl
    })
}


pub fn to_phase_diff() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.to-phase-diff", move |ctx, cfg| {
        let mut prev_phases = Vec::new();
        let ctl = new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InOneOutOne,
            move |_, _, mut frames| {
                prev_phases.resize(frames.len(), Vec::new());
                for (&mut (_, ref mut frame), ref mut prev_phase) in
                    frames.iter_mut().zip(prev_phases.iter_mut())
                {
                    prev_phase.resize(frame.len(), 0.0);
                    for (bin, prev) in frame.iter_mut().zip(prev_phase.iter_mut()) {
                        let norm = bin.norm();
                        let arg = bin.arg();
                        let diff = arg - *prev;
                        *prev = arg;
                        *bin = Complex::<f32>::new(norm, diff);
                    }
                }
                Ok(frames)
            },
        );
        ctl
    })
}
pub fn from_phase_diff() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.from-phase-diff", move |ctx, cfg| {
        let mut accum_phases = Vec::new();
        let ctl = new_stftfx(
            ctx,
            cfg,
            0,
            0,
            PortConfig::InOneOutOne,
            move |_, _, mut frames| {
                accum_phases.resize(frames.len(), Vec::new());
                for (&mut (_, ref mut frame), ref mut accum_phase) in
                    frames.iter_mut().zip(accum_phases.iter_mut())
                {
                    accum_phase.resize(frame.len(), 0.0);
                    for (bin, accum) in frame.iter_mut().zip(accum_phase.iter_mut()) {
                        let norm = bin.re;
                        let diff = bin.im;
                        *accum = (*accum + diff) % (f32::consts::PI * 2.0);
                        *bin = Complex::<f32>::from_polar(&norm, accum);
                    }
                }
                Ok(frames)
            },
        );
        ctl
    })
}

trait FreqOsc {
    fn f(freq: f32, phase: f32) -> Complex<f32>;
}

struct Sin;
impl FreqOsc for Sin {
    fn f(freq: f32, phase: f32) -> Complex<f32> {
        if freq == 0.0 {
            Complex::new(0.0, 0.0)
        } else {
            //let phase = phase % (32.0 * f32::consts::PI);
            Complex::from_polar(&(512.0/((freq+1.0)* 32.0+(phase*freq)%512.0)*(phase*freq).sin()), &(phase-freq%(phase%1.0+0.1)*1024.0))
        }
    }
}
struct Sin2;
impl FreqOsc for Sin2 {
    fn f(freq: f32, phase: f32) -> Complex<f32> {
        if freq == 0.0 {
            Complex::new(0.0, 0.0)
        } else {
            //let phase = phase % (32.0 * f32::consts::PI);
            Complex::from_polar(&(512.0/((freq+1.0)* 32.0+phase%512.0)), &(phase-freq%0.1*1024.0))
        }
    }
}
struct Sin3;
impl FreqOsc for Sin3 {
    fn f(freq: f32, phase: f32) -> Complex<f32> {
        if freq == 0.0 {
            Complex::new(0.0, 0.0)
        } else {
            //let phase = phase % (32.0 * f32::consts::PI);
            Complex::from_polar(&(512.0/((freq+0.1)* 32.0)*(freq*phase*32.0).sin()), &(freq*32.0))
        }
    }
}

pub fn fft_sin3_synth() -> NodeDescriptor {
    NodeDescriptor::new("STFT.sinSynth3", run_fft_synth::<Sin3>)
}
pub fn fft_sin2_synth() -> NodeDescriptor {
    NodeDescriptor::new("STFT.sinSynth2", run_fft_synth::<Sin2>)
}
pub fn fft_sin1_synth() -> NodeDescriptor {
    NodeDescriptor::new("STFT.sinSynth1", run_fft_synth::<Sin>)
}

fn run_fft_synth<O: FreqOsc>(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let mut phase = 0.0f64;
    let mut buffer = vec![];
    let mut freq_q = ReqQueue::new(InPortID(0), OutPortID(0), 128);
    let mut freq = 0.0f32;
    let ctl = macros::simple_node(ctx, cfg, (2, 2), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();

        // wait for request
        lock.wait(|lock| Ok(lock.available::<STFTHeader>(InPortID(1))? >= 1))?;
        let header = lock.read_n::<STFTHeader>(InPortID(1), 1)?[0];
        buffer.resize(header.size, Complex::<f32>::default());

        if let Some(new_freq) = freq_q.next(&lock)? {
            freq = new_freq;
        }
        let max = buffer.len() as f32;
        for (idx, x) in buffer.iter_mut().enumerate() {
            *x = O::f(idx as f32 / max, phase as f32);
            phase += 2.0 * freq as f64 * f64::consts::PI / 44100.0;
        }
        lock.write(OutPortID(1), &[header])?;
        lock.write(OutPortID(1), &buffer)?;
        Ok(())
    });
    ctl.node().in_port(InPortID(0)).unwrap().set_name("freq");
    ctl.node().in_port(InPortID(1)).unwrap().set_name("wave req");
    ctl.node().out_port(OutPortID(0)).unwrap().set_name("freq_req");
    ctl.node().out_port(OutPortID(1)).unwrap().set_name("wave");
    ctl
}

pub fn fft_driver() -> NodeDescriptor {
    NodeDescriptor::new("STFT driver", move |ctx, cfg| {
        let mut size_q = ReqQueue::new(InPortID(0), OutPortID(0), 1);
        let mut hop_q = ReqQueue::new(InPortID(1), OutPortID(1), 1);
        #[derive(Serialize, Deserialize, Default)]
        struct Model {
            size: usize,
            hop: usize,
        }
        let mut model = Model {
            size: 2048,
            hop: 512,
        };
        let mut quota = 0i64;
        let mut init = false;
        let ctl = macros::simple_node(ctx, cfg, (3, 3), vec![], move |node_ctx, ctl| {
            if !init {
                let _ = ctl.restore().map(|saved_model| model = saved_model);
                init = true;
            }
            let lock = node_ctx.lock_all();

            // wait for request
            lock.wait(|lock| Ok(lock.available::<usize>(InPortID(2))? >= 1))?;
            let req_size = lock.read_n::<usize>(InPortID(2), 1)?[0];
            quota += req_size as i64;

            if let Some(new_size) = size_q.next(&lock)? {
                model.size = new_size;
                ctl.save(&model).unwrap();
            }
            if let Some(new_hop) = hop_q.next(&lock)? {
                model.hop = new_hop;
                ctl.save(&model).unwrap();
            }
            let header = STFTHeader {
                size: model.size,
                hop: model.hop,
            };

            while quota > 0 {
                lock.write(OutPortID(2), &[header])?;
                quota -= model.hop as i64;
            }

            Ok(())
        });
        ctl.node().in_port(InPortID(0)).unwrap().set_name("size");
        ctl.node().in_port(InPortID(1)).unwrap().set_name("hop");
        ctl.node().in_port(InPortID(2)).unwrap().set_name("req");
        ctl.node().out_port(OutPortID(0)).unwrap().set_name("size req");
        ctl.node().out_port(OutPortID(1)).unwrap().set_name("hop req");
        ctl.node().out_port(OutPortID(2)).unwrap().set_name("out");
        ctl
    })
}
