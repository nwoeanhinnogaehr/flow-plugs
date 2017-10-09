use modular_flow::graph::*;

use modular_flow::context::*;
use std::thread;
use rustfft::FFTplanner;
use rustfft::num_complex::Complex;
use rustfft::num_traits::Zero;
use std::collections::VecDeque;
use apodize;
use std::iter;
use flow_synth::control::*;
use std::sync::Arc;

type T = f32; // fix this because generic numbers are so annoying

pub fn stft() -> NodeDescriptor {
    NodeDescriptor { name: "STFT".into(), new: new_stft }
}

fn new_stft(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(1, 1));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();

    // TODO add ports for params
    let size = 4096;
    let hop = 256;

    let remote_ctl = Arc::new(RemoteControl::new(
        ctx,
        node,
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
    ));

    let ctl = remote_ctl.clone();
    let window: Vec<T> = apodize::hanning_iter(size).map(|x| x.sqrt() as T).collect();
    thread::spawn(move || {
        let mut empty_q = VecDeque::<T>::new();
        empty_q.extend(vec![0.0; size - hop]);
        let mut queues = vec![empty_q.clone(); ctl.node().in_ports().len()];
        let mut input = vec![Complex::zero(); size];
        let mut output = vec![Complex::zero(); size];

        let mut planner = FFTplanner::new(false);
        let fft = planner.plan_fft(size);

        while !ctl.stopped() {
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node_ctx.node().push_in_port();
                        node_ctx.node().push_out_port();
                        queues.push(empty_q.clone());
                    }
                    "Remove port" => {
                        node_ctx.node().pop_in_port(ctl.context().graph());
                        node_ctx.node().pop_out_port(ctl.context().graph());
                        queues.pop();
                    }
                    _ => panic!(),
                }
            }
            node_ctx.lock_all().sleep(); // wait for next event
            let res: Result<()> = do catch {
                for ((in_port, out_port), queue) in node_ctx
                    .node()
                    .in_ports()
                    .iter()
                    .zip(node_ctx.node().out_ports())
                    .zip(queues.iter_mut())
                {
                    {
                        let lock = node_ctx.lock(&[in_port.clone()], &[]);
                        lock.wait(|lock| Ok(lock.available::<T>(in_port.id())? >= hop))?;
                        queue.extend(lock.read_n::<T>(in_port.id(), hop)?);
                    }

                    for ((dst, src), mul) in input.iter_mut().zip(queue.iter()).zip(&window) {
                        dst.re = *src * mul;
                        dst.im = 0.0;
                    }
                    queue.drain(..hop);
                    fft.process(&mut input, &mut output);

                    {
                        let lock = node_ctx.lock(&[], &[out_port.clone()]);
                        lock.write(out_port.id(), &output[..output.len() / 2])?;
                    }
                }
                Ok(())
            };
            if let Err(e) = res {
                println!("stft {:?}", e);
            }
        }
    });
    remote_ctl
}

pub fn istft() -> NodeDescriptor {
    NodeDescriptor { name: "ISTFT".into(), new: new_istft }
}

fn new_istft(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(1, 1));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();
    let remote_ctl = Arc::new(RemoteControl::new(
        ctx,
        node.clone(),
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
    ));
    let size = 4096;
    let hop = 256;
    let window: Vec<T> = apodize::hanning_iter(size).map(|x| x.sqrt() as T).collect();
    let ctl = remote_ctl.clone();
    thread::spawn(move || {
        let mut empty_q = VecDeque::<T>::new();
        empty_q.extend(vec![0.0; size - hop]);
        let mut queues = vec![empty_q.clone(); node.in_ports().len()];
        let mut output = vec![Complex::zero(); size];

        let mut planner = FFTplanner::new(true);
        let fft = planner.plan_fft(size);

        while !ctl.stopped() {
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node_ctx.node().push_in_port();
                        node_ctx.node().push_out_port();
                        queues.push(empty_q.clone());
                    }
                    "Remove port" => {
                        node_ctx.node().pop_in_port(ctl.context().graph());
                        node_ctx.node().pop_out_port(ctl.context().graph());
                        queues.pop();
                    }
                    _ => panic!(),
                }
            }
            node_ctx.lock_all().sleep(); // wait for next event
            let res: Result<()> = do catch {
                for ((in_port, out_port), queue) in node.in_ports()
                    .iter()
                    .zip(node.out_ports())
                    .zip(queues.iter_mut())
                {
                    let frame = {
                        let lock = node_ctx.lock(&[in_port.clone()], &[]);
                        lock.wait(|lock| {
                            Ok(lock.available::<Complex<T>>(in_port.id())? >= size / 2)
                        })?;
                        lock.read_n::<Complex<T>>(in_port.id(), size / 2)?
                    };
                    queue.extend(vec![0.0; hop]);
                    let mut input: Vec<_> = frame
                        .iter()
                        .cloned()
                        .chain(iter::repeat(Complex::zero()))
                        .take(size)
                        .collect();
                    fft.process(&mut input, &mut output);
                    for ((src, dst), window) in output.iter().zip(queue.iter_mut()).zip(&window) {
                        *dst += src.re * *window / size as T / (size / hop) as T * 2.0;
                    }
                    let samples = queue.drain(..hop).collect::<Vec<_>>();
                    node_ctx
                        .lock(&[], &[out_port.clone()])
                        .write(out_port.id(), &samples)?;
                }
                Ok(())
            };
            if let Err(e) = res {
                println!("istft {:?}", e);
            }
        }
    });
    remote_ctl
}


pub fn spectrogram_render() -> NodeDescriptor {
    NodeDescriptor { name: "SpectrogramRender".into(), new: new_specrogram_render }
}

fn new_specrogram_render(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(1, 1));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();
    let remote_ctl = Arc::new(RemoteControl::new(ctx, node.clone(), vec![]));
    let size = 2048;
    use palette::*;
    use palette::pixel::*;

    let mut max = 1.0;
    let mut prev_frame = vec![Complex::<T>::zero(); size];
    let ctl = remote_ctl.clone();
    thread::spawn(move || while !ctl.stopped() {
        let res: Result<()> = do catch {
            let frame = {
                let lock = node_ctx.lock(&[node.in_port(InPortID(0))?], &[]);
                lock.sleep();
                lock.wait(|lock| {
                    Ok(lock.available::<Complex<T>>(InPortID(0))? >= size)
                })?;
                lock.read_n::<Complex<T>>(InPortID(0), size)?
            };
            let out: Vec<_> = (0..size)
                .map(|idx| {
                    let x = (idx as T / size as T * (size as T).log2()).exp2();
                    let x_i = x as usize;

                    // compute hue
                    let hue1 = frame[x_i].arg() - prev_frame[x_i].arg();
                    let hue2 = frame[(x_i + 1) % size].arg() - prev_frame[(x_i + 1) % size].arg();

                    // compute intensity
                    let norm1 = frame[x_i].norm();
                    let norm2 = frame[(x_i + 1) % size].norm();
                    max = T::max(norm1, max);
                    max = T::max(norm2, max);
                    let value1 = norm1 / max;
                    let value2 = norm2 / max;

                    // output colour
                    let grad = Gradient::new(vec![
                        Hsv::new(RgbHue::from_radians(hue1), 1.0, value1),
                        Hsv::new(RgbHue::from_radians(hue2), 1.0, value2),
                    ]);
                    let (r, g, b, _): (T, T, T, T) = Srgb::linear_to_pixel(grad.get(x % 1.0));
                    let (r, g, b) = (r * 255.0, g * 255.0, b * 255.0);
                    (r as u32) << 24 | (g as u32) << 16 | (b as u32) << 8 | 0xFF
                })
                .collect();
            prev_frame = frame;
            {
                let lock = node_ctx.lock(&[], &[node.out_port(OutPortID(0))?]);
                lock.write(OutPortID(0), &out)?;
            }
            Ok(())
        };
        if let Err(e) = res {
            println!("render {:?}", e);
        }
    });
    remote_ctl
}
