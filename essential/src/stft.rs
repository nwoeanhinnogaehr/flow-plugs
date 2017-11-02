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
    NodeDescriptor::new("STFT", new_stft)
}

#[derive(Copy, Clone, Debug)]
struct STFTHeader {
    pub size: usize,
    pub hop: usize,
}
unsafe impl TransmuteByteConvertible for STFTHeader {}

fn new_stft(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(3, 1));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();
    node.in_port(InPortID(0)).unwrap().set_name("size");
    node.in_port(InPortID(1)).unwrap().set_name("hop");

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
    remote_ctl.set_saved_data(&config.saved_data);
    #[derive(Debug, Serialize, Deserialize)]
    struct Model {
        size: usize,
        hop: usize,
    }
    let mut md = remote_ctl.restore().unwrap_or(Model {
        size: 4096,
        hop: 256,
    });

    let ctl = remote_ctl.clone();
    let mut window: Vec<T> = apodize::hanning_iter(md.size).map(|x| x.sqrt() as T).collect();
    thread::spawn(move || {
        let mut empty_q = VecDeque::<T>::new();
        empty_q.extend(vec![0.0; md.size - md.hop]);
        let mut queues = vec![empty_q.clone(); ctl.node().in_ports().len()];
        let mut input = vec![Complex::zero(); md.size];
        let mut output = vec![Complex::zero(); md.size];

        let mut planner = FFTplanner::new(false);
        let mut fft = planner.plan_fft(md.size);

        while !ctl.stopped() {
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node_ctx.node().push_in_port();
                        node_ctx.node().push_out_port();
                        queues.push(empty_q.clone());
                    }
                    "Remove port" => {
                        if node_ctx.node().in_ports().len() > 3 {
                            node_ctx.node().pop_in_port(ctl.context().graph());
                            node_ctx.node().pop_out_port(ctl.context().graph());
                            queues.pop();
                        }
                    }
                    _ => panic!(),
                }
            }
            let lock = node_ctx.lock_all();
            if let Ok(new_size) = lock.read_n::<usize>(InPortID(0), 1).map(|data| data[0]) {
                md.size = (new_size & !1).max(1).min(1<<20);
                md.hop = md.hop.min(md.size);
                println!("Stft resize {:?}", md);
                ctl.save(&md).unwrap();
                fft = planner.plan_fft(md.size);
                input.resize(md.size, Complex::zero());
                output.resize(md.size, Complex::zero());
                empty_q.resize(md.size - md.hop, 0.0);
                queues.iter_mut().for_each(|q| q.resize(md.size - md.hop, 0.0));
                window = apodize::hanning_iter(md.size).map(|x| x.sqrt() as T).collect();
            }
            if let Ok(new_hop) = lock.read_n::<usize>(InPortID(1), 1).map(|data| data[0]) {
                md.hop = new_hop.max(1).min(md.size);
                println!("Stft resize {:?}", md);
                ctl.save(&md).unwrap();
                empty_q.resize(md.size - md.hop, 0.0);
                queues.iter_mut().for_each(|q| q.resize(md.size - md.hop, 0.0));
            }
            let res: Result<()> = do catch {
                for ((in_port, out_port), queue) in node_ctx
                    .node()
                    .in_ports()[2..]
                    .iter()
                    .zip(node_ctx.node().out_ports())
                    .zip(queues.iter_mut())
                {
                    lock.wait(|lock| Ok(lock.available::<T>(in_port.id())? >= md.hop))?;
                    queue.extend(lock.read_n::<T>(in_port.id(), md.hop)?);

                    for ((dst, src), mul) in input.iter_mut().zip(queue.iter()).zip(&window) {
                        dst.re = *src * mul;
                        dst.im = 0.0;
                    }
                    queue.drain(..md.hop);
                    fft.process(&mut input, &mut output);

                    let header = STFTHeader {
                        size: md.size / 2,
                        hop: md.hop,
                    };
                    lock.write(out_port.id(), &[header])?;
                    lock.write(out_port.id(), &output[..output.len() / 2])?;
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
    NodeDescriptor::new("ISTFT", new_istft)
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
    let mut size = 4096;
    let mut hop = 256;
    let max_size = 1 << 20;
    let mut window: Vec<T> = apodize::hanning_iter(size).map(|x| x.sqrt() as T).collect();
    let ctl = remote_ctl.clone();
    thread::spawn(move || {
        let mut empty_q = VecDeque::<T>::new();
        empty_q.extend(vec![0.0; size - hop]);
        let mut queues = vec![empty_q.clone(); node.in_ports().len()];
        let mut output = vec![Complex::zero(); size];

        let mut planner = FFTplanner::new(true);
        let mut fft = planner.plan_fft(size);

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
            let lock = node_ctx.lock_all();
            let res: Result<()> = do catch {
                for ((idx, in_port), out_port) in
                    node.in_ports().iter().enumerate().zip(node.out_ports())
                {
                    let frame = {
                        lock.wait(|lock| Ok(lock.available::<STFTHeader>(in_port.id())? >= 1))?;
                        let header = lock.read_n::<STFTHeader>(in_port.id(), 1)?[0];
                        if header.size > max_size || header.hop > max_size {
                            eprintln!("istft got header too large {:?}", header);
                            continue;
                        }
                        if header.hop != hop || header.size * 2 != size {
                            println!("istft size change {:?}", header);
                            hop = header.hop;
                            size = header.size * 2;
                            empty_q = VecDeque::<T>::new();
                            empty_q.extend(vec![0.0; size - hop]);
                            queues = vec![empty_q.clone(); node.in_ports().len()];
                            fft = planner.plan_fft(size);
                            output.resize(size, Complex::zero());
                            window = apodize::hanning_iter(size).map(|x| x.sqrt() as T).collect();
                        }
                        lock.wait(|lock| {
                            Ok(lock.available::<Complex<T>>(in_port.id())? >= size / 2)
                        })?;
                        lock.read_n::<Complex<T>>(in_port.id(), size / 2)?
                    };
                    let queue = &mut queues[idx];
                    queue.extend(vec![0.0; hop]);
                    let mut input: Vec<_> = frame
                        .iter()
                        .cloned()
                        .chain(iter::repeat(Complex::zero()))
                        .take(size)
                        .collect();
                    fft.process(&mut input, &mut output);
                    for ((src, dst), window) in output.iter().zip(queue.iter_mut()).zip(&window) {
                        // lol what is this some kind of normalization heuristic?!
                        *dst += src.re * *window / size as T / (size / hop) as T * 2.0;
                    }
                    let samples = queue.drain(..hop).collect::<Vec<_>>();
                    lock.write(out_port.id(), &samples)?;
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
    NodeDescriptor::new("SpectrogramRender", new_specrogram_render)
}

fn new_specrogram_render(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(1, 1));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();
    let remote_ctl = Arc::new(RemoteControl::new(ctx, node.clone(), vec![]));
    let mut size = 2048;
    use palette::*;
    use palette::pixel::*;

    let mut max = 1.0;
    let mut prev_frame = vec![Complex::<T>::zero(); size];
    let ctl = remote_ctl.clone();
    thread::spawn(move || while !ctl.stopped() {
        let res: Result<()> = do catch {
            let lock = node_ctx.lock_all();
            let frame = {
                lock.wait(|lock| Ok(lock.available::<STFTHeader>(InPortID(0))? >= 1))?;
                let header = lock.read_n::<STFTHeader>(InPortID(0), 1)?[0];
                if size != header.size {
                    if header.size >= 8192 {
                        println!("render got header too large {:?}", header);
                        continue;
                    }
                    println!("render size change {:?}", header);
                    size = header.size;
                    prev_frame.resize(size, Complex::<T>::zero());
                    max = 1.0;
                }
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
            lock.write(OutPortID(0), &[size])?;
            lock.write(OutPortID(0), &out)?;
            Ok(())
        };
        if let Err(e) = res {
            println!("render {:?}", e);
        }
    });
    remote_ctl
}
