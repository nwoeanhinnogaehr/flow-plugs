use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;
use std::ops::AddAssign;
use std::thread;
use std::time::{Duration, Instant};
use std::fmt::Debug;

pub fn splitter() -> NodeDescriptor {
    NodeDescriptor::new("Splitter", run_splitter)
}

pub fn run_splitter(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    macros::simple_node(
        ctx,
        cfg,
        (1, 2),
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
            let lock = node_ctx.lock_all();
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node_ctx.node().push_out_port();
                    }
                    "Remove port" => {
                        node_ctx.node().pop_out_port(ctl.context().graph());
                    }
                    _ => panic!(),
                }
            }
            lock.wait(|lock| Ok(lock.available::<u8>(InPortID(0))? >= 1))?;
            let data = lock.read::<u8>(InPortID(0))?;
            if data.len() > 0 {
                for port in lock.node().out_ports() {
                    ignore_nonfatal!({
                        lock.write(port.id(), &data)?;
                    });
                }
            }
            Ok(())
        },
    )
}

pub fn mixer<T: ByteConvertible + Default + Copy + AddAssign + Send + 'static>() -> NodeDescriptor {
    NodeDescriptor::new(
        "Mixer::".to_string() + unsafe { ::std::intrinsics::type_name::<T>() },
        run_mixer::<T>,
    )
}

pub fn run_mixer<T: ByteConvertible + Default + Copy + AddAssign + Send + 'static>(
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
) -> Arc<RemoteControl> {
    let buffer_size = 1024;
    let mut accum = vec![T::default(); buffer_size];
    macros::simple_node(
        ctx,
        cfg,
        (2, 1),
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
            let lock = node_ctx.lock_all();
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Add port" => {
                        node_ctx.node().push_in_port();
                    }
                    "Remove port" => {
                        node_ctx.node().pop_in_port(ctl.context().graph());
                    }
                    _ => panic!(),
                }
            }
            let mut read_any = false;
            for port in lock.node().in_ports() {
                ignore_nonfatal!({
                    lock.wait(|lock| Ok(lock.available::<T>(port.id())? >= buffer_size))?;
                    let data = lock.read_n::<T>(port.id(), buffer_size)?;
                    for i in 0..buffer_size {
                        accum[i] += data[i];
                    }
                    read_any = true;
                });
            }
            if read_any {
                lock.write(OutPortID(0), &accum)?;
                for i in 0..buffer_size {
                    accum[i] = T::default();
                }
            }
            Ok(())
        },
    )
}


pub fn clock() -> NodeDescriptor {
    NodeDescriptor::new("Clock", run_clock)
}

pub fn run_clock(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let freq_port = InPortID(0);
    let bufsize_port = InPortID(1);
    let ticker_port = OutPortID(0);
    let mut buffer_size = 8;
    let mut ticker_buffer: Vec<_> = (0..buffer_size).collect();
    let mut period = 1000000; // 1 sec in us
    let start_time = Instant::now();
    let mut prev_quota = 0;
    let mut spawned = false;

    let set_bufsize = |buffer_size: &mut usize, ticker_buffer: &mut Vec<usize>, new_size: usize| {
        *buffer_size = new_size.min(1 << 20).max(1); // prevent OOM
        ticker_buffer.resize(*buffer_size, 0);
        let tick0 = ticker_buffer[0];
        for (idx, tick) in ticker_buffer.iter_mut().enumerate() {
            *tick = idx + tick0;
        }
    };
    let set_freq = |period: &mut u64, new_freq: f32| {
        *period = (1.0 / new_freq.max(0.0000001)) as u64;
    };

    let ctl = macros::simple_node(
        ctx,
        cfg,
        (2, 1),
        vec![
            message::Desc {
                name: "Set bufsize".into(),
                args: vec![
                    message::ArgDesc {
                        name: "".into(),
                        ty: message::Type::Usize,
                    },
                ],
            },
            message::Desc {
                name: "Set freq".into(),
                args: vec![
                    message::ArgDesc {
                        name: "".into(),
                        ty: message::Type::F32,
                    },
                ],
            },
        ],
        move |node_ctx, ctl| {
            if !spawned {
                let ctl = ctl.clone();
                thread::spawn(move || while !ctl.stopped() {
                    thread::sleep(Duration::from_micros(period));
                    ctl.node().notify(ctl.context().graph());
                });
                spawned = true;
            }
            while let Some(msg) = ctl.recv_message() {
                match msg.desc.name.as_str() {
                    "Set bufsize" => if let message::Value::Usize(new_size) = msg.args[0] {
                        set_bufsize(&mut buffer_size, &mut ticker_buffer, new_size);
                    },
                    "Set freq" => if let message::Value::F32(new_freq) = msg.args[0] {
                        set_freq(&mut period, new_freq);
                    },
                    _ => panic!(),
                }
            }
            let lock = node_ctx.lock_all();
            if let Ok(Some(new_size)) = lock.read::<usize>(bufsize_port)
                .map(|data| data.last().cloned())
            {
                set_bufsize(&mut buffer_size, &mut ticker_buffer, new_size);
            }
            if let Ok(Some(new_freq)) = lock.read::<f32>(freq_port).map(|data| data.last().cloned())
            {
                set_freq(&mut period, new_freq);
            }
            let quota = start_time.elapsed();
            let quota = quota.as_secs() * 1000000 + quota.subsec_nanos() as u64 / 1000;
            while prev_quota <= quota {
                lock.write(ticker_port, &ticker_buffer)?;
                for tick in &mut ticker_buffer {
                    *tick += buffer_size;
                }
                prev_quota += period;
            }
            Ok(())
        },
    );
    ctl.node().in_port(freq_port).unwrap().set_name("frequency");
    ctl.node()
        .in_port(bufsize_port)
        .unwrap()
        .set_name("bufsize");
    ctl.node().out_port(ticker_port).unwrap().set_name("ticker");
    ctl
}


pub fn debug<T: ByteConvertible + Debug + 'static>() -> NodeDescriptor {
    NodeDescriptor::new(
        "Debug::".to_string() + unsafe { ::std::intrinsics::type_name::<T>() },
        run_debug::<T>,
    )
}

pub fn run_debug<T: ByteConvertible + Debug>(
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
) -> Arc<RemoteControl> {
    macros::simple_node(ctx, cfg, (1, 0), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();
        lock.wait(|lock| Ok(lock.available::<T>(InPortID(0))? > 0))?;
        eprintln!("DBG: {:?}", lock.read::<T>(InPortID(0))?);
        Ok(())
    })
}

pub fn constant(ty: message::Type) -> NodeDescriptor {
    NodeDescriptor::new(
        format!("Const::{:?}", ty),
        move |ctx: Arc<Context>, cfg: NewNodeConfig| run_constant(ctx, cfg, ty.clone()),
    )
}

pub fn run_constant(
    ctx: Arc<Context>,
    cfg: NewNodeConfig,
    ty: message::Type,
) -> Arc<RemoteControl> {
    macros::simple_node(ctx, cfg, (0, 1), vec![
            message::Desc {
                name: "Set".into(),
                args: vec![
                    message::ArgDesc {
                        name: "value".into(),
                        ty: ty,
                    }
                ],
            },
    ], move |node_ctx, ctl| {
        let lock = node_ctx.lock_all();
        while let Some(msg) = ctl.recv_message() {
            match msg.desc.name.as_str() {
                "Set" => {
                    match msg.args[0] {
                        message::Value::Usize(x) => {
                            lock.write(OutPortID(0), &[x])?;
                        }
                        message::Value::F32(x) => {
                            lock.write(OutPortID(0), &[x])?;
                        }
                        _ => panic!(),
                    }
                }
                _ => panic!(),
            }
        }
        lock.sleep();
        Ok(())
    })
}

pub fn repeater<T: ByteConvertible + Default + Send + 'static>() -> NodeDescriptor {
    NodeDescriptor::new(
        format!("Repeater::{}", unsafe { ::std::intrinsics::type_name::<T>() }),
        run_repeater::<T>,
    )
}

fn run_repeater<T: ByteConvertible + Default + Send + 'static>(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    use std::collections::VecDeque;

    let mut buffer = Vec::new();
    let mut init = false;
    let mut cur_rep = 1;
    let mut rep_buf = VecDeque::new();
    let mut in_buf = VecDeque::new();
    let ctl = macros::simple_node(ctx, cfg, (3, 3), vec![], move |node_ctx, ctl| {
        if !init {
            if let Ok(data) = ctl.restore() {
                cur_rep = data;
            }
            init = true; // ugh
        }
        let lock = node_ctx.lock_all();

        // wait for request
        lock.wait(|lock| Ok(lock.available::<usize>(InPortID(0))? >= 1))?;
        let req_size = lock.read_n::<usize>(InPortID(0), 1)?[0];
        buffer.resize(req_size, T::default());

        let mut buf_idx = 0;
        while buf_idx < req_size {
            if let Some(rep) = rep_buf.front().cloned() {
                cur_rep = rep;
                ctl.save(cur_rep).unwrap();
                rep_buf.pop_front();
            } else {
                let write_req_result = lock.write(OutPortID(0), &[req_size]);
                let ok = write_req_result.is_ok();
                ignore_nonfatal!({write_req_result?});
                if ok {
                    lock.wait(|lock| Ok(lock.available::<usize>(InPortID(1))? >= req_size))?;
                }
                ignore_nonfatal!({
                    let reps = lock.read::<usize>(InPortID(1))?;
                    rep_buf.extend(&reps);
                });
            }

            if in_buf.is_empty() {
                lock.write(OutPortID(1), &[req_size])?;
                lock.wait(|lock| Ok(lock.available::<T>(InPortID(2))? >= req_size))?;
                let data = lock.read_n::<T>(InPortID(2), req_size)?;
                in_buf.extend(&data);
            }
            let sample = in_buf.front().cloned().unwrap();
            in_buf.pop_front();
            for _ in 0..cur_rep {
                buffer[buf_idx] = sample;

                buf_idx += 1;
                if buf_idx >= req_size {
                    break;
                }
            }
        }
        lock.write(OutPortID(2), &buffer)?;

        Ok(())
    });
    ctl.node().in_port(InPortID(0)).unwrap().set_name("req");
    ctl.node().in_port(InPortID(1)).unwrap().set_name("rep");
    ctl.node().in_port(InPortID(2)).unwrap().set_name("in");
    ctl.node().out_port(OutPortID(0)).unwrap().set_name("rep req");
    ctl.node().out_port(OutPortID(1)).unwrap().set_name("in req");
    ctl.node().out_port(OutPortID(2)).unwrap().set_name("out");
    ctl
}
