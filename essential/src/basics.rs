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
    let max_buffered = 65536;
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
            let data = lock.read::<u8>(InPortID(0))?;
            if data.len() > 0 {
                for port in lock.node().out_ports() {
                    ignore_nonfatal!({
                        lock.wait(|lock| Ok(lock.buffered::<u8>(port.id())? <= max_buffered))?;
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
                        ty: message::Type::Int,
                    },
                ],
            },
            message::Desc {
                name: "Set freq".into(),
                args: vec![
                    message::ArgDesc {
                        name: "".into(),
                        ty: message::Type::Float,
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
                    "Set bufsize" => if let message::Value::Int(new_size) = msg.args[0] {
                        set_bufsize(&mut buffer_size, &mut ticker_buffer, new_size as usize);
                    },
                    "Set freq" => if let message::Value::Float(new_freq) = msg.args[0] {
                        set_freq(&mut period, new_freq as f32);
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
