use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use std::sync::Arc;
use std::ops::AddAssign;
use macros;

pub fn splitter() -> NodeDescriptor {
    NodeDescriptor {
        name: "Splitter".into(),
        new: run_splitter,
    }
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
        |node_ctx, ctl| {
            let lock = node_ctx.lock_all();
            lock.sleep();
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
            for port in lock.node().out_ports() {
                ignore_nonfatal!({
                    lock.write(port.id(), &data)?;
                });
            }
            Ok(())
        },
    )
}

pub fn mixer<T: ByteConvertible + Default + Copy + AddAssign + Send + 'static>() -> NodeDescriptor {
    NodeDescriptor {
        name: "Mixer::".to_string() + unsafe { ::std::intrinsics::type_name::<T>() },
        new: run_mixer::<T>,
    }
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
            lock.sleep();
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
