use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use std::thread;
use std::sync::Arc;
use std::ops::AddAssign;

pub fn splitter() -> NodeDescriptor {
    NodeDescriptor {
        name: "Splitter".into(),
        new: run_splitter,
    }
}

pub fn run_splitter(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let node_id = cfg.node.unwrap_or_else(|| ctx.graph().add_node(1, 2));
    let node = ctx.graph().node(node_id).unwrap();
    let node_ctx = ctx.node_ctx(node_id).unwrap();
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
    thread::spawn(move || {
        while !ctl.stopped() {
            let res: Result<()> = do catch {
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
                ignore_nonfatal!({
                    for port in lock.node().out_ports() {
                        lock.write(port.id(), &data)?;
                    }
                });
                Ok(())
            };
            if let Err(e) = res {
                println!("splitter err {:?}", e);
            }
        }
    });

    remote_ctl
}

pub fn mixer<T: ByteConvertible + Default + Copy + AddAssign>() -> NodeDescriptor {
    NodeDescriptor {
        name: "Mixer::".to_string() + unsafe { ::std::intrinsics::type_name::<T>() },
        new: run_mixer::<T>,
    }
}

pub fn run_mixer<T: ByteConvertible + Default + Copy + AddAssign>(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let node_id = cfg.node.unwrap_or_else(|| ctx.graph().add_node(2, 1));
    let node = ctx.graph().node(node_id).unwrap();
    let node_ctx = ctx.node_ctx(node_id).unwrap();
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

    let buffer_size = 1024;

    let ctl = remote_ctl.clone();
    thread::spawn(move || {
        let mut accum = vec![T::default(); buffer_size];
        while !ctl.stopped() {
            let res: Result<()> = do catch {
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
                for port in lock.node().in_ports() {
                    ignore_nonfatal!({
                        lock.wait(|lock| Ok(lock.available::<T>(port.id())? >= buffer_size))?;
                        let data = lock.read_n::<T>(port.id(), buffer_size)?;
                        for i in 0..buffer_size {
                            accum[i] += data[i];
                        }
                    });
                }
                lock.write(OutPortID(0), &accum)?;
                for i in 0..buffer_size {
                    accum[i] = T::default();
                }
                Ok(())
            };
            if let Err(e) = res {
                println!("mixer err {:?}", e);
            }
        }
    });

    remote_ctl
}
