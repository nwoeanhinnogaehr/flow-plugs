use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;

pub fn beat(name: &str, f: fn(usize) -> f32) -> NodeDescriptor {
    NodeDescriptor::new(
        "bytebeat.".to_string() + name,
        move |ctx: Arc<Context>, cfg: NewNodeConfig| run_beat(f, ctx, cfg),
    )
}

fn run_beat(f: fn(usize) -> f32, ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let mut t = 0;
    let mut buffer = vec![];
    macros::simple_node(ctx, cfg, (1, 1), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();
        lock.wait(|lock| Ok(lock.available::<usize>(InPortID(0))? >= 1))?;
        let req_size = lock.read_n::<usize>(InPortID(0), 1)?[0];
        buffer.resize(req_size, 0f32);
        for x in &mut buffer {
            *x = f(t);
            t += 1;
        }
        lock.write(OutPortID(0), &buffer)?;
        Ok(())
    })
}
