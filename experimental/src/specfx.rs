use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;

pub fn hold() -> NodeDescriptor {
    NodeDescriptor::new("SpecFX.hodl", new_hold)
}

fn new_hold(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let ctl = macros::simple_node(ctx, cfg, (0, 1), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();
        lock.sleep();
        Ok(())
    });
    ctl.node().in_port(InPortID(0)).unwrap().set_name("spec in");
    ctl
}
