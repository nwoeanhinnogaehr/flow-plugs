use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::*;
use flow_synth::macros;
use std::sync::Arc;
use std::f64;

trait Osc {
    fn f(phase: f64) -> f64;
}

struct Sin;
impl Osc for Sin {
    fn f(phase: f64) -> f64 {
        phase.sin()
    }
}
struct Square;
impl Osc for Square {
    fn f(phase: f64) -> f64 {
        (phase / f64::consts::PI % 2.0).floor() * 2.0 - 1.0
    }
}
struct Saw;
impl Osc for Saw {
    fn f(phase: f64) -> f64 {
        (phase / f64::consts::PI % 2.0) - 1.0
    }
}

pub fn sin() -> NodeDescriptor {
    NodeDescriptor::new("SineWave", run_osc::<Sin>)
}
pub fn square() -> NodeDescriptor {
    NodeDescriptor::new("SquareWave", run_osc::<Square>)
}
pub fn saw() -> NodeDescriptor {
    NodeDescriptor::new("SawWave", run_osc::<Saw>)
}

fn run_osc<O: Osc>(ctx: Arc<Context>, cfg: NewNodeConfig) -> Arc<RemoteControl> {
    let mut phase = 0.0f64;
    let mut buffer = vec![];
    let ctl = macros::simple_node(ctx, cfg, (2, 1), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();

        // wait for request
        lock.wait(|lock| Ok(lock.available::<usize>(InPortID(0))? >= 1))?;
        let req_size = lock.read_n::<usize>(InPortID(0), 1)?[0];
        buffer.resize(req_size, 0.0);

        lock.wait(|lock| Ok(lock.available::<usize>(InPortID(1))? >= req_size))?;
        let freqs = lock.read_n::<f32>(InPortID(1), req_size)?;
        for (x, freq) in buffer.iter_mut().zip(freqs) {
            *x = O::f(phase) as f32;
            phase += 2.0 * freq as f64 * f64::consts::PI / 44100.0;
        }
        lock.write(OutPortID(0), &buffer)?;
        Ok(())
    });
    ctl.node().in_port(InPortID(0)).unwrap().set_name("wave req");
    ctl.node().in_port(InPortID(1)).unwrap().set_name("freq");
    ctl.node().out_port(OutPortID(0)).unwrap().set_name("wave");
    ctl
}
