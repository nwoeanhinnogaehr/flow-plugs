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
    let buffer_size = 32;
    let max_buffered = 4096;
    let mut phase = 0.0f64;
    let mut buffer = vec![0.0f32; buffer_size];
    let mut freq = 0.0f64;
    macros::simple_node(ctx, cfg, (1, 1), vec![], move |node_ctx, _| {
        let lock = node_ctx.lock_all();
        if let Ok(Some(new_freq)) = lock.read_n::<f32>(InPortID(0), 1)
            .map(|data| data.last().cloned())
        {
            freq = new_freq as f64;
        }
        lock.wait(|lock| {
            Ok(lock.buffered::<f32>(OutPortID(0))? < max_buffered)
        })?;
        for x in &mut buffer {
            *x = O::f(phase) as f32;
            phase += 2.0 * freq * f64::consts::PI / 44100.0;
        }
        lock.write(OutPortID(0), &buffer)?;
        Ok(())
    })
}
