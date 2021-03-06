use modular_flow::graph::*;
use modular_flow::context::*;
use flow_synth::control::{NewNodeConfig, NodeDescriptor, RemoteControl};
use sdl2;
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use sdl2::event::{Event, WindowEvent};
use sdl2::keyboard::Keycode;
use std::thread;
use std::mem;
use std::sync::Arc;

pub fn pixel_scroller() -> NodeDescriptor {
    NodeDescriptor::new("PixelScroller", new_pixel_scroller)
}

fn new_pixel_scroller(ctx: Arc<Context>, config: NewNodeConfig) -> Arc<RemoteControl> {
    let id = config.node.unwrap_or_else(|| ctx.graph().add_node(1, 0));
    let node_ctx = ctx.node_ctx(id).unwrap();
    let node = ctx.graph().node(id).unwrap();
    let remote_ctl = Arc::new(RemoteControl::new(ctx, node, vec![]));
    let mut width = 1024;
    let mut height = 1024;

    let ctl = remote_ctl.clone();
    thread::spawn(move || {
        let sdl_context = sdl2::init().unwrap();
        let video_subsystem = sdl_context.video().unwrap();

        let window = video_subsystem
            .window("pixel-scroller", width as u32, height as u32)
            .opengl()
            .resizable()
            .build()
            .unwrap();

        let mut canvas = window.into_canvas().build().unwrap();
        let texture_creator = canvas.texture_creator();

        let mut texture = texture_creator
            .create_texture_streaming(PixelFormatEnum::RGBA8888, width as u32, height as u32 * 2)
            .unwrap();

        let mut event_pump = sdl_context.event_pump().unwrap();

        let mut time = 0;

        'mainloop: while !ctl.stopped() {
            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. } |
                    Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => break 'mainloop,
                    Event::Window {
                        win_event: WindowEvent::Resized(_, h),
                        ..
                    } => {
                        height = h;
                        texture = texture_creator
                            .create_texture_streaming(
                                PixelFormatEnum::RGBA8888,
                                width as u32,
                                height as u32 * 2,
                            )
                            .unwrap();
                    }
                    _ => {}
                }
            }

            let lock = node_ctx.lock_all();
            match lock.read_n::<usize>(InPortID(0), 1) {
                Ok(x) => {
                    let size = x[0];
                    if size > 8192 {
                        eprintln!("pix scroller got size too large: {}", size);
                        continue;
                    }
                    if size != width {
                        width = size;
                        texture = texture_creator
                            .create_texture_streaming(
                                PixelFormatEnum::RGBA8888,
                                width as u32,
                                height as u32 * 2,
                            )
                            .unwrap();
                    }
                }
                Err(_) => {
                    lock.sleep();
                    continue;
                }
            }
            let res: Result<_> = do catch {
                lock.wait(|lock| Ok(lock.available::<u32>(InPortID(0))? >= width))?;
                let available_pixels = lock.available::<u32>(InPortID(0)).unwrap_or(0);
                if available_pixels >= width {
                    let frame = lock.read_n::<u32>(InPortID(0), width)?;
                    let scroll_pos = time % (height as i32 / 2);
                    texture
                        .update(
                            Rect::new(0, scroll_pos, width as u32, 1),
                            unsafe { mem::transmute(&frame[..]) },
                            width * 4,
                        )
                        .unwrap();
                    texture
                        .update(
                            Rect::new(0, scroll_pos + height as i32 / 2, width as u32, 1),
                            unsafe { mem::transmute(&frame[..]) },
                            width * 4,
                        )
                        .unwrap();
                    time += 1;
                    canvas
                        .copy(
                            &texture,
                            Some(Rect::new(0, scroll_pos, width as u32, height as u32 / 2)),
                            None,
                        )
                        .unwrap();
                    canvas.present();
                }
                Ok(())
            };
        }
    });
    remote_ctl
}
