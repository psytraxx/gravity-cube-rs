mod camera;
mod cube_transform;
mod input;
mod renderer3d;

use cube_transform::CubeTransform;
use gravity_cube_core::{Config, Simulation, SimulationEffect, Vector3DExt};
use input::InputHandler;
use renderer3d::Renderer3D;
use std::sync::Arc;
use winit::{
    application::ApplicationHandler,
    event::*,
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowAttributes, WindowId},
};

const WINDOW_WIDTH: u32 = 1400;
const WINDOW_HEIGHT: u32 = 1400;

struct App {
    state: Option<State>,
    window: Option<Arc<Window>>,
}

struct State {
    renderer: Renderer3D,
    simulation: Simulation,
    input_handler: InputHandler,
    cube_transform: CubeTransform,
    last_update: std::time::Instant,
}

impl State {
    async fn new(window: Arc<Window>) -> Self {
        // Initialize simulation with fluid physics
        let config = Config::default();

        let renderer = Renderer3D::new(window, config.color).await;

        let mut simulation = Simulation::new(config);
        simulation.populate(config.num_particles);

        let input_handler = InputHandler::new(WINDOW_WIDTH, WINDOW_HEIGHT);
        let cube_transform = CubeTransform::new();

        Self {
            renderer,
            simulation,
            input_handler,
            cube_transform,
            last_update: std::time::Instant::now(),
        }
    }

    fn update(&mut self) {
        let now = std::time::Instant::now();
        let _dt = now.duration_since(self.last_update).as_secs_f32();
        self.last_update = now;

        // Update cube rotation from mouse drag
        let (mouse_dx, mouse_dy) = self.input_handler.get_mouse_delta();

        // Update cube rotation from keyboard
        let (kbd_dx, kbd_dy) = self.input_handler.get_keyboard_delta();

        // Combine mouse and keyboard rotation
        let delta_x = mouse_dx + kbd_dx;
        let delta_y = mouse_dy + kbd_dy;

        if delta_x.abs() > 0.0 || delta_y.abs() > 0.0 {
            self.cube_transform.rotate(delta_x, -delta_y);
            self.renderer.update_cube_transform(&self.cube_transform);
        }

        // Get gravity in cube's local space
        // World gravity is always down, but transformed to cube's rotated coordinate system
        let accel_glam = self.cube_transform.get_local_gravity();

        // Convert glam::Vec3 to gravity_cube_core::Vector3D
        let acceleration =
            gravity_cube_core::Vector3D::new(accel_glam.x, accel_glam.y, accel_glam.z);

        // Update physics (every frame for smooth animation)
        self.simulation.step(&acceleration);
    }

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        self.renderer.render(self.simulation.particles())
    }

    fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.renderer.resize(new_size);
    }

    fn input(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::CursorMoved { position, .. } => {
                self.input_handler.update_mouse_position(*position);
                true
            }
            WindowEvent::MouseInput { state, button, .. } => {
                if *button == winit::event::MouseButton::Left {
                    match state {
                        ElementState::Pressed => {
                            self.input_handler.start_drag(self.input_handler.mouse_pos);
                        }
                        ElementState::Released => {
                            self.input_handler.end_drag();
                        }
                    }
                    return true;
                }
                false
            }
            WindowEvent::KeyboardInput {
                event:
                    KeyEvent {
                        physical_key: PhysicalKey::Code(key_code),
                        state,
                        ..
                    },
                ..
            } => {
                // Handle arrow keys for rotation
                let pressed = *state == ElementState::Pressed;
                self.input_handler.handle_keyboard(*key_code, pressed);

                // Don't consume the event so Escape can still close the window
                matches!(
                    key_code,
                    KeyCode::ArrowLeft
                        | KeyCode::ArrowRight
                        | KeyCode::ArrowUp
                        | KeyCode::ArrowDown
                )
            }
            _ => false,
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let window_attributes = WindowAttributes::default()
                .with_title("Gravity Cube - Fluid Simulation")
                .with_inner_size(winit::dpi::PhysicalSize::new(WINDOW_WIDTH, WINDOW_HEIGHT))
                .with_resizable(true);

            let window = Arc::new(event_loop.create_window(window_attributes).unwrap());
            self.state = Some(pollster::block_on(State::new(window.clone())));
            self.window = Some(window);
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let (Some(state), Some(window)) = (self.state.as_mut(), self.window.as_ref()) else {
            return;
        };

        if !state.input(&event) {
            match event {
                WindowEvent::CloseRequested
                | WindowEvent::KeyboardInput {
                    event:
                        KeyEvent {
                            state: ElementState::Pressed,
                            physical_key: PhysicalKey::Code(KeyCode::Escape),
                            ..
                        },
                    ..
                } => event_loop.exit(),
                WindowEvent::Resized(physical_size) => {
                    state.resize(physical_size);
                }
                WindowEvent::RedrawRequested => {
                    state.update();
                    match state.render() {
                        Ok(_) => {}
                        Err(wgpu::SurfaceError::Lost) => state.resize(window.inner_size()),
                        Err(wgpu::SurfaceError::OutOfMemory) => event_loop.exit(),
                        Err(e) => eprintln!("{:?}", e),
                    }
                }
                _ => {}
            }
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
        }
    }
}

fn main() {
    env_logger::init();

    let event_loop = EventLoop::new().unwrap();
    let mut app = App {
        state: None,
        window: None,
    };

    event_loop.run_app(&mut app).unwrap();
}
