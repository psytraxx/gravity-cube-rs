mod camera;
mod cube_transform;
mod input;
mod renderer3d;

use cube_transform::CubeTransform;
use glam::Quat;
use gravity_cube_core::{Color, Config, EffectSwitcher, SimulationEffect, Vector3D, Vector3DExt};
use input::InputHandler;
use renderer3d::{RenderError, Renderer3D};
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
    simulation: EffectSwitcher,
    input_handler: InputHandler,
    cube_transform: CubeTransform,
    /// Cube orientation at the previous update, to measure how far it turned since.
    last_orientation: Quat,
    last_update: std::time::Instant,
}

impl State {
    async fn new(window: Arc<Window>) -> Self {
        // Each effect has its own particle count and colour; a double click switches
        let fluid_config = Config {
            num: 256,
            ..Config::default()
        };
        let boids_config = Config {
            num: 32,
            color: Color::new(255, 170, 60),
            ..fluid_config
        };

        let renderer = Renderer3D::new(window, fluid_config.color).await;

        let mut simulation = EffectSwitcher::with_configs(fluid_config, boids_config);
        simulation.populate(fluid_config.num);

        let input_handler = InputHandler::new(WINDOW_WIDTH, WINDOW_HEIGHT);
        let cube_transform = CubeTransform::new();
        let last_orientation = cube_transform.orientation();

        Self {
            renderer,
            simulation,
            input_handler,
            cube_transform,
            last_orientation,
            last_update: std::time::Instant::now(),
        }
    }

    /// Switch to the next effect and its colour.
    fn next_effect(&mut self) {
        self.simulation.next_effect();
        self.renderer.set_color(self.simulation.config().color);
        log::info!("Switched to {} effect", self.simulation.name());
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
        let acceleration = Vector3D::new(accel_glam.x, accel_glam.y, accel_glam.z);

        // How far the cube turned this frame, in its own axes, so spinning it swirls the fluid
        let orientation = self.cube_transform.orientation();
        let turn = (self.last_orientation.inverse() * orientation).to_scaled_axis();
        self.last_orientation = orientation;
        self.simulation
            .set_rotation(Vector3D::new(turn.x, turn.y, turn.z));

        // Update physics (every frame for smooth animation)
        self.simulation.step(&acceleration);
    }

    fn render(&mut self) -> Result<(), RenderError> {
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
                            if self.input_handler.register_click(std::time::Instant::now()) {
                                self.next_effect();
                            }
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
                        Err(RenderError::Lost) => state.resize(window.inner_size()),
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
