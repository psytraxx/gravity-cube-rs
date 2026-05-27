use winit::dpi::PhysicalPosition;

/// Handles mouse and keyboard input for cube rotation
pub struct InputHandler {
    pub mouse_pos: PhysicalPosition<f64>,
    last_mouse_pos: PhysicalPosition<f64>,
    is_dragging: bool,
    // Keyboard rotation state
    rotate_left: bool,
    rotate_right: bool,
    rotate_up: bool,
    rotate_down: bool,
}

impl InputHandler {
    pub fn new(window_width: u32, window_height: u32) -> Self {
        let center = PhysicalPosition::new(window_width as f64 / 2.0, window_height as f64 / 2.0);
        Self {
            mouse_pos: center,
            last_mouse_pos: center,
            is_dragging: false,
            rotate_left: false,
            rotate_right: false,
            rotate_up: false,
            rotate_down: false,
        }
    }

    pub fn start_drag(&mut self, position: PhysicalPosition<f64>) {
        self.is_dragging = true;
        self.last_mouse_pos = position;
        self.mouse_pos = position;
    }

    pub fn end_drag(&mut self) {
        self.is_dragging = false;
    }

    pub fn update_mouse_position(&mut self, position: PhysicalPosition<f64>) {
        self.last_mouse_pos = self.mouse_pos;
        self.mouse_pos = position;
    }

    /// Get mouse delta for cube rotation
    pub fn get_mouse_delta(&self) -> (f32, f32) {
        if !self.is_dragging {
            return (0.0, 0.0);
        }

        let dx = (self.mouse_pos.x - self.last_mouse_pos.x) as f32;
        let dy = (self.mouse_pos.y - self.last_mouse_pos.y) as f32;

        // Scale by sensitivity
        (dx * 0.005, dy * 0.005)
    }

    /// Handle keyboard input for rotation
    pub fn handle_keyboard(&mut self, key_code: winit::keyboard::KeyCode, pressed: bool) {
        use winit::keyboard::KeyCode;
        match key_code {
            KeyCode::ArrowLeft => self.rotate_left = pressed,
            KeyCode::ArrowRight => self.rotate_right = pressed,
            KeyCode::ArrowUp => self.rotate_up = pressed,
            KeyCode::ArrowDown => self.rotate_down = pressed,
            _ => {}
        }
    }

    /// Get keyboard rotation delta
    pub fn get_keyboard_delta(&self) -> (f32, f32) {
        const ROTATION_SPEED: f32 = 0.02;

        let mut dx = 0.0;
        let mut dy = 0.0;

        if self.rotate_left {
            dx -= ROTATION_SPEED;
        }
        if self.rotate_right {
            dx += ROTATION_SPEED;
        }
        if self.rotate_up {
            dy += ROTATION_SPEED;
        }
        if self.rotate_down {
            dy -= ROTATION_SPEED;
        }

        (dx, dy)
    }
}
