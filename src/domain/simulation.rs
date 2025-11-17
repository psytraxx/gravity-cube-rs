use super::types::*;
use micromath::F32Ext;

/// Physics simulation engine for gravity particles
pub struct Simulation {
    pub pixels: [Pixel; 512], // 8x8x8 = 512
    config: Config,
}

impl Simulation {
    pub fn new(config: Config) -> Self {
        let mut pixels = [Pixel::new(0, 0, 0); 512];

        // Initialize pixel grid
        let size = config.cube_size;
        for z in 0..size {
            for y in 0..size {
                for x in 0..size {
                    let index = Self::index_from_coords(x, y, z, size);
                    pixels[index as usize] = Pixel::new(x, y, z);
                }
            }
        }

        Self { pixels, config }
    }

    pub fn index_from_coords(x: u8, y: u8, z: u8, size: u8) -> u16 {
        (x as u16) + (y as u16 * size as u16) + (z as u16 * size as u16 * size as u16)
    }

    pub fn populate_particles(&mut self, num_particles: usize) {
        for i in 0..num_particles.min(512) {
            self.pixels[i].activate();
        }
    }

    /// Main simulation step - moves particles according to gravity
    pub fn step(&mut self, gravity: &Vector3D, random_fn: impl Fn() -> bool) {
        let move_params = MoveParams::from_gravity(gravity, self.config.velocity);
        let size = self.config.cube_size;
        let velocity = self.config.velocity as i32;

        // Track moved pixels to prevent double-movement using bitset
        // 512 bits = 8 x 64-bit words for O(1) lookups
        let mut moved_indices: [u64; 8] = [0; 8];

        for i in 0..512 {
            // Skip if already moved (O(1) bitset check)
            let word_idx = i / 64;
            let bit_idx = i % 64;
            if (moved_indices[word_idx] & (1 << bit_idx)) != 0 {
                continue;
            }

            if !self.pixels[i].active {
                continue;
            }

            let pos = self.pixels[i].position;

            // Randomly choose left/right preference
            let prefer_right = random_fn();

            // Try down movement first
            if let Some(new_pos) = self.try_move(&pos, &move_params.down, velocity, size) {
                self.move_pixel(i, new_pos, &mut moved_indices);
                continue;
            }

            // Try right/left based on random preference
            if prefer_right {
                if let Some(new_pos) = self.try_move(&pos, &move_params.right, velocity, size) {
                    self.move_pixel(i, new_pos, &mut moved_indices);
                    continue;
                }
                if let Some(new_pos) = self.try_move(&pos, &move_params.left, velocity, size) {
                    self.move_pixel(i, new_pos, &mut moved_indices);
                    continue;
                }
            } else {
                if let Some(new_pos) = self.try_move(&pos, &move_params.left, velocity, size) {
                    self.move_pixel(i, new_pos, &mut moved_indices);
                    continue;
                }
                if let Some(new_pos) = self.try_move(&pos, &move_params.right, velocity, size) {
                    self.move_pixel(i, new_pos, &mut moved_indices);
                    continue;
                }
            }
        }
    }

    /// Try to move in a direction, checking multiple velocities
    fn try_move(
        &self,
        pos: &Position,
        direction: &Vector3D,
        max_velocity: i32,
        size: u8,
    ) -> Option<Position> {
        for vel in (1..=max_velocity).rev() {
            let scale = vel as f32 / max_velocity as f32;

            let new_x = pos.x as i32 + F32Ext::round(direction.x * scale) as i32;
            let new_y = pos.y as i32 + F32Ext::round(direction.y * scale) as i32;
            let new_z = pos.z as i32 + F32Ext::round(direction.z * scale) as i32;

            // Check bounds
            if new_x < 0
                || new_x >= size as i32
                || new_y < 0
                || new_y >= size as i32
                || new_z < 0
                || new_z >= size as i32
            {
                continue;
            }

            let new_pos = Position::new(new_x as u8, new_y as u8, new_z as u8);
            let index = Self::index_from_coords(new_pos.x, new_pos.y, new_pos.z, size);

            // Check if space is free
            if !self.pixels[index as usize].active {
                return Some(new_pos);
            }
        }

        None
    }

    /// Move a pixel and track the move in bitset
    fn move_pixel(&mut self, old_index: usize, new_pos: Position, moved: &mut [u64; 8]) {
        let size = self.config.cube_size;
        let new_index = Self::index_from_coords(new_pos.x, new_pos.y, new_pos.z, size);

        self.pixels[new_index as usize].activate();
        self.pixels[old_index].deactivate();

        // Mark as moved in bitset (O(1))
        let word_idx = (new_index / 64) as usize;
        let bit_idx = new_index % 64;
        moved[word_idx] |= 1 << bit_idx;
    }

    /// Get all active pixels
    pub fn active_pixels(&self) -> impl Iterator<Item = &Pixel> {
        self.pixels.iter().filter(|p| p.active)
    }

    pub fn config(&self) -> &Config {
        &self.config
    }
}
