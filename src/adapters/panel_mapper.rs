use crate::domain::{Position, Direction, PanelConfig};

/// Panel mapper converts 3D cube coordinates to 2D panel LED indices
pub struct PanelMapper {
    panels: [PanelConfig; 6],
    lookup: [u8; 512], // 8x8x8 lookup table
    cube_size: u8,
}

impl PanelMapper {
    pub fn new(cube_size: u8) -> Self {
        // Default panel configuration from C code
        let panels = [
            PanelConfig {
                panel_num: 0,
                orientation: 0,
                direction: Direction::West,
                inverted_x: true,
                inverted_y: false,
            },
            PanelConfig {
                panel_num: 1,
                orientation: 0,
                direction: Direction::North,
                inverted_x: true,
                inverted_y: false,
            },
            PanelConfig {
                panel_num: 2,
                orientation: 270,
                direction: Direction::Up,
                inverted_x: false,
                inverted_y: false,
            },
            PanelConfig {
                panel_num: 3,
                orientation: 270,
                direction: Direction::South,
                inverted_x: false,
                inverted_y: false,
            },
            PanelConfig {
                panel_num: 4,
                orientation: 270,
                direction: Direction::East,
                inverted_x: false,
                inverted_y: false,
            },
            PanelConfig {
                panel_num: 5,
                orientation: 90,
                direction: Direction::Down,
                inverted_x: false,
                inverted_y: true,
            },
        ];

        let mut mapper = Self {
            panels,
            lookup: [0; 512],
            cube_size,
        };

        mapper.init_lookup();
        mapper
    }

    fn init_lookup(&mut self) {
        for x in 0..self.cube_size {
            for y in 0..self.cube_size {
                for z in 0..self.cube_size {
                    let mut mask = 0u8;

                    // Set bit flags for panels this pixel touches
                    if x == 0 {
                        mask |= 0b000001; // EAST
                    }
                    if x == 7 {
                        mask |= 0b000010; // WEST
                    }
                    if y == 7 {
                        mask |= 0b000100; // SOUTH
                    }
                    if y == 0 {
                        mask |= 0b001000; // NORTH
                    }
                    if z == 0 {
                        mask |= 0b010000; // UP
                    }
                    if z == 7 {
                        mask |= 0b100000; // DOWN
                    }

                    let index = (x as usize)
                        + (y as usize * self.cube_size as usize)
                        + (z as usize * self.cube_size as usize * self.cube_size as usize);
                    self.lookup[index] = mask;
                }
            }
        }
    }

    pub fn get_panel_mask(&self, pos: Position) -> u8 {
        let index = (pos.x as usize)
            + (pos.y as usize * self.cube_size as usize)
            + (pos.z as usize * self.cube_size as usize * self.cube_size as usize);
        self.lookup[index]
    }

    /// Convert 3D position to LED indices for all visible panels
    /// Returns iterator of (led_index, panel_bits)
    pub fn position_to_led_indices(&self, pos: Position) -> PanelLedIndices {
        let mask = self.get_panel_mask(pos);
        PanelLedIndices {
            mapper: self,
            pos,
            mask,
            current_bit: 0,
        }
    }

    fn get_panel_for_direction(&self, direction: Direction) -> Option<&PanelConfig> {
        self.panels.iter().find(|p| p.direction == direction)
    }

    fn calculate_led_index(
        &self,
        panel: &PanelConfig,
        mut x: u8,
        mut y: u8,
    ) -> u16 {
        // Apply orientation transformation
        let (new_x, new_y) = match panel.orientation {
            0 => (x, y),
            90 => (7 - y, x),
            180 => (7 - x, 7 - y),
            270 => (y, 7 - x),
            _ => (x, y),
        };

        x = new_x;
        y = new_y;

        // Apply inversions
        if panel.inverted_x {
            x = 7 - x;
        }
        if panel.inverted_y {
            y = 7 - y;
        }

        // Calculate final LED index
        let panel_offset = panel.panel_num as u16 * 64;
        let pixel_index = x as u16 + y as u16 * 8;

        panel_offset + pixel_index
    }
}

/// Iterator over LED indices for a 3D position
pub struct PanelLedIndices<'a> {
    mapper: &'a PanelMapper,
    pos: Position,
    mask: u8,
    current_bit: u8,
}

impl<'a> Iterator for PanelLedIndices<'a> {
    type Item = u16;

    fn next(&mut self) -> Option<Self::Item> {
        while self.current_bit < 6 {
            let bit = self.current_bit;
            self.current_bit += 1;

            if (self.mask & (1 << bit)) != 0 {
                // Determine direction from bit position
                let direction = match bit {
                    0 => Direction::East,
                    1 => Direction::West,
                    2 => Direction::South,
                    3 => Direction::North,
                    4 => Direction::Up,
                    5 => Direction::Down,
                    _ => continue,
                };

                if let Some(panel) = self.mapper.get_panel_for_direction(direction) {
                    // Map coordinates based on direction
                    let (x, y) = match direction {
                        Direction::East | Direction::West => (self.pos.y, self.pos.z),
                        Direction::North | Direction::South => (self.pos.x, self.pos.z),
                        Direction::Up | Direction::Down => (self.pos.x, self.pos.y),
                    };

                    return Some(self.mapper.calculate_led_index(panel, x, y));
                }
            }
        }

        None
    }
}
