#[cfg(test)]
extern crate alloc;

use gravity_cube_core::{Direction, PanelConfig, Position};

/// Panel mapper converts 3D cube coordinates to 2D panel LED indices
pub struct PanelMapper {
    panels: [PanelConfig; 6],
    lookup: [u8; 512], // 8x8x8 lookup table
    cube_size: u8,
}

impl PanelMapper {
    pub fn new(cube_size: u8) -> Self {
        // Chain order follows the WLED wiring: OUT of each panel feeds IN of the next.
        // Every panel has IN at its bottom-left and scans row by row up to OUT at the
        // top-right. Faces are named as in the unfolded cross (Front = North).
        let panels = [
            // 0 · Left
            PanelConfig {
                panel_num: 0,
                orientation: 0,
                direction: Direction::East,
                inverted_x: true,
                inverted_y: true,
            },
            // 1 · Front
            PanelConfig {
                panel_num: 1,
                orientation: 0,
                direction: Direction::North,
                inverted_x: false,
                inverted_y: true,
            },
            // 2 · Right
            PanelConfig {
                panel_num: 2,
                orientation: 0,
                direction: Direction::West,
                inverted_x: false,
                inverted_y: true,
            },
            // 3 · Back
            PanelConfig {
                panel_num: 3,
                orientation: 0,
                direction: Direction::South,
                inverted_x: true,
                inverted_y: true,
            },
            // 4 · Top
            PanelConfig {
                panel_num: 4,
                orientation: 0,
                direction: Direction::Up,
                inverted_x: false,
                inverted_y: false,
            },
            // 5 · Bottom
            PanelConfig {
                panel_num: 5,
                orientation: 0,
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
    pub fn position_to_led_indices<'a>(&'a self, pos: Position) -> PanelLedIndices<'a> {
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

    fn calculate_led_index(&self, panel: &PanelConfig, mut x: u8, mut y: u8) -> u16 {
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

#[cfg(test)]
mod tests {
    use super::*;

    fn leds(m: &PanelMapper, x: u8, y: u8, z: u8) -> alloc::vec::Vec<u16> {
        m.position_to_led_indices(Position { x, y, z }).collect()
    }

    #[test]
    fn every_led_is_hit_exactly_once() {
        let m = PanelMapper::new(8);
        let mut hits = [0u8; 384];
        for x in 0..8 {
            for y in 0..8 {
                for z in 0..8 {
                    for i in leds(&m, x, y, z) {
                        hits[i as usize] += 1;
                    }
                }
            }
        }
        assert!(hits.iter().all(|&h| h == 1));
    }

    #[test]
    fn chain_corners_match_wiring_diagram() {
        let m = PanelMapper::new(8);
        // Left OUT (top-right, LED 63) and Front's top-left LED are the same voxel.
        assert!(leds(&m, 0, 0, 0).contains(&63));
        assert!(leds(&m, 0, 0, 0).contains(&(64 + 56)));
        // Front IN (LED 64) sits on the bottom edge next to Left's bottom-right LED (7).
        assert!(leds(&m, 0, 0, 7).contains(&64));
        assert!(leds(&m, 0, 0, 7).contains(&7));
        // Front OUT (top-right, LED 127) and Right's top-left LED are the same voxel.
        assert!(leds(&m, 7, 0, 0).contains(&127));
        assert!(leds(&m, 7, 0, 0).contains(&(128 + 56)));
        // Top IN (bottom-left, LED 256) borders Front's top-left LED.
        assert!(leds(&m, 0, 0, 0).contains(&256));
        assert!(leds(&m, 0, 0, 0).contains(&(64 + 56)));
    }
}
