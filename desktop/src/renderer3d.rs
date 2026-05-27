use crate::camera::Camera;
use crate::cube_transform::CubeTransform;
use bytemuck::{Pod, Zeroable};
use gravity_cube_core::{Color, Particle};
use wgpu::util::DeviceExt;

// Matrix dimensions
const MATRIX_WIDTH: usize = 8;
const MATRIX_HEIGHT: usize = 8;
const MATRIX_DEPTH: usize = 8;

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
}

impl Vertex {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[wgpu::VertexAttribute {
                offset: 0,
                shader_location: 0,
                format: wgpu::VertexFormat::Float32x3,
            }],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct Instance {
    voxel_pos: [f32; 3],
    color: [f32; 3],
}

impl Instance {
    fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Instance>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 2,
                    format: wgpu::VertexFormat::Float32x3,
                },
            ],
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    model: [[f32; 4]; 4],
}

pub struct Renderer3D {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    render_pipeline: wgpu::RenderPipeline,
    wireframe_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    instance_buffer: wgpu::Buffer,
    wireframe_vertex_buffer: wgpu::Buffer,
    wireframe_index_buffer: wgpu::Buffer,
    wireframe_num_indices: u32,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    num_indices: u32,
    pub camera: Camera,
    base_color: [f32; 3],
}

impl Renderer3D {
    pub async fn new(window: std::sync::Arc<winit::window::Window>, color: Color) -> Self {
        let size = window.inner_size();

        // Convert Color to normalized RGB
        let base_color = [
            color.r as f32 / 255.0,
            color.g as f32 / 255.0,
            color.b as f32 / 255.0,
        ];

        // Create instance
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        // Create surface
        let surface = instance.create_surface(window.clone()).unwrap();

        // Request adapter
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::default(),
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        // Create device and queue
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                label: None,
                memory_hints: wgpu::MemoryHints::default(),
                experimental_features: Default::default(),
                trace: Default::default(),
            })
            .await
            .unwrap();

        // Configure surface
        let surface_caps = surface.get_capabilities(&adapter);
        let surface_format = surface_caps
            .formats
            .iter()
            .copied()
            .find(|f| f.is_srgb())
            .unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        // Create camera
        let camera = Camera::new(size.width as f32 / size.height as f32);

        // Create camera uniform buffer
        let camera_uniform = CameraUniform {
            view_proj: camera.view_projection_matrix().to_cols_array_2d(),
            model: glam::Mat4::IDENTITY.to_cols_array_2d(),
        };

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Camera Buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        // Create bind group layout for camera
        let camera_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Camera Bind Group Layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Camera Bind Group"),
            layout: &camera_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });

        // Load shaders
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Vertex Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/vertex3d.wgsl").into()),
        });

        let fragment_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Fragment Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/fragment3d.wgsl").into()),
        });

        let wireframe_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Wireframe Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/wireframe.wgsl").into()),
        });

        // Create depth texture
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });

        let _depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Create render pipeline
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[Vertex::desc(), Instance::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &fragment_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });

        // Create wireframe pipeline
        let wireframe_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Wireframe Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &wireframe_shader,
                entry_point: Some("vs_main"),
                buffers: &[Vertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &wireframe_shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
            cache: None,
        });

        // Create cube vertices (unit cube centered at origin)
        let vertices = Self::create_cube_vertices();
        let indices = Self::create_cube_indices();
        let num_indices = indices.len() as u32;

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Index Buffer"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        // Create instance buffer
        let max_instances = MATRIX_WIDTH * MATRIX_HEIGHT * MATRIX_DEPTH;
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Instance Buffer"),
            size: (std::mem::size_of::<Instance>() * max_instances) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Create wireframe buffers
        let wireframe_vertices = Self::create_wireframe_vertices();
        let wireframe_indices = Self::create_wireframe_indices();
        let wireframe_num_indices = wireframe_indices.len() as u32;

        let wireframe_vertex_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Wireframe Vertex Buffer"),
                contents: bytemuck::cast_slice(&wireframe_vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });

        let wireframe_index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Wireframe Index Buffer"),
            contents: bytemuck::cast_slice(&wireframe_indices),
            usage: wgpu::BufferUsages::INDEX,
        });

        Self {
            surface,
            device,
            queue,
            config,
            render_pipeline,
            wireframe_pipeline,
            vertex_buffer,
            index_buffer,
            instance_buffer,
            wireframe_vertex_buffer,
            wireframe_index_buffer,
            wireframe_num_indices,
            camera_buffer,
            camera_bind_group,
            num_indices,
            camera,
            base_color,
        }
    }

    fn create_cube_vertices() -> Vec<Vertex> {
        // Cube vertices centered at 0.5, 0.5, 0.5 (for unit cube)
        vec![
            // Front face
            Vertex {
                position: [0.0, 0.0, 1.0],
            },
            Vertex {
                position: [1.0, 0.0, 1.0],
            },
            Vertex {
                position: [1.0, 1.0, 1.0],
            },
            Vertex {
                position: [0.0, 1.0, 1.0],
            },
            // Back face
            Vertex {
                position: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [1.0, 0.0, 0.0],
            },
            Vertex {
                position: [1.0, 1.0, 0.0],
            },
            Vertex {
                position: [0.0, 1.0, 0.0],
            },
        ]
    }

    fn create_cube_indices() -> Vec<u16> {
        vec![
            // Front
            0, 1, 2, 0, 2, 3, // Back
            5, 4, 7, 5, 7, 6, // Left
            4, 0, 3, 4, 3, 7, // Right
            1, 5, 6, 1, 6, 2, // Top
            3, 2, 6, 3, 6, 7, // Bottom
            4, 5, 1, 4, 1, 0,
        ]
    }

    fn create_wireframe_vertices() -> Vec<Vertex> {
        // 8 corners of the 8x8x8 cube
        vec![
            Vertex {
                position: [0.0, 0.0, 0.0],
            },
            Vertex {
                position: [MATRIX_WIDTH as f32, 0.0, 0.0],
            },
            Vertex {
                position: [MATRIX_WIDTH as f32, MATRIX_HEIGHT as f32, 0.0],
            },
            Vertex {
                position: [0.0, MATRIX_HEIGHT as f32, 0.0],
            },
            Vertex {
                position: [0.0, 0.0, MATRIX_DEPTH as f32],
            },
            Vertex {
                position: [MATRIX_WIDTH as f32, 0.0, MATRIX_DEPTH as f32],
            },
            Vertex {
                position: [
                    MATRIX_WIDTH as f32,
                    MATRIX_HEIGHT as f32,
                    MATRIX_DEPTH as f32,
                ],
            },
            Vertex {
                position: [0.0, MATRIX_HEIGHT as f32, MATRIX_DEPTH as f32],
            },
        ]
    }

    fn create_wireframe_indices() -> Vec<u16> {
        // 12 edges of the cube
        vec![
            // Bottom face edges
            0, 1, 1, 2, 2, 3, 3, 0, // Top face edges
            4, 5, 5, 6, 6, 7, 7, 4, // Vertical edges
            0, 4, 1, 5, 2, 6, 3, 7,
        ]
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
            self.camera
                .update_aspect(new_size.width as f32 / new_size.height as f32);
        }
    }

    pub fn update_cube_transform(&mut self, cube_transform: &CubeTransform) {
        let camera_uniform = CameraUniform {
            view_proj: self.camera.view_projection_matrix().to_cols_array_2d(),
            model: cube_transform.model_matrix().to_cols_array_2d(),
        };
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&[camera_uniform]),
        );
    }

    pub fn render(&mut self, particles: &[Particle]) -> Result<(), wgpu::SurfaceError> {
        // Create instances from particles
        let instances = self.create_instances(particles);

        // Update instance buffer
        self.queue
            .write_buffer(&self.instance_buffer, 0, bytemuck::cast_slice(&instances));

        // Get surface texture
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        // Create depth texture
        let depth_texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("Depth Texture"),
            size: wgpu::Extent3d {
                width: self.config.width,
                height: self.config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });

        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Create command encoder
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.1,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                    depth_slice: None,
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                occlusion_query_set: None,
                timestamp_writes: None,
            });

            // Draw voxels
            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            render_pass.set_vertex_buffer(1, self.instance_buffer.slice(..));
            render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw_indexed(0..self.num_indices, 0, 0..instances.len() as u32);

            // Draw wireframe
            render_pass.set_pipeline(&self.wireframe_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.wireframe_vertex_buffer.slice(..));
            render_pass.set_index_buffer(
                self.wireframe_index_buffer.slice(..),
                wgpu::IndexFormat::Uint16,
            );
            render_pass.draw_indexed(0..self.wireframe_num_indices, 0, 0..1);
        }

        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }

    fn create_instances(&self, particles: &[Particle]) -> Vec<Instance> {
        use std::collections::HashMap;

        let mut grid_cells: HashMap<(i32, i32, i32), (usize, f32)> = HashMap::new();

        // Find which particle should occupy each grid cell (prefer faster particles for brightness)
        for (idx, particle) in particles.iter().enumerate() {
            let x = particle
                .position
                .x
                .round()
                .clamp(0.0, (MATRIX_WIDTH - 1) as f32) as i32;
            let y = particle
                .position
                .y
                .round()
                .clamp(0.0, (MATRIX_HEIGHT - 1) as f32) as i32;
            let z = particle
                .position
                .z
                .round()
                .clamp(0.0, (MATRIX_DEPTH - 1) as f32) as i32;

            let cell_key = (x, y, z);
            let speed = particle.speed();

            // Keep the particle with higher speed for better visual feedback
            grid_cells
                .entry(cell_key)
                .and_modify(|(existing_idx, existing_speed)| {
                    if speed > *existing_speed {
                        *existing_idx = idx;
                        *existing_speed = speed;
                    }
                })
                .or_insert((idx, speed));
        }

        // Create instances only for occupied cells
        let mut instances = Vec::new();
        for ((x, y, z), _) in grid_cells {
            instances.push(Instance {
                voxel_pos: [x as f32, y as f32, z as f32],
                color: self.base_color,
            });
        }

        instances
    }
}
