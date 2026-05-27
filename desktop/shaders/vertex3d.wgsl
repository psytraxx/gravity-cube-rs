struct Camera {
    view_proj: mat4x4<f32>,
    model: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: Camera;

struct VertexInput {
    @location(0) position: vec3<f32>,
};

struct InstanceInput {
    @location(1) voxel_pos: vec3<f32>,
    @location(2) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

@vertex
fn vs_main(
    vertex: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;

    // Scale voxel to unit size and position it in grid
    let scaled_vertex = vertex.position * 0.9; // Slightly smaller for gaps
    let local_pos = scaled_vertex + instance.voxel_pos;

    // Apply model matrix (cube rotation), then view-projection
    let world_pos = camera.model * vec4<f32>(local_pos, 1.0);
    out.clip_position = camera.view_proj * world_pos;
    out.color = instance.color;
    out.normal = vertex.position; // Simple normal for lighting

    return out;
}
