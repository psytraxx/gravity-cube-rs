struct VertexInput {
    @location(0) position: vec2<f32>,
};

struct InstanceInput {
    @location(1) grid_pos: vec2<f32>,
    @location(2) color: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(
    vertex: VertexInput,
    instance: InstanceInput,
) -> VertexOutput {
    var out: VertexOutput;

    // Convert grid position (0-15) to NDC (-1 to 1)
    // Each cell is 20x20 pixels in a 320x320 window
    let cell_size = 2.0 / 16.0; // 0.125 in NDC

    // Calculate position in NDC
    let grid_ndc = vec2<f32>(
        (instance.grid_pos.x / 16.0) * 2.0 - 1.0,
        -((instance.grid_pos.y / 16.0) * 2.0 - 1.0), // Flip Y
    );

    // Scale vertex position and add to grid position
    let scaled_vertex = vertex.position * cell_size;
    let final_pos = grid_ndc + scaled_vertex;

    out.clip_position = vec4<f32>(final_pos, 0.0, 1.0);
    out.color = instance.color;

    return out;
}
