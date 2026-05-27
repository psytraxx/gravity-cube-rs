struct FragmentInput {
    @location(0) color: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

@fragment
fn fs_main(in: FragmentInput) -> @location(0) vec4<f32> {
    // Simple directional lighting
    let light_dir = normalize(vec3<f32>(0.5, 0.8, 0.6));
    let normal = normalize(in.normal);
    let diffuse = max(dot(normal, light_dir), 0.2); // Minimum ambient light

    let lit_color = in.color * diffuse;
    return vec4<f32>(lit_color, 1.0);
}
