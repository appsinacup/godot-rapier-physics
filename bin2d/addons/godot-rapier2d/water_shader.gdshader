shader_type canvas_item;

uniform float threshold = 0.8;
uniform vec4 water_color: source_color = vec4(0.12,0.24,0.45,0.65);
uniform vec4 test_color: source_color = vec4(1,0,1,1);

uniform float speed = 0.1; // Speed of movement
uniform float amplitude = 0.1; // Amplitude of movement

uniform sampler2D water_texture;

void fragment(){
	float displacement = sin(TIME * speed) * amplitude;
	vec4 screen_tex = texture(TEXTURE, SCREEN_UV).rgba;

	float color_distance = screen_tex.r;
	if (color_distance > threshold) {
		COLOR = texture(water_texture, SCREEN_UV + displacement).rgba * water_color;
	} else {
		COLOR = vec4(0.0);
	}
}