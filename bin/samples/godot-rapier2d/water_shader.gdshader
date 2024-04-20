shader_type canvas_item;

uniform float threshold = 0.8;
uniform vec4 water_color: source_color = vec4(0,1,1,1);
uniform vec4 test_color: source_color = vec4(1,0,1,1);

uniform float speed = 0.1; // Speed of movement
uniform float amplitude = 0.1; // Amplitude of movement

uniform sampler2D screen_texture: hint_screen_texture, filter_linear_mipmap;

void fragment(){
	float displacement = sin(TIME * speed) * amplitude;
	vec4 screen_tex = textureLod(screen_texture, SCREEN_UV, 1.5).rgba;
	
	// check if the color is pink i.e ff00ff
	// if it is set it to the new color else set it to transparent
	float color_distance = distance(test_color.rgb, screen_tex.rgb);
	if (color_distance < threshold) {
		COLOR = texture(TEXTURE, SCREEN_UV + displacement).rgba * water_color;
		COLOR.a = (1.0-color_distance) * 100.0;
	} else {
		COLOR = vec4(0.0);
	}
}