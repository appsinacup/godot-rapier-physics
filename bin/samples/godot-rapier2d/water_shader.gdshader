shader_type canvas_item;

uniform float threshold = 0.3;
uniform vec4 color:source_color = vec4(0,0,1,1);
const int MAX_POINTS = 1000;
uniform vec2 points[MAX_POINTS];

uniform sampler2D screen_texture: hint_screen_texture, filter_linear_mipmap;

varying vec4 world_pos;

void vertex() {
	world_pos = MODEL_MATRIX * vec4(VERTEX, 1.0, 1.0);
}

void fragment(){
	// get a blurred version of the screen texture
	vec4 screen_tex = textureLod(screen_texture, SCREEN_UV, 1.5).rgba;
	
	// check if the color is pink i.e ff00ff
	// if it is set it to the new color else set it to transparent
	bool found = false;
	for (int i=0;i<MAX_POINTS;i++) {
		if(distance(points[i], vec2(world_pos.x, world_pos.y)) < 10.0){
			found = true;
			break;
		}
	}
	if (found) {
		COLOR = texture(TEXTURE, SCREEN_UV).rgba * color;
	} else {
		COLOR = vec4(0.0);
	}
}