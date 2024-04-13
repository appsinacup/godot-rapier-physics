@tool
extends Fluid2D

@export var color: Color = Color.WHITE
@export var height := 10:
	set(value):
		if height != value:
			height = value
			_create()
	get:
		return height
@export var width := 10:
	set(value):
		if width != value:
			width = value
			_create()
	get:
		return width

func _draw():
	for point in points:
		if Engine.is_editor_hint():
			draw_circle(point, 5, color)
		else:
			draw_circle(point - position, 5, color)

func _create():
	var new_points = []
	var radius = ProjectSettings.get_setting("physics/rapier_2d/fluid/fluid_particle_radius")
	for i in width:
		for j in height:
			if Engine.is_editor_hint():
				new_points.append(Vector2(i*radius * 2, j*radius * 2) - position)
			else:
				new_points.append(Vector2(i*radius * 2, j*radius * 2))
	points = new_points

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	queue_redraw()
