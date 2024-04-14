@tool
extends Fluid2D

@export var height := 10:
	set(value):
		if height != value:
			height = value
			points = create_rectangle_points(width, height, global_position)
	get:
		return height
@export var width := 10:
	set(value):
		if width != value:
			width = value
			points = create_rectangle_points(width, height, global_position)
	get:
		return width
