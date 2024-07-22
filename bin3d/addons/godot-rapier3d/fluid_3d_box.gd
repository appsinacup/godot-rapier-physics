@tool
extends Fluid3D

@export var height := 10:
	set(value):
		if height != value:
			height = value
			points = create_box_points(width, height, depth)
	get:
		return height
@export var width := 10:
	set(value):
		if width != value:
			width = value
			points = create_box_points(width, height, depth)
	get:
		return width

@export var depth := 10:
	set(value):
		if depth != value:
			depth = value
			points = create_box_points(width, height, depth)
	get:
		return depth
