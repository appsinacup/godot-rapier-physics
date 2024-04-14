@tool
extends Fluid2D

@export var height := 10:
	set(value):
		if height != value:
			height = value
			points = create_rectangle_points(width, height)
	get:
		return height
@export var width := 10:
	set(value):
		if width != value:
			width = value
			points = create_rectangle_points(width, height)
	get:
		return width
