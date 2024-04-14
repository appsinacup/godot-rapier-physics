@tool
extends Fluid2D

@export var radius := 10:
	set(value):
		if radius != value:
			radius = value
			points = create_circle_points(radius)
	get:
		return radius
