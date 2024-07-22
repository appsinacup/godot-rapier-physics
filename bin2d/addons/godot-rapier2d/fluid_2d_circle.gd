@tool
extends Fluid2D

@export var circle_radius := 10:
	set(value):
		if circle_radius != value:
			circle_radius = value
			points = create_circle_points(circle_radius)
	get:
		return circle_radius
