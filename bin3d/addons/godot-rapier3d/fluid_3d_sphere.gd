@tool
extends Fluid3D

@export var sphere_radius := 10:
	set(value):
		if sphere_radius != value:
			sphere_radius = value
			points = create_sphere_points(sphere_radius)
	get:
		return sphere_radius
