extends Node2D

@export var fluid :Fluid2D

var points: PackedVector2Array
var velocities: PackedVector2Array

func _ready():
	points = fluid.points
	velocities.resize(points.size())
	velocities.fill(Vector2(0, 980))

func _on_timer_timeout():
	if len(fluid.points) > 2000:
		return
	fluid.set_points_and_velocities(fluid.points+points, fluid.velocities+velocities)
	print(len(fluid.points))
