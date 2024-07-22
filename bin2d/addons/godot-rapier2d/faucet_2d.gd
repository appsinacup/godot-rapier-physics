class_name Faucet2D
extends Fluid2D

var points_new: PackedVector2Array
var velocities_new: PackedVector2Array
@export var max_particles: int = 1000

func _ready():
	points_new = points
	velocities_new.resize(points.size())
	velocities_new.fill(Vector2(0, 980))

func _on_timer_timeout():
	if len(points) > max_particles:
		return
	add_points_and_velocities(points_new, velocities_new)
