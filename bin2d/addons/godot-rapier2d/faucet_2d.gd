class_name Faucet2D
extends Fluid2D

@export var interval := 0.06
@export var max_particles: int = 1000
@export var width: int = 4
@export var height: int = 2

var points_new: PackedVector2Array
var velocities_new: PackedVector2Array


func _ready():
	points_new = create_rectangle_points(width, height)
	velocities_new.resize(points_new.size())
	var gravity_value = ProjectSettings.get("physics/2d/default_gravity")
	var gravity_dir = ProjectSettings.get("physics/2d/default_gravity_vector")
	var dir = global_transform.basis_xform(gravity_dir * gravity_value)
	velocities_new.fill(dir)
	get_tree().create_timer(interval).timeout.connect(_on_timer_timeout)


func _on_timer_timeout():
	get_tree().create_timer(interval).timeout.connect(_on_timer_timeout)
	if len(points) > max_particles:
		return
	add_points_and_velocities(points_new, velocities_new)
