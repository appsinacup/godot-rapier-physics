class_name Faucet3D
extends Fluid3D

@export var interval := 0.06
@export var max_particles: int = 1000
@export var width: int = 4
@export var height: int = 1
@export var depth: int = 4

var points_new: PackedVector3Array
var velocities_new: PackedVector3Array


func _ready():
	points_new = create_box_points(width, height, depth)
	velocities_new.resize(points_new.size())
	var gravity_value = ProjectSettings.get("physics/3d/default_gravity")
	var gravity_dir = ProjectSettings.get("physics/3d/default_gravity_vector")
	var dir = global_transform.basis * gravity_dir * gravity_value
	velocities_new.fill(dir)
	get_tree().create_timer(interval).timeout.connect(_on_timer_timeout)


func _on_timer_timeout():
	get_tree().create_timer(interval).timeout.connect(_on_timer_timeout)
	if len(points) > max_particles:
		return
	add_points_and_velocities(points_new, velocities_new)
