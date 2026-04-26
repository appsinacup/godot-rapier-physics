class_name Faucet2D
extends Fluid2D

@export_range(0, 1, 0.01, "or_greater") var interval: float = 0.06
@export_range(0, 1024, 1, "or_greater") var max_particles: int = 1000
@export_range(0, 8, 1, "or_greater") var width: int = 4
@export_range(0, 4, 1, "or_greater") var height: int = 2

var points_new: PackedVector2Array
var velocities_new: PackedVector2Array

var timer: Timer

func _ready():
	timer = Timer.new()
	timer.name = &"Fluid2DTimer"
	timer.timeout.connect(_on_timer_timeout)
	add_child(timer, false, Node.INTERNAL_MODE_DISABLED)
	#timer.one_shot = true
	points_new = create_rectangle_points(width, height)
	velocities_new.resize(points_new.size())
	velocities_new.fill(global_transform.basis_xform(ProjectSettings.get("physics/2d/default_gravity_vector") * ProjectSettings.get("physics/2d/default_gravity")))
	timer.start(interval)


func _on_timer_timeout():
	if points.size() > max_particles:
		return
	add_points_and_velocities(points_new, velocities_new)
