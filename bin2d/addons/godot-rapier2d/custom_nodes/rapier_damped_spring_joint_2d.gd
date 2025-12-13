class_name RapierDampedSpringJoint2D
extends DampedSpringJoint2D

@export_enum("Impulse", "Multibody") var joint_type: int = 0:
	get:
		return joint_type
	set(value):
		if value != joint_type:
			joint_type = value
			set_joint_type(value)

@export var ik_target: Node2D = null:
	set(value):
		ik_target = value

@export_group("IK Constraints")

@export var ik_constrain_x: bool = true:
	set(value):
		ik_constrain_x = value

@export var ik_constrain_y: bool = true:
	set(value):
		ik_constrain_y = value

@export var ik_constrain_rotation: bool = false:
	set(value):
		ik_constrain_rotation = value

@export_group("IK Solver Settings")

@export_range(0.0, 100.0, 0.01) var ik_damping: float = 1.0:
	set(value):
		ik_damping = value
		if is_inside_tree():
			_update_ik_options()

@export_range(1, 100, 1) var ik_max_iterations: int = 10:
	set(value):
		ik_max_iterations = value
		if is_inside_tree():
			_update_ik_options()

var _ik_constrained_axes: int = 3


func _init() -> void:
	set_joint_type(joint_type)

func _ready() -> void:
	_update_constrained_axes()
	_update_ik_options()

func _physics_process(delta: float) -> void:
	_solve_ik_for_target()

func _solve_ik_for_target() -> void:
	if not is_inside_tree() or ik_target == null:
		return
	
	if joint_type != 1 and joint_type != 2:
		return
	
	solve_ik(ik_target.global_transform)

func _update_ik_options() -> void:
	if not is_inside_tree():
		return
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		return
	
	RapierPhysicsServer2D.joint_set_ik_options(
		joint_rid,
		ik_damping,
		ik_max_iterations,
		_ik_constrained_axes,
		0.001,
		0.001,
	)

func _update_constrained_axes() -> void:
	_ik_constrained_axes = 0
	if ik_constrain_x:
		_ik_constrained_axes |= 1
	if ik_constrain_y:
		_ik_constrained_axes |= 2
	if ik_constrain_rotation:
		_ik_constrained_axes |= 4
	
	if is_inside_tree():
		_update_ik_options()


func set_joint_type(type: int) -> void:
	RapierPhysicsServer2D.joint_set_extra_param(get_rid(), RapierPhysicsServer2D.JOINT_TYPE, type)

func solve_ik(target_transform: Transform2D) -> void:
	if not is_inside_tree():
		return
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		return
	
	RapierPhysicsServer2D.joint_solve_inverse_kinematics(joint_rid, target_transform)
