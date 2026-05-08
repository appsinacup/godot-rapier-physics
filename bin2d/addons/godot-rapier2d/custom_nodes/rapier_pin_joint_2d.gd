class_name RapierPinJoint2D
extends PinJoint2D

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
		_update_ik_options()

@export_range(1, 100, 1) var ik_max_iterations: int = 10:
	set(value):
		ik_max_iterations = value
		_update_ik_options()


@export_group("Motor Position", "motor_position_")
## Motor Position Targeting acts like a spring to pull the joint
## back to the target angle
@export_custom(PROPERTY_HINT_GROUP_ENABLE, "") var motor_position_enabled: bool = false:
	set(value):
		motor_position_enabled = value
		_update_motor_position_options()

@export_range(-180, 180, 0.1, "radians_as_degrees") var motor_position_target_angle: float = 0.0:
	set(value):
		motor_position_target_angle = value
		_update_motor_position_options()

@export var motor_position_stiffness: float = 0.0:
	set(value):
		motor_position_stiffness = value
		_update_motor_position_options()

@export var motor_position_damping: float = 0.0:
	set(value):
		motor_position_damping = value
		_update_motor_position_options()

@export_group("","")

var _ik_constrained_axes: int = 3


func _init() -> void:
	set_joint_type(joint_type)

func _ready() -> void:
	_update_constrained_axes()
	_update_ik_options()
	_update_motor_position_options()

func _physics_process(delta: float) -> void:
	_solve_ik_for_target()

func _solve_ik_for_target() -> void:
	if not is_inside_tree() or ik_target == null:
		return
	
	if joint_type != 1 and joint_type != 2:
		return

	if not _has_valid_joint_nodes():
		return

	solve_ik(ik_target.global_transform)

func _has_valid_joint_nodes() -> bool:
	return get_node_or_null(node_a) != null and get_node_or_null(node_b) != null

func _update_ik_options() -> void:
	if not is_inside_tree():
		return
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		push_error("Invalid joint rid")
		return
	
	RapierPhysicsServer2D.joint_set_ik_options(
		joint_rid,
		ik_damping,
		ik_max_iterations,
		_ik_constrained_axes,
		0.001,
		0.001,
	)


func _update_motor_position_options() -> void:
	if not is_inside_tree():
		return
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		push_error("Invalid joint rid")
		return
		
	# workaround what seems like a rapier issue
	# stiffness of 0.0 behaves strangely
	# expected behaviour is for joint to hang freely
	if is_zero_approx(motor_position_stiffness):
		motor_position_stiffness = 0.00001
	RapierPhysicsServer2D.joint_set_motor_position_options(
		joint_rid,
		motor_position_target_angle,
		motor_position_stiffness,
		motor_position_damping,
		motor_position_enabled,
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
	if not _has_valid_joint_nodes():
		return
	RapierPhysicsServer2D.joint_solve_inverse_kinematics(joint_rid, target_transform)
