class_name RapierPinJoint2D
extends PinJoint2D

## A pin joint for the Rapier physics engine. A pin joint lets two bodies rotate around a common anchor point.
##
## Pin joints can be created to join two bodies or to join a body to a fixed world location.

## Type of joint (0 = Impulse, 1 = Multibody, 2 = Multibody Kinematic)
@export_enum("Impulse", "Multibody", "Multibody Kinematic") var joint_type: int = 0:
	get:
		return joint_type
	set(value):
		if value != joint_type:
			joint_type = value
			set_joint_type(value)


## The target node for inverse kinematics (only works with multibody joints)
@export var ik_target: Node2D = null:
	set(value):
		ik_target = value
		update_configuration_warnings()

@export_group("IK Constraints")

## Control X position with IK
@export var ik_constrain_x: bool = true:
	set(value):
		ik_constrain_x = value
		_update_constrained_axes()

## Control Y position with IK
@export var ik_constrain_y: bool = true:
	set(value):
		ik_constrain_y = value
		_update_constrained_axes()

## Control rotation with IK
@export var ik_constrain_rotation: bool = false:
	set(value):
		ik_constrain_rotation = value
		_update_constrained_axes()

@export_group("IK Solver Settings")

## Custom IK damping
@export_range(0.0, 100.0, 0.1) var ik_damping: float = 1.0:
	set(value):
		ik_damping = value
		if is_inside_tree():
			_update_ik_options()

## Custom IK max iterations
@export_range(1, 32, 1) var ik_max_iterations: int = 10:
	set(value):
		ik_max_iterations = value
		if is_inside_tree():
			_update_ik_options()

## Custom IK linear epsilon (threshold for position convergence)
@export_range(0.001, 1.0, 0.001) var ik_epsilon_linear: float = 0.001:
	set(value):
		ik_epsilon_linear = value
		if is_inside_tree():
			_update_ik_options()

## Custom IK angular epsilon (threshold for rotation convergence)
@export_range(0.001, 1.0, 0.001) var ik_epsilon_angular: float = 0.001:
	set(value):
		ik_epsilon_angular = value
		if is_inside_tree():
			_update_ik_options()

# Internal state for tracking target changes
var _last_target_position: Vector2
var _last_target_rotation: float
# Internal computed value
var ik_constrained_axes: int = 3


func _init() -> void:
	set_joint_type(joint_type)

func _ready() -> void:
	update_configuration_warnings()
	_update_constrained_axes()
	_update_ik_options()
	if ik_target:
		_last_target_position = ik_target.global_position
		_last_target_rotation = ik_target.global_rotation

func _physics_process(_delta: float) -> void:
	# Automatically solve IK if target is set and at least one constraint is enabled
	if ik_target != null and ik_constrained_axes > 0:
		_solve_ik_for_target()
		_last_target_position = ik_target.global_position
		_last_target_rotation = ik_target.global_rotation

func _solve_ik_for_target() -> void:
	if not is_inside_tree() or ik_target == null:
		return
	
	# Only works for multibody joints
	if joint_type != 1 and joint_type != 2:
		return
	
	# Build target transform from current target position/rotation
	var target_transform := Transform2D(ik_target.global_rotation, ik_target.global_position)
	
	# Solve IK
	solve_ik(target_transform)

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
		ik_constrained_axes,
		ik_epsilon_linear,
		ik_epsilon_angular,
	)

func _update_constrained_axes() -> void:
	# Build bitmask from checkboxes
	ik_constrained_axes = 0
	if ik_constrain_x:
		ik_constrained_axes |= 1  # Bit 0
	if ik_constrain_y:
		ik_constrained_axes |= 2  # Bit 1
	if ik_constrain_rotation:
		ik_constrained_axes |= 32  # Bit 5
	
	if is_inside_tree():
		_update_ik_options()


func set_joint_type(type: int) -> void:
	RapierPhysicsServer2D.joint_set_extra_param(get_rid(), RapierPhysicsServer2D.JOINT_TYPE, type)

## Solve inverse kinematics to move the end effector to a target transform.
## This only works for multibody joints (joint_type = 1 or 2).
## The constrained axes determine which parts of the transform are used.
## Returns true if IK converged successfully.
func solve_ik(target_transform: Transform2D) -> bool:
	if not is_inside_tree():
		return false
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		return false
	
	return RapierPhysicsServer2D.joint_solve_inverse_kinematics(joint_rid, target_transform)

## Get the current end effector transform for this multibody joint.
## This only works for multibody joints (joint_type = 1 or 2).
func get_link_transform() -> Transform2D:
	if not is_inside_tree():
		return Transform2D.IDENTITY
	
	var joint_rid := get_rid()
	if not joint_rid.is_valid():
		return Transform2D.IDENTITY
	
	return RapierPhysicsServer2D.joint_get_link_transform(joint_rid)

func _get_configuration_warnings() -> PackedStringArray:
	var warnings: PackedStringArray = []
	
	if ik_target != null and ik_constrained_axes > 0:
		if joint_type == 0:
			warnings.append("IK target only works with Multibody joints (joint_type = 1 or 2)")
	
	return warnings
