class_name RapierGeneric6DOFJoint3D
extends Generic6DOFJoint3D

@export var is_multibody: bool = false:
	get:
		return is_multibody
	set(value):
		if value != is_multibody:
			is_multibody = value
			set_is_multibody(value)

func _ready() -> void:
	set_is_multibody(is_multibody)

func set_is_multibody(enabled: bool) -> void:
	if enabled:
		RapierPhysicsServer3D.joint_set_extra_param(get_rid(), RapierPhysicsServer3D.JOINT_TYPE, RapierPhysicsServer3D.MULTIBODY_JOINT)
	else:
		RapierPhysicsServer3D.joint_set_extra_param(get_rid(), RapierPhysicsServer3D.JOINT_TYPE, RapierPhysicsServer3D.IMPULSE_JOINT)
