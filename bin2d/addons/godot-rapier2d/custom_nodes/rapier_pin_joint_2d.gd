class_name RapierPinJoint2D
extends PinJoint2D

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
		RapierPhysicsServer2D.joint_set_extra_param(get_rid(), RapierPhysicsServer2D.JOINT_TYPE, RapierPhysicsServer2D.MULTIBODY_JOINT)
	else:
		RapierPhysicsServer2D.joint_set_extra_param(get_rid(), RapierPhysicsServer2D.JOINT_TYPE, RapierPhysicsServer2D.IMPULSE_JOINT)
