class_name RapierGrooveJoint2D
extends GrooveJoint2D

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
	RapierPhysicsServer2D.joint_set_extra_param(get_rid(), RapierPhysicsServer2D.JOINT_TYPE, enabled)
