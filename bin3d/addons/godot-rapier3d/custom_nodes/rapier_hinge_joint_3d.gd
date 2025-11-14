class_name RapierHingeJoint3D
extends HingeJoint3D

@export_enum("Impulse", "Multibody", "Multibody Kinematic") var joint_type: int = 0:
	get:
		return joint_type
	set(value):
		if value != joint_type:
			joint_type = value
			set_joint_type(value)

func _init() -> void:
	set_joint_type(joint_type)

func set_joint_type(joint_type: int) -> void:
	RapierPhysicsServer3D.joint_set_extra_param(get_rid(), RapierPhysicsServer3D.JOINT_TYPE, joint_type)
