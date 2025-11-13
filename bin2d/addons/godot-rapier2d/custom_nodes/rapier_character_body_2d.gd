class_name RapierCharacterBody2D
extends CharacterBody2D

@export var body_skin: float = 0.0:
	get:
		return body_skin
	set(value):
		if value != body_skin:
			body_skin = value
			set_body_skin(value)

func _ready() -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.CONTACT_SKIN, body_skin)

func set_body_skin(value: float) -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.CONTACT_SKIN, value)
