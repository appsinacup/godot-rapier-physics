class_name RapierRigidBody2D
extends RigidBody2D

@export var massless: bool = false:
	get:
		return massless
	set(value):
		if value != massless:
			massless = value
			set_massless(value)

@export var body_skin: float = 0.0:
	get:
		return body_skin
	set(value):
		if value != body_skin:
			body_skin = value
			set_body_skin(value)

@export var dominance: int = 0:
	get:
		return dominance
	set(value):
		if value != dominance:
			dominance = value
			set_dominance(value)

@export var soft_ccd: float = 0.0:
	get:
		return soft_ccd
	set(value):
		if value != soft_ccd:
			soft_ccd = value
			set_soft_ccd(value)

func _init() -> void:
	set_massless(massless)
	set_body_skin(body_skin)
	set_dominance(dominance)
	set_soft_ccd(soft_ccd)

func set_body_skin(value: float) -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.BODY_PARAM_CONTACT_SKIN, value)

func set_dominance(value: int) -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.BODY_PARAM_DOMINANCE, value)

func set_soft_ccd(value: float) -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.BODY_PARAM_SOFT_CCD, value)

func set_massless(value: bool) -> void:
	RapierPhysicsServer2D.body_set_extra_param(get_rid(), RapierPhysicsServer2D.BODY_PARAM_MASSLESS, value)
