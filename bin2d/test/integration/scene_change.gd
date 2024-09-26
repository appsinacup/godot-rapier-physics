extends Node2D

@export var rigidbody: RigidBody2D
@onready var space := get_viewport().world_2d.space

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	FileAccess.open("user://rigidbody.json", FileAccess.WRITE).store_string(RapierPhysicsServer2D.body_export_json(rigidbody.get_rid()))
	FileAccess.open("user://space.json", FileAccess.WRITE).store_string(RapierPhysicsServer2D.space_export_json(space))
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
