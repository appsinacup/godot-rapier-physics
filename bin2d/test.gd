extends Node2D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var file = FileAccess.open("user://space.json", FileAccess.WRITE)
	var space_state = RapierPhysicsServer2D.space_export_json(get_viewport().world_2d.space)
	file.store_string(space_state)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	if Input.is_key_pressed(KEY_B):
		get_tree().reload_current_scene()
