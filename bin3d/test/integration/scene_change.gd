extends Node2D

@export var rapier_state : Rapier2DState
@export var save_state: bool = true

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	# Disable so it won't run
	PhysicsServer2D.set_active(false)
	print("Before")
	print(RapierPhysicsServer2D.get_stats())
	if save_state:
		print(rapier_state.save_state(false))
		FileAccess.open("user://save.json", FileAccess.WRITE).store_string(JSON.stringify(rapier_state.state, " "))
	else:
		var state = JSON.parse_string(FileAccess.open("user://save.json", FileAccess.READ).get_as_text())
		rapier_state.state = state
		print(rapier_state.load_state())
		print(rapier_state.save_state(false))
	print("After")
	print(RapierPhysicsServer2D.get_stats())
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
