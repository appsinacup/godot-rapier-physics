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
		rapier_state.export_state()
		print(rapier_state.save_state(true))
		FileAccess.open("user://save.debug.json", FileAccess.WRITE).store_string(JSON.stringify(rapier_state.state, " "))
	else:
		rapier_state.import_state()
		FileAccess.open("user://load.json", FileAccess.WRITE).store_string(JSON.stringify(rapier_state.state, " "))
		print(rapier_state.save_state(false))
		FileAccess.open("user://load.save.json", FileAccess.WRITE).store_string(JSON.stringify(rapier_state.state, " "))
		print(rapier_state.save_state(true))
		FileAccess.open("user://load.save.debug.json", FileAccess.WRITE).store_string(JSON.stringify(rapier_state.state, " "))
	print("After")
	print(RapierPhysicsServer2D.get_stats())
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
