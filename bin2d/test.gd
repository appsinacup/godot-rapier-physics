extends Node2D

@export var body: RigidBody2D
var collider:= RapierColliderJSON.new()

func _on_timer_timeout():
	get_tree().quit()


func _process(delta: float) -> void:
	var space := get_viewport().find_world_2d().direct_space_state as RapierDirectSpaceState2D
	var space_json = JSON.parse_string(space.export_json())
	var save_world := FileAccess.open("user://world.json", FileAccess.WRITE)
	#save_world.store_var(JSON.stringify(space_json, " "))
	save_world.store_string(space.export_json())
