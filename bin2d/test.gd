extends Node2D

@export var body: RigidBody2D
var collider:= RapierColliderJSON.new()

func _on_timer_timeout():
	get_tree().quit()


func _process(delta: float) -> void:
	return
	var space := get_viewport().find_world_2d().direct_space_state as RapierDirectSpaceState2D
	var space_json := FileAccess.open("user://shapes.json", FileAccess.WRITE)
	space_json.store_string(space.export_json())
