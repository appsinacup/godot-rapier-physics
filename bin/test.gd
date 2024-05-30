extends Node2D

func _on_timer_timeout():
	get_tree().quit()

func _ready():
	var space := get_viewport().find_world_2d().direct_space_state as RapierDirectSpaceState2D
	#var space_json := space.export_json()
	#print(space_json)
