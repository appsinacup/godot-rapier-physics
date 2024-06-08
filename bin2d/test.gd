extends Node2D

@export var body: RigidBody2D

func _on_timer_timeout():
	get_tree().quit()

func _ready():
	var space := get_viewport().find_world_2d().direct_space_state as RapierDirectSpaceState2D
	var physics_server = PhysicsServer2D
	#physics_server.body_set_extra_param(body.get_rid(), 0, 20.0)

	#var space_json := space.export_json()
	#print(space_json)
