extends Node2D

@onready var body := preload("res://test/integration/rigid_body_2d_delete.tscn")

func _on_timer_timeout() -> void:
	var body1 := body.instantiate() as RigidBody2D
	add_child(body1)
