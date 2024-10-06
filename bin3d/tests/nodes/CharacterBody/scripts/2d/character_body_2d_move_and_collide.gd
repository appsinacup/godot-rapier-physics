extends CharacterBody2D

var distance: Vector2
var test_only: bool = false

func _physics_process(delta: float) -> void:
	move_and_collide(distance, test_only, safe_margin)
