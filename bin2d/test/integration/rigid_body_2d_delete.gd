extends RigidBody2D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	position = Vector2(randf() * 100.0, randf() * 100.0)
	apply_central_force(Vector2(randf() * 100, randf() * 100))


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	if randf() < 0.001:
		queue_free()
