extends CollisionShape2D


# Called when the node enters the scene tree for the first time.

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	shape.size = Vector2(randf() * 100, randf() * 100)
	shape = null
	shape = RectangleShape2D.new()
	shape.size = Vector2(randf() * 100, randf() * 100)
