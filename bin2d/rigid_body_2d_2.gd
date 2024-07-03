extends PhysicsBody2D


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print(get_rid())
	RapierPhysicsServer2D.body_set_extra_param(self.get_rid(), 0, 0)


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
