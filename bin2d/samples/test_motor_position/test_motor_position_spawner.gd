extends Node2D

@export var mass : float = 1.0

func spawn_circle_body()->void:		
	var rb := RigidBody2D.new()
	var cs := CollisionShape2D.new()
	var sp := CircleShape2D.new()
	var vn := VisibleOnScreenNotifier2D.new()
	sp.radius = 10.0	 * randf_range(0.5, 3.0)
	cs.shape = sp
	
	rb.mass = mass * (sp.radius / 20.0) ** 3
	rb.transform.origin.x = randf_range(-200.0, 200.0)
	rb.add_child(cs)
	rb.add_child(vn)
	add_child(rb)	
	vn.screen_exited.connect(_on_screen_exited.bind(rb))

func _on_screen_exited(rb: RigidBody2D)->void:	
	rb.queue_free()
	
func _on_spawn_timer_timeout() -> void:
	spawn_circle_body()
