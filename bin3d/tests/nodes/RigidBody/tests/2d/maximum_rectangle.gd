extends PhysicsUnitTest2D

var timer: Timer
var body_size := Vector2(25, 25)
var min_body_expected := 550
var simulation_duration := 15

var bodies : Array[RigidBody2D] = []

func test_description() -> String:
	return """Checks the maximum number of supported rectangles before the simulation goes crazy.
	"""
	
func test_name() -> String:
	return "RigidBody2D | testing if %d rectangles can be handled before instablity" % [min_body_expected]

func test_start() -> void:
	add_collision_bottom(1)

	timer = Timer.new()
	timer.wait_time = 0.2
	timer.process_callback =Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(spawn_body)
	add_child(timer)
	timer.start()
	
	
	var maximum_bodies_supported = func(p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		var _bodies := p_target.bodies as Array[RigidBody2D]
		for body in _bodies:
			if body.position.y > (Global.WINDOW_SIZE.y) or _bodies.size() >= min_body_expected:
				p_target.timer.stop()
				if _bodies.size() >= min_body_expected:
					p_monitor.passed()
				else:
					p_monitor.test_name += " → [color=orange]failed at %d[/color]" % [_bodies.size()]
					p_monitor.failed()
	
	var check_max_stability_monitor := create_generic_manual_monitor(self, maximum_bodies_supported, simulation_duration)
	check_max_stability_monitor.test_name = "Handle at least %d bodies" % [min_body_expected]

func spawn_body() -> void:
	var offset = (Global.WINDOW_SIZE.x - 100) / 19
	for i in range(20):
		var body = _get_rigid_body(TOP_LEFT + Vector2(50 + i * offset, 0))
		bodies.append(body)
		add_child(body)
	
func _get_rigid_body(p_position: Vector2) -> RigidBody2D:
	var body = RigidBody2D.new()
	var shape = PhysicsTest2D.get_collision_shape(Rect2(Vector2(0, 0), body_size), TestCollisionShape.RECTANGLE, false)
	body.add_child(shape)
	p_position.x = int(p_position.x)
	p_position.y = int(p_position.y)
	body.position = p_position
	return body
