extends PhysicsUnitTest2D

class Launcher2D:
	extends CanvasGroup

	var ball_scene: PackedScene
	var launch_mode := 0
	var _current_ball: RigidBody2D = null
	var launched_balls: Array[RigidBody2D] = []

	func _input(event: InputEvent) -> void:
		if event is InputEventMouseButton:
			launch()

	func launch() -> void:
		var new_ball := ball_scene.instantiate() as RigidBody2D
		new_ball.gravity_scale = 0.0
		new_ball.contact_monitor = true
		new_ball.max_contacts_reported = 8
		if launch_mode == 0:
			add_child(new_ball)
			new_ball.reparent(get_tree().root)
		else:
			get_tree().root.add_child(new_ball)
			new_ball.global_position = global_position
		_current_ball = new_ball
		launched_balls.append(new_ball)

const BALL_SCENE := preload("res://tests/nodes/RigidBody/tests/2d/input_reparent_ball.tscn")
const REPARENT_POSITION := Vector2(0.0, -600.0)
const ROOT_SET_POSITION := Vector2(180.0, -600.0)
const POSITION_TOLERANCE := 96.0
const ORIGIN_RESET_TOLERANCE := 4.0

var reparent_launcher: Launcher2D
var root_set_launcher: Launcher2D
var origin_blocker: StaticBody2D
var input_sent := false

func test_description() -> String:
	return """Checks that a RigidBody2D added under a launcher and reparented to root from _input keeps its transform."""

func test_name() -> String:
	return "RigidBody2D | _input reparent keeps transform"

func test_start() -> void:
	origin_blocker = create_origin_blocker()
	reparent_launcher = create_launcher(REPARENT_POSITION, 0)
	root_set_launcher = create_launcher(ROOT_SET_POSITION, 1)

	var reparent_test = func(_p_target: PhysicsTest2D, p_monitor: GenericManualMonitor):
		var input_received := reparent_launcher._current_ball != null and root_set_launcher._current_ball != null
		if not input_received and p_monitor.frame < 30:
			return
		if p_monitor.frame < 6:
			return

		p_monitor.add_test("Launcher spawned a ball from _input")
		if not input_received:
			p_monitor.add_test_error("Input event did not spawn the rigid body")
		p_monitor.add_test_result(input_received)

		p_monitor.add_test("Reparented RigidBody2D does not reset to origin")
		add_position_result(p_monitor, reparent_launcher._current_ball, REPARENT_POSITION)

		p_monitor.add_test("Root-added RigidBody2D does not reset to origin")
		add_position_result(p_monitor, root_set_launcher._current_ball, ROOT_SET_POSITION)

		p_monitor.add_test("RigidBody2D does not collide at origin after _input launch")
		add_no_origin_collision_result(p_monitor)
		cleanup_reparented_balls()
		p_monitor.monitor_completed()

	var monitor := create_generic_manual_monitor(self, reparent_test, 2.0)
	monitor.test_name = "_input reparent preserves transform"
	launch_from_code_deferred()

func create_launcher(p_position: Vector2, p_launch_mode: int) -> Launcher2D:
	var instance := Launcher2D.new()
	instance.ball_scene = BALL_SCENE
	instance.launch_mode = p_launch_mode
	instance.position = p_position
	instance.set_process_input(true)
	add_child(instance)
	return instance

func create_origin_blocker() -> StaticBody2D:
	var body := StaticBody2D.new()
	body.add_child(PhysicsTest2D.get_collision_shape(Rect2(-25.0, -25.0, 50.0, 50.0), TestCollisionShape.RECTANGLE, true))
	add_child(body)
	return body

func launch_from_code_deferred() -> void:
	await get_tree().process_frame
	launch_from_code()

func launch_from_code() -> void:
	if input_sent:
		return
	input_sent = true
	var event := InputEventMouseButton.new()
	event.button_index = MOUSE_BUTTON_LEFT
	event.pressed = true
	event.position = CENTER
	event.global_position = CENTER
	get_viewport().push_input(event, true)

func cleanup_reparented_balls() -> void:
	for launcher in [reparent_launcher, root_set_launcher]:
		for body in launcher.launched_balls:
			if is_instance_valid(body):
				body.queue_free()

func add_position_result(p_monitor: GenericManualMonitor, p_body: RigidBody2D, p_expected_position: Vector2) -> void:
	var transform_kept := false
	if p_body != null:
		var actual_position := p_body.global_position
		var server_transform: Transform2D = PhysicsServer2D.body_get_state(p_body.get_rid(), PhysicsServer2D.BODY_STATE_TRANSFORM)
		var server_position := server_transform.origin
		var node_reset := actual_position.distance_to(Vector2.ZERO) <= ORIGIN_RESET_TOLERANCE
		var server_reset := server_position.distance_to(Vector2.ZERO) <= ORIGIN_RESET_TOLERANCE
		transform_kept = not node_reset and not server_reset
		if not transform_kept:
			p_monitor.add_test_error("Expected non-origin position near %v, got node %v and server %v" % [p_expected_position, actual_position, server_position])
	else:
		p_monitor.add_test_error("RigidBody2D was not created")
	p_monitor.add_test_result(transform_kept)

func add_no_origin_collision_result(p_monitor: GenericManualMonitor) -> void:
	var passed := true
	for launcher in [reparent_launcher, root_set_launcher]:
		for body in launcher.launched_balls:
			if not is_instance_valid(body):
				continue
			if body.get_colliding_bodies().has(origin_blocker):
				passed = false
				p_monitor.add_test_error("RigidBody2D collided with origin blocker from launch mode %d at %v" % [launcher.launch_mode, body.global_position])
	p_monitor.add_test_result(passed)
