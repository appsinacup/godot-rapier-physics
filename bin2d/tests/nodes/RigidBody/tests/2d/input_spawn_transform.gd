extends PhysicsUnitTest2D

class Launcher2D:
	extends CanvasGroup

	var ball_scene: PackedScene
	var _current_ball: RigidBody2D = null
	var launched_balls: Array[RigidBody2D] = []

	func _input(event: InputEvent) -> void:
		if event is InputEventMouseButton:
			launch()

	func launch() -> void:
		var new_ball := ball_scene.instantiate() as RigidBody2D
		add_child(new_ball)
		new_ball.reparent(get_tree().root)
		_current_ball = new_ball
		launched_balls.append(new_ball)

const SPAWN_POSITION := Vector2(180.0, -600.0)
const IMPULSE_POSITION := Vector2(360.0, -600.0)
const REENABLE_POSITION := Vector2(540.0, -600.0)
const REPARENT_POSITION := Vector2(720.0, -600.0)
const SPAWN_IMPULSE := Vector2(120.0, 0.0)
const POSITION_TOLERANCE := 96.0
const ORIGIN_RESET_TOLERANCE := 4.0

var simulation_duration := 1.0
var spawned_body: RigidBody2D
var impulse_body: RigidBody2D
var reenable_body: RigidBody2D
var launcher: Launcher2D
var input_sent := false
var input_received := false

func test_description() -> String:
	return """Checks that RigidBody2D operations from _input keep non-zero initial transforms."""

func test_name() -> String:
	return "RigidBody2D | _input operations keep transform"

func test_start() -> void:
	set_process_input(true)
	impulse_body = create_offset_body(IMPULSE_POSITION)
	reenable_body = create_offset_body(REENABLE_POSITION)
	reenable_body.process_mode = Node.PROCESS_MODE_DISABLED
	launcher = Launcher2D.new()
	launcher.ball_scene = create_ball_scene()
	launcher.global_position = REPARENT_POSITION
	launcher.set_process_input(true)
	add_child(launcher)

	var input_spawn_test = func(_p_target: PhysicsTest2D, p_monitor: GenericManualMonitor):
		var launcher_spawned := launcher._current_ball != null
		if (not input_received or not launcher_spawned) and p_monitor.frame < 30:
			return
		if p_monitor.frame < 6:
			return

		p_monitor.add_test("Input event reached _input")
		if not input_received:
			p_monitor.add_test_error("Input event did not reach test _input")
		p_monitor.add_test_result(input_received)

		add_position_result(p_monitor, spawned_body, SPAWN_POSITION, "Spawned RigidBody2D keeps non-zero transform")
		add_position_result(p_monitor, impulse_body, IMPULSE_POSITION, "RigidBody2D impulse from _input keeps non-zero transform")
		add_position_result(p_monitor, reenable_body, REENABLE_POSITION, "RigidBody2D re-enabled from _input keeps non-zero transform")
		add_position_result(p_monitor, launcher._current_ball, REPARENT_POSITION, "RigidBody2D reparented from _input keeps non-zero transform")
		cleanup_reparented_body()
		p_monitor.monitor_completed()

	var monitor := create_generic_manual_monitor(self, input_spawn_test, 2.0)
	monitor.test_name = "_input operations preserve transforms"
	_send_spawn_input_deferred()

func _send_spawn_input_deferred() -> void:
	await get_tree().process_frame
	_send_spawn_input()

func _send_spawn_input() -> void:
	if input_sent:
		return
	input_sent = true
	var event := InputEventMouseButton.new()
	event.button_index = MOUSE_BUTTON_LEFT
	event.pressed = true
	event.position = CENTER
	event.global_position = CENTER
	get_viewport().push_input(event, true)

func _input(event: InputEvent) -> void:
	if input_received:
		return
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		input_received = true
		spawned_body = create_offset_body(SPAWN_POSITION)
		spawned_body.apply_central_impulse(SPAWN_IMPULSE)
		impulse_body.apply_central_impulse(SPAWN_IMPULSE)
		reenable_body.process_mode = Node.PROCESS_MODE_INHERIT
		reenable_body.apply_central_impulse(SPAWN_IMPULSE)

func create_offset_body(p_global_position: Vector2) -> RigidBody2D:
	var parent := Node2D.new()
	parent.position = p_global_position
	add_child(parent)

	var body := RigidBody2D.new()
	body.add_child(PhysicsTest2D.get_default_collision_shape(TestCollisionShape.RECTANGLE))
	body.gravity_scale = 0.0
	body.position = Vector2.ZERO
	parent.add_child(body)
	return body

func add_position_result(p_monitor: GenericManualMonitor, p_body: RigidBody2D, p_expected_position: Vector2, p_test_name: String) -> void:
	p_monitor.add_test(p_test_name)
	var transform_kept := false
	if p_body != null:
		var actual_position := p_body.global_position
		var server_transform: Transform2D = PhysicsServer2D.body_get_state(p_body.get_rid(), PhysicsServer2D.BODY_STATE_TRANSFORM)
		var server_position := server_transform.origin
		var node_reset := actual_position.distance_to(Vector2.ZERO) <= ORIGIN_RESET_TOLERANCE
		var server_reset := server_position.distance_to(Vector2.ZERO) <= ORIGIN_RESET_TOLERANCE
		transform_kept = not node_reset and not server_reset and actual_position.distance_to(p_expected_position) <= POSITION_TOLERANCE and server_position.distance_to(p_expected_position) <= POSITION_TOLERANCE
		if not transform_kept:
			p_monitor.add_test_error("Expected position near %v, got node %v and server %v" % [p_expected_position, actual_position, server_position])
	else:
		p_monitor.add_test_error("RigidBody2D was not created")
	p_monitor.add_test_result(transform_kept)

func create_ball_scene() -> PackedScene:
	var ball := RigidBody2D.new()
	var shape := PhysicsTest2D.get_default_collision_shape(TestCollisionShape.RECTANGLE)
	ball.add_child(shape)
	shape.owner = ball
	var scene := PackedScene.new()
	var error := scene.pack(ball)
	assert(error == OK)
	ball.free()
	return scene

func cleanup_reparented_body() -> void:
	if launcher == null:
		return
	for body in launcher.launched_balls:
		if is_instance_valid(body):
			body.queue_free()
