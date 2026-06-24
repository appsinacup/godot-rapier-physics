extends PhysicsUnitTest2D

const SPAWN_SCALE := Vector2(0.005539404, 6.0135145)
const VISUAL_SCALE := Vector2(20.0, 20.0)
const SCALE_TOLERANCE := 0.01

var spawned_body: RigidBody2D
var spawned_visual: Sprite2D

func test_description() -> String:
	return """Checks that a RigidBody2D instantiated under a scaled parent does not keep that parent scale after physics sync."""

func test_name() -> String:
	return "RigidBody2D | parent scale does not stretch spawned scene"

func test_start() -> void:
	var spawn := Node2D.new()
	spawn.name = "Spawn"
	spawn.position = Vector2(119.0, -206.0)
	spawn.scale = SPAWN_SCALE
	add_child(spawn)

	var packed_scene := create_spawned_scene()
	var spawned_scene := packed_scene.instantiate() as Node2D
	spawn.add_child(spawned_scene)
	spawned_body = spawned_scene.get_node("RigidBody2D")
	spawned_visual = spawned_body.get_node("Sprite2D")

	var scale_test = func(_p_target: PhysicsTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame < 4:
			return

		p_monitor.add_test("RigidBody2D server transform strips parent scale")
		add_body_scale_result(p_monitor)

		p_monitor.add_test("Sprite2D visual keeps its authored scale")
		add_visual_scale_result(p_monitor)

		p_monitor.monitor_completed()

	var monitor := create_generic_manual_monitor(self, scale_test, 2.0)
	monitor.test_name = "Instantiated scene under scaled parent keeps body scale"

func create_spawned_scene() -> PackedScene:
	var scene_root := Node2D.new()
	scene_root.name = "SpawnedScene"

	var body := RigidBody2D.new()
	body.name = "RigidBody2D"
	body.gravity_scale = 0.0
	scene_root.add_child(body)
	body.owner = scene_root

	var shape := PhysicsTest2D.get_default_collision_shape(TestCollisionShape.RECTANGLE)
	body.add_child(shape)
	shape.owner = scene_root

	var visual := Sprite2D.new()
	visual.name = "Sprite2D"
	visual.scale = VISUAL_SCALE
	body.add_child(visual)
	visual.owner = scene_root

	var packed_scene := PackedScene.new()
	var error := packed_scene.pack(scene_root)
	assert(error == OK)
	scene_root.free()
	return packed_scene

func add_body_scale_result(p_monitor: GenericManualMonitor) -> void:
	var passed := false
	if spawned_body != null:
		var server_transform: Transform2D = PhysicsServer2D.body_get_state(spawned_body.get_rid(), PhysicsServer2D.BODY_STATE_TRANSFORM)
		var node_scale := spawned_body.global_scale
		var server_scale := server_transform.get_scale()
		passed = vector_near(node_scale, Vector2.ONE, SCALE_TOLERANCE) and vector_near(server_scale, Vector2.ONE, SCALE_TOLERANCE)
		if not passed:
			p_monitor.add_test_error("Expected body scale near %v, got node %v and server %v" % [Vector2.ONE, node_scale, server_scale])
	else:
		p_monitor.add_test_error("RigidBody2D was not created")
	p_monitor.add_test_result(passed)

func add_visual_scale_result(p_monitor: GenericManualMonitor) -> void:
	var passed := false
	if spawned_visual != null:
		var visual_scale := spawned_visual.global_scale
		passed = vector_near(visual_scale, VISUAL_SCALE, SCALE_TOLERANCE)
		if not passed:
			p_monitor.add_test_error("Expected Sprite2D scale near %v, got %v" % [VISUAL_SCALE, visual_scale])
	else:
		p_monitor.add_test_error("Sprite2D was not created")
	p_monitor.add_test_result(passed)

func vector_near(p_a: Vector2, p_b: Vector2, p_tolerance: float) -> bool:
	return absf(p_a.x - p_b.x) <= p_tolerance and absf(p_a.y - p_b.y) <= p_tolerance
