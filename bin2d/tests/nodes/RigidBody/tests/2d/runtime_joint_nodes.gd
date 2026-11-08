extends PhysicsUnitTest2D

const SWITCH_FRAME := 30
const CHECK_FRAME := 90
# Free-falling for a second covers far more than this, and a held body far less.
const FALL_DISTANCE := 200.0

var simulation_duration := 3.0
var cases := []

func test_description() -> String:
	return """Checks that changing node_b at runtime moves the joint to the new body."""

func test_name() -> String:
	return "RapierFixedJoint2D, RapierRopeJoint2D | changing the bodies at runtime"

func create_body(holder: Node2D, body: PhysicsBody2D, body_name: String, at: Vector2) -> PhysicsBody2D:
	body.name = body_name
	body.position = at
	body.collision_layer = 0
	body.collision_mask = 0
	var shape := CollisionShape2D.new()
	var circle := CircleShape2D.new()
	circle.radius = 5.0
	shape.shape = circle
	body.add_child(shape)
	holder.add_child(body)
	return body

func add_case(label: String, joint: Node2D, offset: Vector2) -> void:
	var holder := Node2D.new()
	holder.position = offset
	add_child(holder)
	create_body(holder, StaticBody2D.new(), "anchor", Vector2.ZERO)
	var first := create_body(holder, RigidBody2D.new(), "first", Vector2(0, 20))
	var second := create_body(holder, RigidBody2D.new(), "second", Vector2(30, 20))
	joint.position = Vector2(0, 20)
	joint.set("node_a", ^"../anchor")
	joint.set("node_b", ^"../first")
	holder.add_child(joint)
	cases.append({"label": label, "joint": joint, "first": first, "second": second})

func test_start() -> void:
	add_case("Fixed joint", RapierFixedJoint2D.new(), Vector2(0, 0))
	add_case("Rope joint", RapierRopeJoint2D.new(), Vector2(400, 0))

	var run = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == SWITCH_FRAME:
			for entry in cases:
				entry.first_at_switch = entry.first.global_position.y
				entry.second_at_switch = entry.second.global_position.y
				entry.joint.set("node_b", ^"../second")
		if p_monitor.frame == CHECK_FRAME:
			for entry in cases:
				p_monitor.add_test("%s releases the old body" % entry.label)
				p_monitor.add_test_result(entry.first.global_position.y - entry.first_at_switch > FALL_DISTANCE)
				p_monitor.add_test("%s holds the new body" % entry.label)
				p_monitor.add_test_result(entry.second.global_position.y - entry.second_at_switch < FALL_DISTANCE)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
