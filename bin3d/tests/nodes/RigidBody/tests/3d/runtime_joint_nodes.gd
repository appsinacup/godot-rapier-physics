extends PhysicsUnitTest3D

const SWITCH_FRAME := 30
const CHECK_FRAME := 90
# Free-falling for a second covers far more than this, and a held body far less.
const FALL_DISTANCE := 2.0

var simulation_duration := 3.0
var cases := []

func test_description() -> String:
	return """Checks that changing node_b at runtime moves the joint to the new body."""

func test_name() -> String:
	return "RapierFixedJoint3D, RapierRopeJoint3D | changing the bodies at runtime"

func create_body(holder: Node3D, body: PhysicsBody3D, body_name: String, at: Vector3) -> PhysicsBody3D:
	body.name = body_name
	body.position = at
	body.collision_layer = 0
	body.collision_mask = 0
	var shape := CollisionShape3D.new()
	var sphere := SphereShape3D.new()
	sphere.radius = 0.1
	shape.shape = sphere
	body.add_child(shape)
	holder.add_child(body)
	return body

func add_case(label: String, joint: Node3D, offset: Vector3) -> void:
	var holder := Node3D.new()
	holder.position = offset
	add_child(holder)
	create_body(holder, StaticBody3D.new(), "anchor", Vector3.ZERO)
	var first := create_body(holder, RigidBody3D.new(), "first", Vector3(0, -0.5, 0))
	var second := create_body(holder, RigidBody3D.new(), "second", Vector3(0.5, -0.5, 0))
	joint.position = Vector3(0, -0.25, 0)
	joint.set("node_a", ^"../anchor")
	joint.set("node_b", ^"../first")
	holder.add_child(joint)
	cases.append({"label": label, "joint": joint, "first": first, "second": second})

func test_start() -> void:
	add_case("Fixed joint", RapierFixedJoint3D.new(), Vector3(0, 0, 0))
	add_case("Rope joint", RapierRopeJoint3D.new(), Vector3(5, 0, 0))

	var run = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == SWITCH_FRAME:
			for entry in cases:
				entry.first_at_switch = entry.first.global_position.y
				entry.second_at_switch = entry.second.global_position.y
				entry.joint.set("node_b", ^"../second")
		if p_monitor.frame == CHECK_FRAME:
			for entry in cases:
				p_monitor.add_test("%s releases the old body" % entry.label)
				p_monitor.add_test_result(entry.first_at_switch - entry.first.global_position.y > FALL_DISTANCE)
				p_monitor.add_test("%s holds the new body" % entry.label)
				p_monitor.add_test_result(entry.second_at_switch - entry.second.global_position.y < FALL_DISTANCE)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
