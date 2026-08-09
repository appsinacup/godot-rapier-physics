extends PhysicsUnitTest2D

const BODY_RADIUS := 60.0
const QUERY_RADIUS := 10.0
const TRAVEL := 400.0

var simulation_duration := 10

func test_description() -> String:
	return """Checks the [cast_motion] contract: it returns [safe, unsafe] where unsafe comes
	strictly after safe, and the shape really does collide once advanced by unsafe. A grazing
	cast is included because there the gap closes far slower than the motion travels.
	"""

func test_name() -> String:
	return "DirectSpaceState2D | testing [cast_motion] safe and unsafe fractions"

func test_start() -> void:
	add_child(build_body(CENTER))

	var d_space := get_world_2d().direct_space_state
	var shape_rid := PhysicsServer2D.circle_shape_create()
	PhysicsServer2D.shape_set_data(shape_rid, QUERY_RADIUS)

	# Head-on aims at the centre; the grazing cast is offset so it only clips the rim.
	var head_on_offset := 0.0
	var grazing_offset := BODY_RADIUS + QUERY_RADIUS - 0.05

	var checks := func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return

		for probe in [["Head-on", head_on_offset], ["Grazing", grazing_offset]]:
			var label: String = probe[0]
			var offset: float = probe[1]
			var start := CENTER + Vector2(-TRAVEL, offset)
			var motion := Vector2(TRAVEL, 0)
			var fractions := cast_motion_at(d_space, shape_rid, start, motion)

			p_monitor.add_test("%s cast reports a collision" % label)
			p_monitor.add_test_result(fractions.size() == 2 and fractions[0] < 1.0)

			p_monitor.add_test("%s unsafe comes after safe" % label)
			p_monitor.add_test_result(fractions[0] < fractions[1] and fractions[1] <= 1.0)

			p_monitor.add_test("%s shape actually collides at unsafe" % label)
			p_monitor.add_test_result(
				collides_at(d_space, shape_rid, start + motion * fractions[1])
			)

		PhysicsServer2D.free_rid(shape_rid)
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func cast_motion_at(
	p_space: PhysicsDirectSpaceState2D, p_shape_rid: RID, p_from: Vector2, p_motion: Vector2
) -> PackedFloat32Array:
	var query := PhysicsShapeQueryParameters2D.new()
	query.shape_rid = p_shape_rid
	query.transform = Transform2D(0, p_from)
	query.motion = p_motion
	query.collide_with_bodies = true
	return p_space.cast_motion(query)

func collides_at(
	p_space: PhysicsDirectSpaceState2D, p_shape_rid: RID, p_position: Vector2
) -> bool:
	var query := PhysicsShapeQueryParameters2D.new()
	query.shape_rid = p_shape_rid
	query.transform = Transform2D(0, p_position)
	query.collide_with_bodies = true
	return not p_space.collide_shape(query).is_empty()

func build_body(p_position: Vector2) -> StaticBody2D:
	var body := StaticBody2D.new()
	body.position = p_position
	var col := CollisionShape2D.new()
	var shape := CircleShape2D.new()
	shape.radius = BODY_RADIUS
	col.shape = shape
	body.add_child(col)
	return body
