extends DSSTest3D

# Checks PhysicsDirectSpaceState3D.cast_motion: it returns [safe, unsafe] fractions along a
# motion. A path that hits a collider returns a safe fraction < 1; a clear path returns
# [1, 1]. Also checks collide flags and exclude.

var simulation_duration := 10.0

var _shape: RID

func test_description() -> String:
	return "Checks that cast_motion returns a safe fraction < 1 on a blocked path and [1,1] on a clear one."

func test_name() -> String:
	return "DirectSpaceState3D | cast_motion"

func _params(p_pos: Vector3, p_motion: Vector3) -> PhysicsShapeQueryParameters3D:
	var q := PhysicsShapeQueryParameters3D.new()
	q.shape_rid = _shape
	q.transform = Transform3D(Basis(), p_pos)
	q.motion = p_motion
	return q

func test_start() -> void:
	add_area(LEFT, 1)
	var body := add_static_body(RIGHT, 2)
	_shape = query_box_shape()

	var clear := func(r): return r.size() == 2 and is_equal_approx(r[0], 1.0) and is_equal_approx(r[1], 1.0)

	var checks := func(_t, m: GenericManualMonitor):
		if m.frame != 2:
			return
		var d := get_world_3d().direct_space_state

		m.add_test("Blocked path toward a body returns a safe fraction < 1")
		var q := _params(CENTER, Vector3(8, 0, 0))
		q.collide_with_bodies = true
		var r: PackedFloat32Array = d.cast_motion(q)
		if r.size() == 2 and r[0] >= 0.9:
			m.add_test_error("safe fraction was %.3f" % r[0])
		m.add_test_result(r.size() == 2 and r[0] < 0.9)

		m.add_test("Clear path (moving away) returns [1,1]")
		q = _params(CENTER, Vector3(-8, 0, 0))
		q.collide_with_bodies = true   # nothing but an area to the left; areas not queried
		m.add_test_result(clear.call(d.cast_motion(q)))

		m.add_test("No collision when bodies disabled returns [1,1]")
		q = _params(CENTER, Vector3(8, 0, 0))
		q.collide_with_bodies = false
		m.add_test_result(clear.call(d.cast_motion(q)))

		m.add_test("Excluding the body clears the path")
		q = _params(CENTER, Vector3(8, 0, 0))
		q.collide_with_bodies = true
		q.exclude = [body.get_rid()]
		m.add_test_result(clear.call(d.cast_motion(q)))

		m.add_test("Blocked path toward an area returns a safe fraction < 1")
		q = _params(CENTER, Vector3(-8, 0, 0))
		q.collide_with_areas = true
		r = d.cast_motion(q)
		m.add_test_result(r.size() == 2 and r[0] < 0.9)

		PhysicsServer3D.free_rid(_shape)
		m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
