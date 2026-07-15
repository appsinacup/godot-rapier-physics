extends DSSTest3D

# Checks PhysicsDirectSpaceState3D.collide_shape: when the query shape overlaps a collider
# it returns contact points; when it does not overlap (disabled / excluded) it returns none.

var simulation_duration := 10.0

var _shape: RID

func test_description() -> String:
	return "Checks that collide_shape returns contact points on overlap and nothing otherwise."

func test_name() -> String:
	return "DirectSpaceState3D | collide_shape"

func _params(p_pos: Vector3) -> PhysicsShapeQueryParameters3D:
	var q := PhysicsShapeQueryParameters3D.new()
	q.shape_rid = _shape
	q.transform = Transform3D(Basis(), p_pos)
	return q

func test_start() -> void:
	var area := add_area(LEFT, 1)
	var body := add_static_body(RIGHT, 2)
	_shape = query_box_shape()

	var checks := func(_t, m: GenericManualMonitor):
		if m.frame != 2:
			return
		var d := get_world_3d().direct_space_state

		m.add_test("Returns contact points overlapping a body")
		var q := _params(RIGHT)
		q.collide_with_bodies = true
		m.add_test_result(d.collide_shape(q).size() > 0)

		m.add_test("No contacts when far from any collider")
		q = _params(Vector3(0, 30, 0))
		q.collide_with_bodies = true
		q.collide_with_areas = true
		m.add_test_result(d.collide_shape(q).is_empty())

		m.add_test("No body contacts when bodies disabled")
		q = _params(RIGHT)
		q.collide_with_bodies = false
		m.add_test_result(d.collide_shape(q).is_empty())

		m.add_test("Returns contact points overlapping an area")
		q = _params(LEFT)
		q.collide_with_areas = true
		m.add_test_result(d.collide_shape(q).size() > 0)

		m.add_test("Can exclude a body by RID")
		q = _params(RIGHT)
		q.collide_with_bodies = true
		q.exclude = [body.get_rid()]
		m.add_test_result(d.collide_shape(q).is_empty())

		m.add_test("Can exclude an area by RID")
		q = _params(LEFT)
		q.collide_with_areas = true
		q.exclude = [area.get_rid()]
		m.add_test_result(d.collide_shape(q).is_empty())

		PhysicsServer3D.free_rid(_shape)
		m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
