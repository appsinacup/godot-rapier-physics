extends DSSTest3D

# Checks the parameters of PhysicsDirectSpaceState3D.intersect_shape: overlap with a body /
# area, collide flags, exclude by RID, collision_mask, and multiple-result counting/limit.

var simulation_duration := 10.0

const MULTI_A := Vector3(0, 3.5, 0)
const MULTI_B := Vector3(0, 4.5, 0)

var _shape: RID

func test_description() -> String:
	return "Checks the PhysicsShapeQueryParameters3D options of intersect_shape."

func test_name() -> String:
	return "DirectSpaceState3D | intersect_shape"

func _params(p_pos: Vector3) -> PhysicsShapeQueryParameters3D:
	var q := PhysicsShapeQueryParameters3D.new()
	q.shape_rid = _shape
	q.transform = Transform3D(Basis(), p_pos)
	return q

func test_start() -> void:
	var area := add_area(LEFT, 1)
	var body := add_static_body(RIGHT, 2)
	add_static_body(MULTI_A, 1)
	add_static_body(MULTI_B, 1)
	_shape = query_box_shape()

	var checks := func(_t, m: GenericManualMonitor):
		if m.frame != 2:
			return
		var d := get_world_3d().direct_space_state

		m.add_test("Overlaps a body")
		var q := _params(RIGHT)
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_shape(q).size() > 0)

		m.add_test("No body overlap when bodies disabled")
		q = _params(RIGHT)
		q.collide_with_bodies = false
		m.add_test_result(d.intersect_shape(q).is_empty())

		m.add_test("Overlaps an area")
		q = _params(LEFT)
		q.collide_with_areas = true
		m.add_test_result(d.intersect_shape(q).size() > 0)

		m.add_test("No area overlap when areas disabled")
		q = _params(LEFT)
		q.collide_with_areas = false
		m.add_test_result(d.intersect_shape(q).is_empty())

		m.add_test("Can exclude a body by RID")
		q = _params(RIGHT)
		q.collide_with_bodies = true
		q.exclude = [body.get_rid()]
		m.add_test_result(d.intersect_shape(q).is_empty())

		m.add_test("Can exclude an area by RID")
		q = _params(LEFT)
		q.collide_with_areas = true
		q.exclude = [area.get_rid()]
		m.add_test_result(d.intersect_shape(q).is_empty())

		m.add_test("Detects both overlapping bodies")
		q = _params((MULTI_A + MULTI_B) * 0.5)
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_shape(q, 8).size() == 2)

		m.add_test("Respects the result limit")
		q = _params((MULTI_A + MULTI_B) * 0.5)
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_shape(q, 1).size() == 1)

		m.add_test("Does NOT overlap on the wrong collision layer")
		q = _params(RIGHT)                 # body on layer 2
		q.collide_with_bodies = true
		q.collision_mask = 1               # only layer 1
		m.add_test_result(d.intersect_shape(q).is_empty())

		m.add_test("Overlaps on the correct collision layer")
		q = _params(RIGHT)
		q.collide_with_bodies = true
		q.collision_mask = 1 << 1          # only layer 2
		m.add_test_result(d.intersect_shape(q).size() > 0)

		PhysicsServer3D.free_rid(_shape)
		m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
