extends DSSTest3D

# Checks the parameters of PhysicsDirectSpaceState3D.intersect_point: point inside vs
# outside a shape, collide_with_bodies / collide_with_areas, exclude by RID, collision_mask,
# and multiple-result counting with a limit.

var simulation_duration := 10.0

func test_description() -> String:
	return "Checks the PhysicsPointQueryParameters3D options of intersect_point."

func test_name() -> String:
	return "DirectSpaceState3D | intersect_point"

func test_start() -> void:
	var area := add_area(LEFT, 1)
	var body := add_static_body(RIGHT, 2)
	# Four overlapping objects at the center for multi-result counting.
	add_static_body(CENTER, 1)
	add_area(CENTER, 1)
	add_static_body(CENTER, 1)
	add_area(CENTER, 1)

	var checks := func(_t, m: GenericManualMonitor):
		if m.frame != 2:
			return
		var d := get_world_3d().direct_space_state

		m.add_test("No hit just outside the body")
		var q := PhysicsPointQueryParameters3D.new()
		q.position = RIGHT - Vector3(1.5, 0, 0)   # x = 2.5, body spans [3,5]
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Hit inside the body")
		q = PhysicsPointQueryParameters3D.new()
		q.position = RIGHT
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_point(q).size() == 1)

		m.add_test("No hit on body when bodies disabled")
		q = PhysicsPointQueryParameters3D.new()
		q.position = RIGHT
		q.collide_with_bodies = false
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Hit inside the area")
		q = PhysicsPointQueryParameters3D.new()
		q.position = LEFT
		q.collide_with_areas = true
		m.add_test_result(d.intersect_point(q).size() == 1)

		m.add_test("No hit on area when areas disabled")
		q = PhysicsPointQueryParameters3D.new()
		q.position = LEFT
		q.collide_with_areas = false
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Can exclude a body by RID")
		q = PhysicsPointQueryParameters3D.new()
		q.position = RIGHT
		q.collide_with_bodies = true
		q.exclude = [body.get_rid()]
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Can exclude an area by RID")
		q = PhysicsPointQueryParameters3D.new()
		q.position = LEFT
		q.collide_with_areas = true
		q.exclude = [area.get_rid()]
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Detects all four overlapping objects")
		q = PhysicsPointQueryParameters3D.new()
		q.position = CENTER
		q.collide_with_bodies = true
		q.collide_with_areas = true
		m.add_test_result(d.intersect_point(q, 8).size() == 4)

		m.add_test("Respects the result limit")
		q = PhysicsPointQueryParameters3D.new()
		q.position = CENTER
		q.collide_with_bodies = true
		q.collide_with_areas = true
		m.add_test_result(d.intersect_point(q, 2).size() == 2)

		m.add_test("Does NOT report a hit on the wrong collision layer")
		q = PhysicsPointQueryParameters3D.new()
		q.position = LEFT                # area on layer 1
		q.collide_with_areas = true
		q.collision_mask = 1 << 1        # only layer 2
		m.add_test_result(d.intersect_point(q).is_empty())

		m.add_test("Reports a hit on the correct collision layer")
		q = PhysicsPointQueryParameters3D.new()
		q.position = RIGHT               # body on layer 2
		q.collide_with_bodies = true
		q.collision_mask = 1 << 1        # only layer 2
		m.add_test_result(d.intersect_point(q).size() == 1)

		m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
