extends DSSTest3D

# Checks the parameters of PhysicsDirectSpaceState3D.intersect_ray: collide_with_bodies /
# collide_with_areas, hit_from_inside, exclude by RID, and collision_mask.

var simulation_duration := 10.0

func test_description() -> String:
	return "Checks the PhysicsRayQueryParameters3D options of intersect_ray."

func test_name() -> String:
	return "DirectSpaceState3D | intersect_ray"

func test_start() -> void:
	var area := add_area(LEFT, 1)
	var body := add_static_body(RIGHT, 2)

	var checks := func(_t, m: GenericManualMonitor):
		if m.frame != 2:
			return
		var d := get_world_3d().direct_space_state

		m.add_test("Can collide with body")
		var q := PhysicsRayQueryParameters3D.create(CENTER, RIGHT)
		q.collide_with_bodies = true
		m.add_test_result(d.intersect_ray(q).size() > 0)

		m.add_test("Can NOT collide with body when bodies disabled")
		q = PhysicsRayQueryParameters3D.create(CENTER, RIGHT)
		q.collide_with_bodies = false
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Can collide with area")
		q = PhysicsRayQueryParameters3D.create(CENTER, LEFT)
		q.collide_with_areas = true
		m.add_test_result(d.intersect_ray(q).size() > 0)

		m.add_test("Can NOT collide with area when areas disabled")
		q = PhysicsRayQueryParameters3D.create(CENTER, LEFT)
		q.collide_with_areas = false
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Detects hit from inside when enabled")
		q = PhysicsRayQueryParameters3D.create(RIGHT, CENTER)
		q.collide_with_bodies = true
		q.hit_from_inside = true
		m.add_test_result(d.intersect_ray(q).size() > 0)

		m.add_test("Does NOT detect hit from inside when disabled")
		q = PhysicsRayQueryParameters3D.create(RIGHT, CENTER)
		q.collide_with_bodies = true
		q.hit_from_inside = false
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Can exclude a body by RID")
		q = PhysicsRayQueryParameters3D.create(CENTER, RIGHT)
		q.collide_with_bodies = true
		q.exclude = [body.get_rid()]
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Can exclude an area by RID")
		q = PhysicsRayQueryParameters3D.create(CENTER, LEFT)
		q.collide_with_areas = true
		q.exclude = [area.get_rid()]
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Does NOT report a hit on the wrong collision layer")
		q = PhysicsRayQueryParameters3D.create(CENTER, LEFT)   # area is on layer 1
		q.collide_with_areas = true
		q.collision_mask = 1 << 1                              # only layer 2
		m.add_test_result(d.intersect_ray(q).is_empty())

		m.add_test("Reports a hit on the correct collision layer")
		q = PhysicsRayQueryParameters3D.create(CENTER, RIGHT)  # body is on layer 2
		q.collide_with_bodies = true
		q.collision_mask = 1 << 1                              # only layer 2
		m.add_test_result(d.intersect_ray(q).size() > 0)

		m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
