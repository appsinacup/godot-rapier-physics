extends PhysicsUnitTest2D

var simulation_duration := 10

# A static body whose shapes live in one compound collider still has to answer queries with the
# index of the shape that was actually hit, not the collider's first shape.

var ground: StaticBody2D

func test_description() -> String:
	return """Checks that space queries against a decomposed [CollisionPolygon2D] report the shape
	that was hit rather than the compound collider's first shape.
	"""

func test_name() -> String:
	return "DirectSpaceState2D | shape indices against a compound collider"

func test_start() -> void:
	ground = StaticBody2D.new()
	var poly := CollisionPolygon2D.new()
	# two towers and a valley: distinct convex pieces at distinct x ranges
	poly.polygon = PackedVector2Array([
		Vector2(0, 507), Vector2(0, 0), Vector2(64, 0), Vector2(64, 507),
		Vector2(502, 155), Vector2(502, 0), Vector2(634, 0), Vector2(634, 155),
		Vector2(1072, 507), Vector2(1072, 0), Vector2(1152, 0), Vector2(1152, 648),
		Vector2(0, 648)])
	ground.add_child(poly)
	# point queries honor pickable by default (physics/rapier/queries/point_query_honors_pickable)
	ground.input_pickable = true
	add_child(ground)

	var probes := [Vector2(30, 300), Vector2(200, 600), Vector2(560, 100), Vector2(1120, 300)]

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var space := get_world_2d().direct_space_state
		var circle := CircleShape2D.new()
		circle.radius = 10.0

		if true:
			p_monitor.add_test("Rays report the piece they hit")
			var seen := {}
			for x in [30.0, 200.0, 560.0, 900.0, 1120.0]:
				var q := PhysicsRayQueryParameters2D.create(Vector2(x, -50), Vector2(x, 700))
				var hit := space.intersect_ray(q)
				if hit:
					seen[hit.shape] = true
			p_monitor.add_test_result(seen.size() >= 4)

		if true:
			p_monitor.add_test("Point queries report the piece containing the point")
			var seen := {}
			for p in probes:
				var q := PhysicsPointQueryParameters2D.new()
				q.position = p
				for hit in space.intersect_point(q):
					seen[hit.shape] = true
			if seen.is_empty():
				p_monitor.add_test_error("no point hits at all")
			p_monitor.add_test_result(seen.size() >= 4)

		if true:
			p_monitor.add_test("Shape queries report the piece they overlap")
			var seen := {}
			for p in probes:
				var q := PhysicsShapeQueryParameters2D.new()
				q.shape = circle
				q.transform = Transform2D(0.0, p)
				for hit in space.intersect_shape(q):
					seen[hit.shape] = true
			p_monitor.add_test_result(seen.size() >= 4)

		if true:
			p_monitor.add_test("Rest info reports the piece it rests on")
			var seen := {}
			for p in probes:
				var q := PhysicsShapeQueryParameters2D.new()
				q.shape = circle
				q.transform = Transform2D(0.0, p)
				var info := space.get_rest_info(q)
				if info:
					seen[info.shape] = true
			p_monitor.add_test_result(seen.size() >= 4)

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
