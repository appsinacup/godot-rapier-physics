extends PhysicsUnitTest3D

var simulation_duration := 10

var forward_area: Area3D
var reverse_area: Area3D
var scanning_area: Area3D
var scanned_body: RigidBody3D
var retuned_area: Area3D
var retuned_other: Area3D

func test_description() -> String:
	return """Checks that an [Area3D] only reports what its own collision mask covers, so a pair
	whose masks point one way is reported by one side of the pair and not the other.
	"""

func test_name() -> String:
	return "Area3D | monitoring follows the mask direction"

func test_start() -> void:
	# Only forward_area scans the layer its neighbour is on.
	forward_area = add_area(Vector3.ZERO, 1, 2)
	reverse_area = add_area(Vector3(0.5, 0, 0), 2, 4)

	# The same rule applies to a body: the body's mask covers the area's layer, but the area
	# does not scan the body's layer, so the area must not pick it up.
	scanning_area = add_area(Vector3(0, 0, 20), 1, 4)
	scanned_body = add_body(Vector3(0.5, 0, 20), 2, 1)

	# A mask that stops matching while the overlap is live has to release the overlap.
	retuned_area = add_area(Vector3(0, 0, 40), 1, 2)
	retuned_other = add_area(Vector3(0.5, 0, 40), 2, 1)

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			if true:
				p_monitor.add_test("Detect the area covered by our own mask")
				p_monitor.add_test_result(forward_area.overlaps_area(reverse_area))

			if true:
				p_monitor.add_test("Don't detect the area our mask doesn't cover")
				p_monitor.add_test_result(not reverse_area.overlaps_area(forward_area))

			if true:
				p_monitor.add_test("Don't detect the body our mask doesn't cover")
				p_monitor.add_test_result(not scanning_area.overlaps_body(scanned_body))

			scanning_area.collision_mask = 2
			retuned_area.collision_mask = 8

		if p_monitor.frame == 4:
			if true:
				p_monitor.add_test("Detect the body covered after the mask is widened")
				p_monitor.add_test_result(scanning_area.overlaps_body(scanned_body))

			if true:
				p_monitor.add_test("Release the area once the mask stops covering it")
				p_monitor.add_test_result(retuned_area.get_overlapping_areas().is_empty())

			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func add_area(p_position: Vector3, p_layer: int, p_mask: int) -> Area3D:
	var area := Area3D.new()
	area.add_child(PhysicsTest3D.get_default_collision_shape(PhysicsTest3D.TestCollisionShape.BOX))
	area.position = p_position
	area.collision_layer = p_layer
	area.collision_mask = p_mask
	add_child(area)
	return area

func add_body(p_position: Vector3, p_layer: int, p_mask: int) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.add_child(PhysicsTest3D.get_default_collision_shape(PhysicsTest3D.TestCollisionShape.SPHERE))
	body.position = p_position
	body.collision_layer = p_layer
	body.collision_mask = p_mask
	body.freeze = true
	body.freeze_mode = RigidBody3D.FREEZE_MODE_KINEMATIC
	add_child(body)
	return body
