extends PhysicsUnitTest3D

var simulation_duration := 10

const SEPARATION := 2.5
const GROWN := Vector3(6, 6, 6)

var grown_area: Area3D
var shrunk_area: Area3D
var reparented_scale_area: Area3D
var mirrored_area: Area3D
var mirrored_target: Area3D

func test_description() -> String:
	return """Checks that an [Area3D] whose scale changes at runtime rescales its colliders,
	so overlaps start and stop where the scaled shapes actually are.
	"""

func test_name() -> String:
	return "Area3D | scale changed at runtime updates the collision shapes"

func test_start() -> void:
	# Grows to reach a neighbour that is out of range at scale 1.
	grown_area = add_area(Vector3.ZERO)
	add_area(Vector3(SEPARATION, 0, 0))

	# Starts overlapping a neighbour and shrinks away from it.
	shrunk_area = add_area(Vector3(0, 0, 20))
	shrunk_area.scale = GROWN
	add_area(Vector3(SEPARATION, 0, 20))

	# The scale comes from a parent rather than the area itself.
	var parent := Node3D.new()
	parent.position = Vector3(0, 0, 40)
	add_child(parent)
	reparented_scale_area = add_area(Vector3.ZERO, parent)
	add_area(Vector3(SEPARATION, 0, 40))

	# A negative scale mirrors the shape offset onto the other side, the way a character
	# flipped to face the other direction moves its hitboxes across.
	var mirror_root := Node3D.new()
	mirror_root.position = Vector3(0, 0, 60)
	add_child(mirror_root)
	mirrored_area = add_area(Vector3.ZERO, mirror_root, Vector3(1, 0, 0))
	mirrored_target = add_area(Vector3(-1, 0, 60))

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 2:
			grown_area.scale = GROWN
			shrunk_area.scale = Vector3.ONE
			parent.scale = GROWN
			mirror_root.scale = Vector3(-1, 1, 1)

		if p_monitor.frame == 4:
			if true:
				p_monitor.add_test("Detect area reached by growing")
				p_monitor.add_test_result(grown_area.get_overlapping_areas().size() == 1)

			if true:
				p_monitor.add_test("Don't detect area left behind by shrinking")
				p_monitor.add_test_result(shrunk_area.get_overlapping_areas().is_empty())

			if true:
				p_monitor.add_test("Detect area reached by a scaled parent")
				p_monitor.add_test_result(reparented_scale_area.get_overlapping_areas().size() == 1)

			if true:
				p_monitor.add_test("Detect area reached by a mirrored shape offset")
				p_monitor.add_test_result(mirrored_area.overlaps_area(mirrored_target))

			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func add_area(p_position: Vector3, p_parent: Node = null, p_shape_offset := Vector3.ZERO) -> Area3D:
	var area := Area3D.new()
	var shape: CollisionShape3D = PhysicsTest3D.get_default_collision_shape(PhysicsTest3D.TestCollisionShape.BOX)
	shape.position = p_shape_offset
	area.add_child(shape)
	area.position = p_position
	if p_parent:
		p_parent.add_child(area)
	else:
		add_child(area)
	return area
