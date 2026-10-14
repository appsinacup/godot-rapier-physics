extends PhysicsUnitTest3D

var simulation_duration := 10

# Rapier gives a freed collider's index to the next collider. The freed one stays in the broad
# phase until the step. space_update_queries() must find the new collider and not the freed one
# (#649).

const PROBE_AT := Vector3(200, 0, 0)
const FAR := Vector3(0, 500, 0)

var viewport: SubViewport
var space: RID
var state_manager: Node

func test_description() -> String:
	return """Checks that space_update_queries() lets space queries see a body whose collider
	reuses the index of a body freed since the last step, and no longer see the freed body.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | space_update_queries with a reused collider index (#649)"

func test_start() -> void:
	# Node bodies in their own world, stepped by hand. A state export gives their collider indices.
	viewport = SubViewport.new()
	viewport.world_3d = World3D.new()
	add_child(viewport)
	space = viewport.world_3d.space
	PhysicsServer3D.space_set_active(space, false)
	state_manager = ClassDB.instantiate("StateManager3D")
	state_manager.root_node = viewport
	add_child(state_manager)

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var freed := add_body("freed", FAR)
		step()
		var freed_index := collider_index("freed")
		freed.free()

		add_body("reusing", PROBE_AT)
		RapierPhysicsServer3D.space_update_queries(space)
		p_monitor.add_test("A body reusing a freed body's index is found, the freed one isn't")
		p_monitor.add_test_result(collider_index("reusing") == freed_index and probe(PROBE_AT) == 1 and probe(FAR) == 0)

		step()
		p_monitor.add_test("A body reusing a freed body's index is still found after the step")
		p_monitor.add_test_result(probe(PROBE_AT) == 1 and probe(FAR) == 0)

		viewport.queue_free()
		state_manager.queue_free()
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

func add_body(p_name: String, p_position: Vector3) -> StaticBody3D:
	var collision_shape := CollisionShape3D.new()
	collision_shape.shape = BoxShape3D.new()
	collision_shape.shape.size = Vector3(20, 20, 20)
	var body := StaticBody3D.new()
	body.name = p_name
	body.position = p_position
	body.add_child(collision_shape)
	viewport.add_child(body)
	return body

func collider_index(p_name: String) -> int:
	var exported = JSON.parse_string(state_manager.export_state(space, "Json"))
	return exported["physics_objects_state"][p_name]["state"]["Body"]["base_state"]["shapes"][0]["collider_handle"]["index"]

func step() -> void:
	RapierPhysicsServer3D.space_step(space, 1.0 / 60)
	RapierPhysicsServer3D.space_flush_queries(space)

# The number of bodies at p_position.
func probe(p_position: Vector3) -> int:
	var sphere := SphereShape3D.new()
	sphere.radius = 5
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = sphere
	query.transform = Transform3D(Basis(), p_position)
	return PhysicsServer3D.space_get_direct_state(space).intersect_shape(query).size()
