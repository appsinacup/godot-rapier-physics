extends PhysicsUnitTest3D

var simulation_duration := 10

# space_update_queries() must not change the simulation (#649). The same ticks run with and
# without it, and the whole physics state is compared. The bodies can end at the same place from
# different states.

const TICKS := 300
# Static bodies that never move. With few changes against a large tree, rapier re-inserts the
# changed leaves one by one. The tree then depends on the order of the changes.
const STATIC_COUNT := 400

var space: RID
var box: RID
var plank: RID
var ground: RID
var fixed_bodies: Array[RID] = []
var platforms: Array[RID] = []
var boxes: Array[RID] = []
var area: RID

func test_description() -> String:
	return """Checks that the physics state after many ticks is the same with and without
	space_update_queries(), with changes before and after it in each tick.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | space_update_queries doesn't change the simulation (#649)"

func test_start() -> void:
	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		p_monitor.add_test("The physics state doesn't depend on space_update_queries, with changes and removals after it")
		p_monitor.add_test_result(simulate(true) == simulate(false))
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

# Returns the physics state after TICKS ticks.
func simulate(p_update_queries: bool) -> Dictionary:
	build_scene()
	for t in TICKS:
		change_before_update(t)
		if p_update_queries:
			RapierPhysicsServer3D.space_update_queries(space)
		change_after_update(t)
		RapierPhysicsServer3D.space_step(space, 1.0 / 60)
		RapierPhysicsServer3D.space_flush_queries(space)
	var state := export_world()
	free_scene()
	return state

# Boxes falling on moving platforms, in their own space stepped by hand.
func build_scene() -> void:
	space = PhysicsServer3D.space_create()
	PhysicsServer3D.space_set_active(space, false)
	box = add_shape(Vector3(0.4, 0.4, 0.4))
	plank = add_shape(Vector3(3, 0.25, 2))
	ground = add_shape(Vector3(50, 0.5, 10))
	fixed_bodies.clear()
	platforms.clear()
	boxes.clear()
	fixed_bodies.append(add_body(PhysicsServer3D.BODY_MODE_STATIC, ground, Transform3D(Basis(), Vector3(0, -15, 0))))
	for i in 5:
		platforms.append(add_body(PhysicsServer3D.BODY_MODE_STATIC, plank, Transform3D()))
	area = PhysicsServer3D.area_create()
	PhysicsServer3D.area_add_shape(area, plank)
	PhysicsServer3D.area_set_space(area, space)
	for i in STATIC_COUNT:
		fixed_bodies.append(add_body(PhysicsServer3D.BODY_MODE_STATIC, box, Transform3D(Basis(), Vector3(-20 + (i % 40), -20 - (i / 40), 0))))
	for i in 30:
		boxes.append(add_body(PhysicsServer3D.BODY_MODE_RIGID, box, Transform3D(Basis(Vector3.BACK, 0.1 * i), Vector3(-7.5 + (i % 10) * 1.5, -7.5 + (i / 10) * 1.5, 0))))

# Platforms move, and sometimes a box is teleported.
func change_before_update(p_tick: int) -> void:
	for i in 4:
		PhysicsServer3D.body_set_state(platforms[i], PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(Vector3.BACK, 0.01 * p_tick), Vector3(-12.5 + ((p_tick * 7 + i * 131) % 500) * 0.05, -12.5 + 0.75 * i, 0)))
	if p_tick % 37 == 0:
		PhysicsServer3D.body_set_state(boxes[p_tick % boxes.size()], PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(), Vector3(-5 + (p_tick % 200) * 0.05, -7.5, 0)))

# The area moves, a platform's shape moves, and a box is freed or added.
func change_after_update(p_tick: int) -> void:
	PhysicsServer3D.area_set_transform(area, Transform3D(Basis(), Vector3(-10 + ((p_tick * 5) % 400) * 0.05, -12.5, 0)))
	PhysicsServer3D.body_set_shape_transform(platforms[4], 0, Transform3D(Basis(), Vector3(10 - ((p_tick * 3) % 400) * 0.05, -13.5, 0)))
	if p_tick % 20 == 10:
		PhysicsServer3D.free_rid(boxes.pop_front())
	if p_tick % 20 == 15:
		boxes.append(add_body(PhysicsServer3D.BODY_MODE_RIGID, box, Transform3D(Basis(), Vector3(-5 + p_tick * 0.05, -5, 0))))

func add_shape(p_size: Vector3) -> RID:
	var shape := PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(shape, p_size)
	return shape

func add_body(p_mode: int, p_shape: RID, p_transform: Transform3D) -> RID:
	var body := PhysicsServer3D.body_create()
	PhysicsServer3D.body_set_mode(body, p_mode)
	PhysicsServer3D.body_add_shape(body, p_shape)
	PhysicsServer3D.body_set_space(body, space)
	PhysicsServer3D.body_set_state(body, PhysicsServer3D.BODY_STATE_TRANSFORM, p_transform)
	return body

# The space's physics world, without the ids that differ between the two runs.
func export_world() -> Dictionary:
	var holder := Node.new()
	add_child(holder)
	var state_manager: Node = ClassDB.instantiate("StateManager3D")
	state_manager.root_node = holder
	add_child(state_manager)
	var world: Dictionary = JSON.parse_string(state_manager.export_state(space, "Json"))["rapier_space"]["world"]
	state_manager.free()
	holder.free()
	world.erase("handle")
	return without_user_data(world)

func without_user_data(p_value: Variant) -> Variant:
	if p_value is Dictionary:
		var result := {}
		for key in p_value:
			if key != "user_data":
				result[key] = without_user_data(p_value[key])
		return result
	if p_value is Array:
		return p_value.map(without_user_data)
	return p_value

func free_scene() -> void:
	for rid in boxes + platforms + fixed_bodies + [area, space, box, plank, ground]:
		PhysicsServer3D.free_rid(rid)
