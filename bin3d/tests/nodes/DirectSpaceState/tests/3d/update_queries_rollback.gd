extends PhysicsUnitTest3D

var simulation_duration := 10

# Rollback re-simulates from a snapshot. space_update_queries() must not leave anything outside the
# snapshot that changes the re-simulated ticks (#649).

const TICKS := 120
const SAVE_BEFORE_UPDATE := 40
const SAVE_AFTER_UPDATE := 45
const SAVE_AT_TICK_END := 50
# The phases of a tick, in order.
const MOVE_BEFORE := 0
const UPDATE_AND_QUERY := 1
const MOVE_AFTER_AND_STEP := 2
const TICK_END := 3

var viewport: SubViewport
var space: RID
var state_manager: Node
var platform: StaticBody3D
var area: Area3D
var boxes: Array[RID] = []
var probe := BoxShape3D.new()

func test_description() -> String:
	return """Checks that ticks using space_update_queries() re-simulate identically from a
	StateManager3D snapshot. The snapshot can be taken at any point of a tick, even after changes
	made after the update.
	"""

func test_name() -> String:
	return "DirectSpaceState3D | space_update_queries is replayed identically from a snapshot (#649)"

func test_start() -> void:
	# Its own world. Stepping it by hand leaves the other tests alone.
	viewport = SubViewport.new()
	viewport.world_3d = World3D.new()
	add_child(viewport)
	space = viewport.world_3d.space
	PhysicsServer3D.space_set_active(space, false)
	var world := Node3D.new()
	viewport.add_child(world)
	var floor_body := StaticBody3D.new()
	floor_body.add_child(make_shape(Vector3(100, 1, 10)))
	floor_body.position = Vector3(0, -16, 0)
	world.add_child(floor_body)
	platform = StaticBody3D.new()
	platform.add_child(make_shape(Vector3(6, 0.5, 3)))
	world.add_child(platform)
	area = Area3D.new()
	area.add_child(make_shape(Vector3(2, 2, 2)))
	world.add_child(area)
	for i in 12:
		var box := RigidBody3D.new()
		box.add_child(make_shape(Vector3(0.8, 0.8, 0.8)))
		box.position = Vector3(-6 + i * 1.1, -5 + (i % 3) * 1.5, 0)
		box.rotation = Vector3(0, 0, 0.1 * i)
		world.add_child(box)
		boxes.append(box.get_rid())
	state_manager = ClassDB.instantiate("StateManager3D")
	state_manager.root_node = world
	add_child(state_manager)
	probe.size = Vector3(3, 3, 3)

	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 2:
			return
		var hits_original: Array[int] = []
		var exported_json: Variant
		var exported_binary: Variant
		for t in TICKS:
			move_before(t)
			if t == SAVE_BEFORE_UPDATE:
				state_manager.cache_state(space, t)
			hits_original.append(update_and_query(t))
			if t == SAVE_AFTER_UPDATE:
				state_manager.cache_state(space, t)
				exported_json = state_manager.export_state(space, "Json")
				exported_binary = state_manager.export_state(space, "RustBincode")
			move_after_and_step(t)
			if t == SAVE_AT_TICK_END:
				state_manager.cache_state(space, t)
		var bodies_original = RapierPhysicsServer3D.space_get_bodies_transform(space, boxes)

		# How to load the snapshot, its tick, and the first phase of that tick left to run.
		var snapshots := [
			[func(): state_manager.load_cached_state(space, 0), SAVE_BEFORE_UPDATE, UPDATE_AND_QUERY, "a snapshot taken between moves and the update"],
			[func(): state_manager.load_cached_state(space, 1), SAVE_AFTER_UPDATE, MOVE_AFTER_AND_STEP, "a snapshot taken between the update and the step"],
			[func(): state_manager.import_state(space, exported_json), SAVE_AFTER_UPDATE, MOVE_AFTER_AND_STEP, "a JSON export taken between the update and the step"],
			[func(): state_manager.import_state(space, exported_binary), SAVE_AFTER_UPDATE, MOVE_AFTER_AND_STEP, "a binary export taken between the update and the step"],
			[func(): state_manager.load_cached_state(space, 2), SAVE_AT_TICK_END, TICK_END, "a snapshot taken after a step"],
		]
		for snapshot in snapshots:
			snapshot[0].call()
			var tick: int = snapshot[1]
			var first_phase: int = snapshot[2]
			var hits_replayed := hits_original.slice(0, tick + (1 if first_phase > UPDATE_AND_QUERY else 0))
			hits_replayed.append_array(replay(tick, first_phase))
			p_monitor.add_test("Queries see the same objects when re-simulated from %s" % snapshot[3])
			p_monitor.add_test_result(hits_replayed == hits_original)
			p_monitor.add_test("Bodies end where they did when re-simulated from %s" % snapshot[3])
			p_monitor.add_test_result(RapierPhysicsServer3D.space_get_bodies_transform(space, boxes) == bodies_original)

		viewport.queue_free()
		state_manager.queue_free()
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)

# Runs the ticks from p_tick on, starting at p_first_phase of p_tick. Returns the query hits.
func replay(p_tick: int, p_first_phase: int) -> Array[int]:
	var hits: Array[int] = []
	for t in range(p_tick, TICKS):
		var first_phase := p_first_phase if t == p_tick else MOVE_BEFORE
		if first_phase <= MOVE_BEFORE:
			move_before(t)
		if first_phase <= UPDATE_AND_QUERY:
			hits.append(update_and_query(t))
		if first_phase <= MOVE_AFTER_AND_STEP:
			move_after_and_step(t)
	return hits

# A tick as a game would run it. Move by script, update the queries and query, move some more, then
# step. The platform moves before the update and the area after it.
func move_before(p_tick: int) -> void:
	PhysicsServer3D.body_set_state(platform.get_rid(), PhysicsServer3D.BODY_STATE_TRANSFORM, Transform3D(Basis(Vector3.BACK, 0.002 * p_tick), Vector3(5 - (p_tick % 40) * 0.15, -14.5, 0)))

func update_and_query(p_tick: int) -> int:
	RapierPhysicsServer3D.space_update_queries(space)
	var query := PhysicsShapeQueryParameters3D.new()
	query.shape = probe
	query.transform = Transform3D(Basis(), Vector3(-7.5 + (p_tick % 60) * 0.25, -12.5, 0))
	query.collide_with_areas = true
	return PhysicsServer3D.space_get_direct_state(space).intersect_shape(query, 64).size()

func move_after_and_step(p_tick: int) -> void:
	PhysicsServer3D.area_set_transform(area.get_rid(), Transform3D(Basis(), Vector3(-7.5 + (p_tick % 60) * 0.25, -12.5, 0)))
	RapierPhysicsServer3D.space_step(space, 1.0 / 60)
	RapierPhysicsServer3D.space_flush_queries(space)

func make_shape(p_size: Vector3) -> CollisionShape3D:
	var collision_shape := CollisionShape3D.new()
	collision_shape.shape = BoxShape3D.new()
	collision_shape.shape.size = p_size
	return collision_shape
