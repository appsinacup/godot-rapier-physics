extends PhysicsUnitTest2D


# A static body gathers its shapes into one compound collider, and the shapes arrive one at a time:
# the server is told about each of them with its own body_add_shape call. Every one of them has to
# end up in the compound. A TileMapLayer is the case that made this matter -- it merges a quadrant's
# tiles into one body and hands over one convex piece per shape -- so it is checked end to end here
# as well.

const TILE_SIZE := 16
const TILE_ROW := 22
const SHAPE_COUNT := 7
const PIECE_WIDTH := 40.0
const PIECE_HEIGHT := 20.0
const SETTLE_FRAME := 60

var simulation_duration := 8
var piecewise_body := RID()
var piecewise_shapes: Array[RID] = []
var piecewise_origin := Vector2.ZERO
var stair_fallers: Array[RigidBody2D] = []
var stair_surface_y: Array[float] = []

func test_description() -> String:
	return """Checks that a static body keeps every shape it is given, however many arrive after the
	first two, and that a TileMapLayer whose tiles decompose into several convex pieces collides on
	all of them.
	"""

func test_name() -> String:
	return "CollisionShape2D | a compound keeps every shape added to it"

func _exit_tree() -> void:
	for shape in piecewise_shapes:
		PhysicsServer2D.free_rid(shape)
	if piecewise_body.is_valid():
		PhysicsServer2D.free_rid(piecewise_body)

func build_tileset() -> TileSet:
	var image := Image.create(TILE_SIZE, TILE_SIZE, false, Image.FORMAT_RGBA8)
	image.fill(Color.WHITE)

	var tile_set := TileSet.new()
	tile_set.tile_size = Vector2i(TILE_SIZE, TILE_SIZE)
	tile_set.add_physics_layer()

	var source := TileSetAtlasSource.new()
	source.texture = ImageTexture.create_from_image(image)
	source.texture_region_size = Vector2i(TILE_SIZE, TILE_SIZE)
	source.create_tile(Vector2i(0, 0))
	tile_set.add_source(source, 0)

	var tile_data := source.get_tile_data(Vector2i(0, 0), 0)
	tile_data.add_collision_polygon(0)
	tile_data.set_collision_polygon_points(0, 0, PackedVector2Array([
		Vector2(-8, -8), Vector2(8, -8), Vector2(8, 8), Vector2(-8, 8)]))
	return tile_set

# A static body built through the server the way a TileMapLayer quadrant is: one add_shape per
# convex piece, each piece already placed in body space.
func build_piecewise_body() -> void:
	piecewise_origin = Vector2(CENTER.x - SHAPE_COUNT * PIECE_WIDTH / 2.0, 80.0)
	piecewise_body = PhysicsServer2D.body_create()
	PhysicsServer2D.body_set_mode(piecewise_body, PhysicsServer2D.BODY_MODE_STATIC)
	PhysicsServer2D.body_set_space(piecewise_body, get_world_2d().space)
	PhysicsServer2D.body_set_state(piecewise_body, PhysicsServer2D.BODY_STATE_TRANSFORM,
		Transform2D(0.0, piecewise_origin))
	for i in SHAPE_COUNT:
		var shape := PhysicsServer2D.convex_polygon_shape_create()
		var x := i * PIECE_WIDTH
		PhysicsServer2D.shape_set_data(shape, PackedVector2Array([
			Vector2(x, 0.0), Vector2(x + PIECE_WIDTH, 0.0),
			Vector2(x + PIECE_WIDTH, PIECE_HEIGHT), Vector2(x, PIECE_HEIGHT)]))
		piecewise_shapes.append(shape)
		PhysicsServer2D.body_add_shape(piecewise_body, shape)
		PhysicsServer2D.body_set_shape_as_one_way_collision(piecewise_body, i, false, 1.0)

# A staircase of tiles: the quadrant merges into one non-convex polygon, which Godot decomposes into
# several convex pieces and adds to the body one at a time.
func build_staircase() -> void:
	var layer := TileMapLayer.new()
	layer.tile_set = build_tileset()
	var first_column := int(CENTER.x / TILE_SIZE) - 12
	for step in 6:
		var top := TILE_ROW - step
		for x in range(first_column + step * 4, first_column + 24):
			for y in range(top, TILE_ROW + 2):
				layer.set_cell(Vector2i(x, y), 0, Vector2i(0, 0))
	add_child(layer)

	for step in 6:
		var top := TILE_ROW - step
		stair_surface_y.append(top * TILE_SIZE)
		var faller := RigidBody2D.new()
		var collision_shape := CollisionShape2D.new()
		var box := RectangleShape2D.new()
		box.size = Vector2(12, 12)
		collision_shape.shape = box
		faller.add_child(collision_shape)
		faller.position = Vector2((first_column + step * 4 + 1) * TILE_SIZE + 8, top * TILE_SIZE - 60)
		add_child(faller)
		stair_fallers.append(faller)

func test_start() -> void:
	build_piecewise_body()
	build_staircase()

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != SETTLE_FRAME:
			return

		var space := get_world_2d().direct_space_state
		var query := PhysicsShapeQueryParameters2D.new()
		var probe := RectangleShape2D.new()
		probe.size = Vector2(4, 4)
		query.shape = probe

		# Every piece has to answer for itself: a compound that stopped growing leaves the pieces
		# added after it a hole with no collider at all.
		p_monitor.add_test("Every shape added to the body has a collider")
		var missing := []
		var reported := {}
		for i in SHAPE_COUNT:
			query.transform = Transform2D(0.0, piecewise_origin
				+ Vector2(i * PIECE_WIDTH + PIECE_WIDTH / 2.0, PIECE_HEIGHT / 2.0))
			var hits := space.intersect_shape(query, 8)
			if hits.is_empty():
				missing.append(i)
			for hit in hits:
				if hit.rid == piecewise_body:
					reported[hit.shape] = true
		if not missing.is_empty():
			p_monitor.add_test_error("Shapes %s report no collider." % [missing])
		p_monitor.add_test_result(missing.is_empty())

		p_monitor.add_test("Each shape is reported under its own index")
		if reported.size() != SHAPE_COUNT:
			p_monitor.add_test_error("Reported indices %s, expected %d of them."
				% [reported.keys(), SHAPE_COUNT])
		p_monitor.add_test_result(reported.size() == SHAPE_COUNT)

		p_monitor.add_test("Bodies land on every step of a TileMapLayer staircase")
		var fell := []
		for i in stair_fallers.size():
			var resting_y: float = stair_surface_y[i] - 6.0
			if stair_fallers[i].global_position.y > resting_y + 10.0:
				fell.append("step %d rested at y=%.1f, expected near y=%.1f"
					% [i, stair_fallers[i].global_position.y, resting_y])
		for error in fell:
			p_monitor.add_test_error(error)
		p_monitor.add_test_result(fell.is_empty())

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
