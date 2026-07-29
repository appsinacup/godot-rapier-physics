extends PhysicsUnitTest2D

const TILE_SIZE := 16
const TILE_ROW := 20
const GRAVITY := 980.0

var simulation_duration := 5
var land_frames := 60
var jump_frames := 40

var lander: CharacterBody2D
var jumper: CharacterBody2D
var surface_y := 0.0
var half_height := 12.0

func test_description() -> String:
	return """Checks [One Way Collision] on TileMapLayer tiles.
	A TileMapLayer merges every tile of a quadrant into a single body whose shapes all sit at the
	body origin, so the one-way direction has to be resolved from the contact itself. The body must
	land on the tiles from above and still jump up through them from below.
	"""

func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision] with TileMapLayer tiles"

func build_one_way_tileset() -> TileSet:
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
	# A thin strip along the top of the tile, the usual shape of a one-way tile.
	tile_data.set_collision_polygon_points(0, 0, PackedVector2Array([
		Vector2(-8, -8), Vector2(8, -8), Vector2(8, -4), Vector2(-8, -4)]))
	tile_data.set_collision_polygon_one_way(0, 0, true)
	return tile_set

func make_character(p_position: Vector2) -> CharacterBody2D:
	var character := CharacterBody2D.new()
	var collision_shape := CollisionShape2D.new()
	var capsule := CapsuleShape2D.new()
	capsule.radius = 6
	capsule.height = half_height * 2.0
	collision_shape.shape = capsule
	character.add_child(collision_shape)
	character.position = p_position
	add_child(character)
	return character

func test_start() -> void:
	var layer := TileMapLayer.new()
	layer.tile_set = build_one_way_tileset()
	var first_column := int(CENTER.x / TILE_SIZE) - 8
	for x in range(first_column, first_column + 16):
		layer.set_cell(Vector2i(x, TILE_ROW), 0, Vector2i(0, 0))
	add_child(layer)

	# map_to_local() returns the centre of the cell, and the strip starts 8 px above it.
	surface_y = TILE_ROW * TILE_SIZE + TILE_SIZE / 2.0 - 8.0
	lander = make_character(Vector2(CENTER.x - 60.0, surface_y - 150.0))
	jumper = make_character(Vector2(CENTER.x + 60.0, surface_y + 120.0))

	var checks_tiles = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 1:
			p_monitor.data["highest"] = jumper.position.y
			jumper.velocity = Vector2(0, -700.0)
		lander.velocity.y += GRAVITY * get_physics_process_delta_time()
		lander.move_and_slide()
		if p_monitor.frame > land_frames:
			jumper.velocity.y += GRAVITY * get_physics_process_delta_time()
			jumper.move_and_slide()
			p_monitor.data["highest"] = min(p_monitor.data["highest"], jumper.position.y)

		if p_monitor.frame == land_frames:
			p_monitor.add_test("Lands on top of the one-way tiles")
			var resting_y := surface_y - half_height
			var success: bool = lander.is_on_floor() and lander.position.y < resting_y + 5.0
			if not success:
				p_monitor.add_test_error("Rested at y=%.1f (on_floor=%s), expected to stop near y=%.1f."
					% [lander.position.y, lander.is_on_floor(), resting_y])
			p_monitor.add_test_result(success)

		if p_monitor.frame == land_frames + jump_frames:
			p_monitor.add_test("Jumps up through the one-way tiles from below")
			var clearance := surface_y - half_height - 10.0
			var highest: float = p_monitor.data["highest"]
			var success := highest < clearance
			if not success:
				p_monitor.add_test_error("Only reached y=%.1f, expected to pass y=%.1f."
					% [highest, clearance])
			p_monitor.add_test_result(success)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks_tiles, simulation_duration)
