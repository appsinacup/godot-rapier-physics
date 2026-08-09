extends PhysicsUnitTest2D

const STEP_HEIGHT := 20.0
const STEP_WIDTH := 60.0
const STEP_COUNT := 4

const RAY_LENGTH := 30.0
const BODY_HALF_SIZE := 20.0
const BODY_LIFT := 30.0

const GROUND_Y := 450.0
const SPEED := 400.0
const SIMULATION_DURATION := 1.5
const SLOPE_DEGREES := 60.0
const TOLERANCE := 3.0

const STAIRS_LAYER := 1
const PLANTED_LAYER := 2
const SLIDING_LAYER := 3

var slope_apex := Vector2(500, GROUND_Y - 300)

func test_description() -> String:
	return """Checks SeparationRayShape2D. The ray holds the body at a fixed height above
	whatever is underneath it, so a raised body box climbs a staircase and comes to rest one
	ray-length above the top step rather than sitting on it. With [slide_on_slope] off the
	reported normal is the ray axis, so the body stands on a slope steeper than
	[floor_max_angle]; with it on the real surface normal is reported and the body slides.
	"""

func test_name() -> String:
	return "CharacterBody2D | testing [SeparationRayShape2D]"

func test_start() -> void:
	test_climbs_stairs()
	test_slope_behaviour()

func test_climbs_stairs() -> void:
	var start := Vector2(-500, GROUND_Y - RAY_LENGTH)
	add_child(build_ground(-560, 560, GROUND_Y, [STAIRS_LAYER]))
	for i in STEP_COUNT:
		var left := -200 + i * STEP_WIDTH
		add_child(build_ground(left, left + STEP_WIDTH, step_top(i), [STAIRS_LAYER]))
	var platform_top := step_top(STEP_COUNT - 1)
	add_child(build_ground(-200 + STEP_COUNT * STEP_WIDTH, 560, platform_top, [STAIRS_LAYER]))

	var character := create_character(start, false, STAIRS_LAYER)

	var callback := func(p_target: CharacterBody2D, _p_monitor: GenericExpirationMonitor):
		p_target.velocity.x = SPEED

	var check := func(p_target: CharacterBody2D, _p_monitor: GenericExpirationMonitor):
		if not p_target.is_on_floor():
			return false
		if p_target.position.x < 0:
			return false
		return absf(p_target.position.y - (platform_top - RAY_LENGTH)) < TOLERANCE

	var monitor := create_generic_expiration_monitor(
		character, check, callback, SIMULATION_DURATION
	)
	monitor.test_name = "The ray carries the body up stairs and holds it above the top step"

func test_slope_behaviour() -> void:
	add_child(build_slope(300, [PLANTED_LAYER, SLIDING_LAYER]))

	var spawn_x := 450.0
	var spawn := Vector2(spawn_x, slope_surface_y(spawn_x) - RAY_LENGTH)

	var callback := func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		p_monitor.data["drop"] = maxf(p_monitor.data["drop"], p_target.position.y - spawn.y)

	var planted := create_character(spawn, false, PLANTED_LAYER)
	var planted_check := func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		return p_monitor.data["drop"] < TOLERANCE and p_target.is_on_floor()
	var planted_monitor := create_generic_expiration_monitor(
		planted, planted_check, callback, SIMULATION_DURATION
	)
	planted_monitor.test_name = "Without [slide_on_slope] the body stays put on a steep slope"
	planted_monitor.data["drop"] = 0.0

	var sliding := create_character(spawn, true, SLIDING_LAYER)
	var sliding_check := func(p_target: CharacterBody2D, p_monitor: GenericExpirationMonitor):
		return p_monitor.data["drop"] > STEP_HEIGHT and not p_target.is_on_floor()
	var sliding_monitor := create_generic_expiration_monitor(
		sliding, sliding_check, callback, SIMULATION_DURATION
	)
	sliding_monitor.test_name = "With [slide_on_slope] the body slides down a steep slope"
	sliding_monitor.data["drop"] = 0.0

static func step_top(p_index: int) -> float:
	return GROUND_Y - (p_index + 1) * STEP_HEIGHT

func slope_surface_y(p_x: float) -> float:
	return slope_apex.y + (slope_apex.x - p_x) * tan(deg_to_rad(SLOPE_DEGREES))

func create_character(
	p_position: Vector2, p_slide_on_slope: bool, p_layer: int
) -> CharacterBody2D:
	var character := CharacterBody2D.new()
	character.script = load(
		"res://tests/nodes/CharacterBody/scripts/2d/character_body_2d_move_and_slide_with_gravity.gd"
	)
	character.position = p_position
	character.floor_snap_length = RAY_LENGTH
	character.collision_layer = 0
	character.collision_mask = 0
	character.set_collision_layer_value(p_layer, true)
	character.set_collision_mask_value(p_layer, true)

	var body_col := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = Vector2(BODY_HALF_SIZE, BODY_HALF_SIZE) * 2.0
	body_col.shape = box
	body_col.position = Vector2(0, -BODY_LIFT - BODY_HALF_SIZE)
	character.add_child(body_col)

	var ray_col := CollisionShape2D.new()
	var ray := SeparationRayShape2D.new()
	ray.length = RAY_LENGTH
	ray.slide_on_slope = p_slide_on_slope
	ray_col.shape = ray
	character.add_child(ray_col)

	add_child(character)
	return character

func build_ground(
	p_left: float, p_right: float, p_top: float, p_layers: Array
) -> StaticBody2D:
	var body := PhysicsTest2D.get_static_body_with_collision_shape(
		Rect2(p_left, p_top, p_right - p_left, 200), PhysicsTest2D.TestCollisionShape.RECTANGLE, true
	)
	return apply_layers(body, p_layers)

func build_slope(p_height: float, p_layers: Array) -> StaticBody2D:
	var run := p_height / tan(deg_to_rad(SLOPE_DEGREES))
	var body := StaticBody2D.new()
	var col := CollisionPolygon2D.new()
	col.polygon = PackedVector2Array([
		slope_apex,
		slope_apex + Vector2(-run, p_height),
		slope_apex + Vector2(200, p_height),
	])
	body.add_child(col)
	return apply_layers(body, p_layers)

static func apply_layers(p_body: StaticBody2D, p_layers: Array) -> StaticBody2D:
	p_body.collision_layer = 0
	p_body.collision_mask = 0
	for layer in p_layers:
		p_body.set_collision_layer_value(layer, true)
		p_body.set_collision_mask_value(layer, true)
	return p_body
