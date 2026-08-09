extends PhysicsUnitTest3D

const RAY_LENGTH := 0.6
const STEP_HEIGHT := 0.3
const STEP_DEPTH := 1.2
const STEP_COUNT := 4

const BODY_HALF_HEIGHT := 0.5
const BODY_LIFT := 0.6
const RAY_POINTS_DOWN := Vector3(90, 0, 0)

const GROUND_TOP := 0.0
const STAIRS_START_Z := -1.0
const SPEED := 3.0
const SIMULATION_DURATION := 4.0
const TOLERANCE := 0.05

func test_description() -> String:
	return """Checks SeparationRayShape3D. The ray holds the body a fixed distance above
	whatever is underneath it, so a body box raised clear of the ground climbs a staircase
	and comes to rest one ray-length above the top step rather than sitting on it.
	"""

func test_name() -> String:
	return "CharacterBody3D | testing [SeparationRayShape3D] climbing stairs"

func test_start() -> void:
	add_child(build_platform(GROUND_TOP, 5.0, -20.0))
	for i in STEP_COUNT:
		var near_z := STAIRS_START_Z - i * STEP_DEPTH
		add_child(build_platform(step_top(i), near_z, near_z - STEP_DEPTH))
	var platform_top := step_top(STEP_COUNT - 1)
	var platform_near_z := STAIRS_START_Z - STEP_COUNT * STEP_DEPTH
	add_child(build_platform(platform_top, platform_near_z, -20.0))

	var start := Vector3(0, GROUND_TOP + RAY_LENGTH, 0)
	var character := create_character(start)

	var callback := func(p_target: CharacterBody3D, _p_monitor: GenericExpirationMonitor):
		p_target.velocity.z = -SPEED

	var check := func(p_target: CharacterBody3D, _p_monitor: GenericExpirationMonitor):
		if not p_target.is_on_floor():
			return false
		if p_target.position.z > platform_near_z:
			return false
		return absf(p_target.position.y - (platform_top + RAY_LENGTH)) < TOLERANCE

	var monitor := create_generic_expiration_monitor(
		character, check, callback, SIMULATION_DURATION
	)
	monitor.test_name = "The ray carries the body up stairs and holds it above the top step"

static func step_top(p_index: int) -> float:
	return GROUND_TOP + (p_index + 1) * STEP_HEIGHT

func create_character(p_position: Vector3) -> CharacterBody3D:
	var character := CharacterBody3D.new()
	character.script = load(
		"res://tests/nodes/CharacterBody/scripts/3d/character_body_3d_move_and_slide_with_gravity.gd"
	)
	character.position = p_position
	character.floor_snap_length = RAY_LENGTH

	var body_col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.8, BODY_HALF_HEIGHT * 2.0, 0.8)
	body_col.shape = box
	body_col.position = Vector3(0, BODY_LIFT + BODY_HALF_HEIGHT, 0)
	character.add_child(body_col)

	character.add_child(build_ray(RAY_LENGTH, false))
	add_child(character)
	return character

static func build_ray(p_length: float, p_slide_on_slope: bool) -> CollisionShape3D:
	var ray_col := CollisionShape3D.new()
	var ray := SeparationRayShape3D.new()
	ray.length = p_length
	ray.slide_on_slope = p_slide_on_slope
	ray_col.shape = ray
	ray_col.rotation_degrees = RAY_POINTS_DOWN
	return ray_col

static func build_platform(p_top: float, p_near_z: float, p_far_z: float) -> StaticBody3D:
	const THICKNESS := 1.0
	var depth: float = absf(p_near_z - p_far_z)
	var body := StaticBody3D.new()
	body.position = Vector3(0, p_top - THICKNESS * 0.5, (p_near_z + p_far_z) * 0.5)
	var col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(8, THICKNESS, depth)
	col.shape = box
	body.add_child(col)
	return body
