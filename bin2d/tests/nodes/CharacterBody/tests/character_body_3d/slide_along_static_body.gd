extends PhysicsUnitTest3D

@export var static_body_shape: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.BOX
@export var static_body_position: Vector3 = Vector3(5,0,0)
@export var static_body_rotation_degrees: Vector3
@export var static_body_scale: float = 10.0
@export var character_body_speed = 30.0
@export var character_body_rotation_degrees: Vector3
@export var character_body_shapes_failing: Array[PhysicsTest3D.TestCollisionShape]

var simulation_duration := 6.0
var char_velocity: Vector3

enum State {
	ARRIVING,
	TOUCHING,
	DEPARTING,
}

func test_description() -> String:
	return "Checks that the body slides along a %s static body." % [PhysicsTest3D.shape_name(static_body_shape)]

func test_name() -> String:
	return "CharacterBody3D | testing if the body can slide along a %s static body, params: [rotation: %.1v, speed:%s]" % [PhysicsTest3D.shape_name(static_body_shape), character_body_rotation_degrees, character_body_speed]

func test_start() -> void:
	var slide_test_cbk = func(p_target: CharacterBody3D, p_monitor: GenericManualMonitor):
		if p_monitor.first_iteration:
			p_monitor.data['state'] = State.ARRIVING
			p_monitor.data['impact_pos'] = Vector3(INF, INF, INF)
		match p_monitor.data['state']:
			State.ARRIVING:
				if p_target.get_slide_collision_count() > 0:
					p_monitor.data['state'] = State.TOUCHING
					p_monitor.data['impact_pos'] = p_target.position
			State.TOUCHING:
				if p_target.velocity.length_squared() < 1.0:
					p_monitor.failed("Got stuck while touching.")
				if p_target.get_slide_collision_count() == 0:
					p_monitor.data['state'] = State.DEPARTING
			State.DEPARTING:
				if p_target.velocity.length_squared() < 1.0:
					p_monitor.failed("Got stuck during departure.")
		if p_monitor.frame == 60*4:
			if p_monitor.data['state'] != State.DEPARTING:
				p_monitor.failed("Failed to reach departing state.")
			elif p_target.position.x >= p_monitor.data['impact_pos'].x:
				p_monitor.failed("Failed to slide in the correct direction.")
			else:
				p_monitor.passed()
		# Always keep going.
		p_target.velocity = char_velocity

	var cpt := 1
	for char_shape in PhysicsTest3D.TestCollisionShape.values():
		# FIXME: Skip trimesh characters for now, due to lack of trimesh - trimesh collision support.
		if char_shape == PhysicsTest3D.TestCollisionShape.CONCAVE_POLYGON:
			continue
		# FIXME: Skip high poly convex characters for now, due to performance issues.
		if char_shape in [PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX, PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX]:
			continue

		# Create static body.
		var static_body = create_static(cpt, static_body_position, static_body_shape)
		static_body.rotation_degrees = static_body_rotation_degrees
		add_child(static_body)

		# Create character body.
		char_velocity = Vector3(-1, 0, -1).normalized() * character_body_speed
		var spawn_pos = -char_velocity * 2.0
		var char_body := create_character(cpt, spawn_pos, char_shape)
		char_body.velocity = char_velocity
		char_body.rotation_degrees = character_body_rotation_degrees
		add_child(char_body)

		# Create slide monitor.
		var slide_monitor = create_generic_manual_monitor(char_body, slide_test_cbk, simulation_duration)
		slide_monitor.test_name = "%s can slide along %s"  % [PhysicsTest3D.shape_name(char_shape), PhysicsTest3D.shape_name(static_body_shape)]
		if char_shape in character_body_shapes_failing:
			slide_monitor.expected_to_fail = true

		cpt += 1

func create_character(p_layer: int, p_position: Vector3, p_body_shape := PhysicsTest3D.TestCollisionShape.CAPSULE) -> CharacterBody3D:
	var character = CharacterBody3D.new()
	character.script = load("res://tests/nodes/CharacterBody/scripts/3d/character_body_3d_move_and_slide.gd")
	character.position = p_position
	character.collision_layer = 0
	character.collision_mask = 0
	character.set_collision_layer_value(p_layer, true)
	character.set_collision_mask_value(p_layer, true)
	var body_col: Node3D = PhysicsTest3D.get_default_collision_shape(p_body_shape, 2)
	character.add_child(body_col)
	return character

func create_static(p_layer: int, p_position: Vector3, p_body_shape := PhysicsTest3D.TestCollisionShape.BOX) -> StaticBody3D:
	var static_body = StaticBody3D.new()
	static_body.position = p_position
	static_body.collision_layer = 0
	static_body.collision_mask = 0
	static_body.set_collision_layer_value(p_layer, true)
	static_body.set_collision_mask_value(p_layer, true)
	var body_col: Node3D = PhysicsTest3D.get_default_collision_shape(p_body_shape, static_body_scale)
	static_body.add_child(body_col)
	return static_body
