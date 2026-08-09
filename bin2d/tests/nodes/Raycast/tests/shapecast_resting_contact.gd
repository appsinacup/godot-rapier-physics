extends PhysicsUnitTest2D

const BALL_RADIUS := 4.0
const TARGET_RADIUS := 15.0
const TARGET_X := 200.0
const SPEED := 30.0
const SIMULATION_DURATION := 12.0
const LAYER := 1

var ball: RigidBody2D
var caster: ShapeCast2D

func test_description() -> String:
	return """Checks that a zero-length ShapeCast2D reports a body it is resting against. The
	solver settles bodies exactly touching rather than overlapping, so a query that only
	accepts strict penetration reports nothing, which showed up as collisions going undetected
	at low impact speeds.
	"""

func test_name() -> String:
	return "ShapeCast2D | testing detection against a resting body"

func test_start() -> void:
	add_child(build_target())
	ball = build_ball()

	var callback := func(_p_target: RigidBody2D, p_monitor: GenericExpirationMonitor):
		caster.force_shapecast_update()
		var gap: float = (
			ball.global_position.distance_to(Vector2(TARGET_X, 0))
			- (BALL_RADIUS + TARGET_RADIUS)
		)
		if gap <= 0.0:
			p_monitor.data["touched"] = true
			p_monitor.data["detected"] = (
				p_monitor.data["detected"] or caster.is_colliding()
			)

	var check := func(_p_target: RigidBody2D, p_monitor: GenericExpirationMonitor):
		return p_monitor.data["touched"] and p_monitor.data["detected"]

	var monitor := create_generic_expiration_monitor(
		ball, check, callback, SIMULATION_DURATION
	)
	monitor.test_name = "A zero-length ShapeCast2D detects a body it rests against"
	monitor.data["touched"] = false
	monitor.data["detected"] = false

func build_ball() -> RigidBody2D:
	var body := RigidBody2D.new()
	body.gravity_scale = 0.0
	body.can_sleep = false
	body.collision_layer = 0
	body.collision_mask = 0
	body.set_collision_layer_value(LAYER, true)
	body.set_collision_mask_value(LAYER, true)

	var col := CollisionShape2D.new()
	var shape := CircleShape2D.new()
	shape.radius = BALL_RADIUS
	col.shape = shape
	body.add_child(col)

	caster = ShapeCast2D.new()
	caster.enabled = false
	var cast_shape := CircleShape2D.new()
	cast_shape.radius = BALL_RADIUS
	caster.shape = cast_shape
	caster.target_position = Vector2.ZERO
	caster.collision_mask = 0
	caster.set_collision_mask_value(LAYER, true)
	body.add_child(caster)

	add_child(body)
	body.linear_velocity = Vector2(SPEED, 0)
	return body

func build_target() -> StaticBody2D:
	var body := StaticBody2D.new()
	body.position = Vector2(TARGET_X, 0)
	body.collision_layer = 0
	body.collision_mask = 0
	body.set_collision_layer_value(LAYER, true)
	body.set_collision_mask_value(LAYER, true)
	var col := CollisionShape2D.new()
	var shape := CircleShape2D.new()
	shape.radius = TARGET_RADIUS
	col.shape = shape
	body.add_child(col)
	return body
