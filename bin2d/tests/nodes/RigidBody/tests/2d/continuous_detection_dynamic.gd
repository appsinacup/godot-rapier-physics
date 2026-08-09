extends PhysicsUnitTest2D

@export var speed: float = 25000
@export var simulation_duration: float = 0.1

const GAP := 700.0

var detect_head_on := false

func test_description() -> String:
	return """Checks that Continuous Collision Detection works between two moving bodies, not
	just against static geometry. Two fast rigid bodies are fired at each other; neither may
	pass through the other.
	"""

func test_name() -> String:
	return "RigidBody2D | testing CCD between moving bodies"

func test_start() -> void:
	var left := create_rigid_body(RigidBody2D.CCD_MODE_CAST_SHAPE, Vector2(speed, 0))
	left.position = CENTER - Vector2(GAP * 0.5, 120)
	left.body_entered.connect(func(_b): detect_head_on = true)

	var right := create_rigid_body(RigidBody2D.CCD_MODE_CAST_SHAPE, Vector2(-speed, 0))
	right.position = CENTER + Vector2(GAP * 0.5, -120)

	var fast := create_rigid_body(RigidBody2D.CCD_MODE_CAST_SHAPE, Vector2(speed, 0))
	fast.position = CENTER - Vector2(GAP * 0.5, -120)

	var slow := create_rigid_body(RigidBody2D.CCD_MODE_DISABLED, Vector2(-speed * 0.05, 0))
	slow.position = CENTER + Vector2(GAP * 0.5, 120)

	var head_on_lambda = func(_t, _m: GenericExpirationMonitor):
		return left.position.x <= right.position.x

	var mixed_lambda = func(_t, _m: GenericExpirationMonitor):
		return fast.position.x <= slow.position.x

	var collision_lambda = func(_t, _m: GenericExpirationMonitor):
		return detect_head_on

	var head_on := create_generic_expiration_monitor(left, head_on_lambda, null, simulation_duration)
	head_on.test_name = "Two bodies with CCD moving at each other do not pass through"
	head_on.expected_to_fail = true

	var mixed := create_generic_expiration_monitor(fast, mixed_lambda, null, simulation_duration)
	mixed.test_name = "A body with CCD does not pass through a slower moving body"

	var reported := create_generic_expiration_monitor(
		left, collision_lambda, null, simulation_duration
	)
	reported.test_name = "CCD between moving bodies reports the collision"
	reported.expected_to_fail = true

func create_rigid_body(p_ccd_mode: RigidBody2D.CCDMode, p_impulse: Vector2) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.add_child(PhysicsTest2D.get_default_collision_shape(TestCollisionShape.RECTANGLE))
	body.gravity_scale = 0
	body.continuous_cd = p_ccd_mode
	body.contact_monitor = true
	body.max_contacts_reported = 4
	add_child(body)
	body.apply_central_impulse(p_impulse)
	return body
