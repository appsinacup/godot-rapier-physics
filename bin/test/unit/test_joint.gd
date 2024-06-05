extends TestBase

var body_a := PhysicsServer2D.body_create()
var body_b := PhysicsServer2D.body_create()

func _ready():
	test_base_joint_empty()
	test_spring_joint_empty()
	test_spring_joint()
	test_pin_joint_empty()
	print("Joint tests passed.")

func test_base_joint_empty():
	PhysicsServer2D.joint_disable_collisions_between_bodies(RID(), false)
	PhysicsServer2D.joint_disable_collisions_between_bodies(RID(), true)
	var disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(RID())
	assert(disabled == true)
	print("Base joint empty tests passed.")

func test_spring_joint_empty():
	var stiffness = PhysicsServer2D.damped_spring_joint_get_param(RID(), PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_STIFFNESS)
	assert(stiffness == 0)
	var length = PhysicsServer2D.damped_spring_joint_get_param(RID(), PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_REST_LENGTH)
	assert(length == 0)
	var damping = PhysicsServer2D.damped_spring_joint_get_param(RID(), PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_DAMPING)
	assert(damping == 0)
	#var invalid = PhysicsServer2D.damped_spring_joint_get_param(RID(), 100)
	#assert(invalid == 0)
	# Test no crash here
	PhysicsServer2D.damped_spring_joint_set_param(RID(), PhysicsServer2D.DAMPED_SPRING_STIFFNESS, 1)
	PhysicsServer2D.damped_spring_joint_set_param(RID(), PhysicsServer2D.DAMPED_SPRING_REST_LENGTH, 2)
	PhysicsServer2D.damped_spring_joint_set_param(RID(), PhysicsServer2D.DAMPED_SPRING_DAMPING, 3)
	# PhysicsServer2D.damped_spring_joint_set_param(RID(), 100, 4)
	PhysicsServer2D.joint_clear(RID())
	var joint_type := PhysicsServer2D.joint_get_type(RID())
	assert(joint_type == PhysicsServer2D.JOINT_TYPE_MAX)
	PhysicsServer2D.joint_make_damped_spring(RID(), Vector2.ZERO, Vector2.ZERO, RID(), RID())
	PhysicsServer2D.joint_make_damped_spring(RID(), Vector2.ZERO, Vector2.ZERO, body_a, RID())
	PhysicsServer2D.joint_make_damped_spring(RID(), Vector2.ZERO, Vector2.ZERO, RID(), body_b)
	PhysicsServer2D.joint_make_damped_spring(RID(), Vector2.ZERO, Vector2.ZERO, body_a, body_a)
	PhysicsServer2D.joint_make_damped_spring(RID(), Vector2.ZERO, Vector2.ZERO, body_a, body_b)
	PhysicsServer2D.joint_make_damped_spring(body_a, Vector2.ZERO, Vector2.ZERO, body_a, body_b)
	print("Spring joint empty tests passed.")

# Test functions for joint functions
func test_spring_joint():
	
	# Base Joint
	var joint_rid = PhysicsServer2D.joint_create()
	assert(joint_rid != null)
	var joint_type = PhysicsServer2D.joint_get_type(joint_rid)
	# Should not be able to set any of specific values
	assert(joint_type == PhysicsServer2D.JointType.JOINT_TYPE_MAX)
	var stiffness = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_STIFFNESS)
	assert(stiffness == 0)
	var length = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_REST_LENGTH)
	assert(length == 0)
	var damping = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_DAMPING)
	assert(damping == 0)

	PhysicsServer2D.damped_spring_joint_set_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_STIFFNESS, 2)
	PhysicsServer2D.damped_spring_joint_set_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_REST_LENGTH, 3)
	PhysicsServer2D.damped_spring_joint_set_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_DAMPING, 4)
	
	
	stiffness = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_STIFFNESS)
	assert(stiffness == 2)
	length = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_REST_LENGTH)
	assert(length == 3)
	damping = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DAMPED_SPRING_DAMPING)
	assert(damping == 4)
	PhysicsServer2D.free_rid(joint_rid)
	
	PhysicsServer2D.joint_clear(joint_rid)
	PhysicsServer2D.joint_disable_collisions_between_bodies(joint_rid, false)
	
	joint_type = PhysicsServer2D.joint_get_type(joint_rid)
	assert(joint_type == PhysicsServer2D.JointType.JOINT_TYPE_MAX)
	
	var disabled_collisions = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled_collisions == true)

	PhysicsServer2D.joint_make_damped_spring(joint_rid, Vector2.ZERO, Vector2.ZERO, RID())
	
	print("Spring joint tests passed.")
	
func test_pin_joint_empty():
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, RID(), RID())
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_a, RID())
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_a, body_a)
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_b, body_a)
	PhysicsServer2D.joint_make_pin(body_a, Vector2.ZERO, body_b, body_a)

	var angular_limit = PhysicsServer2D.pin_joint_get_flag(RID(), PhysicsServer2D.PinJointFlag.PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED)
	assert(angular_limit == false)
	var motor_enabled = PhysicsServer2D.pin_joint_get_flag(RID(), PhysicsServer2D.PinJointFlag.PIN_JOINT_FLAG_MOTOR_ENABLED)
	assert(motor_enabled == false)

	var limit_lower = PhysicsServer2D.pin_joint_get_param(RID(), PhysicsServer2D.PinJointParam.PIN_JOINT_LIMIT_LOWER)
	assert(limit_lower == 0.0)
	var limit_upper = PhysicsServer2D.pin_joint_get_param(RID(), PhysicsServer2D.PinJointParam.PIN_JOINT_LIMIT_UPPER)
	assert(limit_upper == 0.0)
	var motor_velocity = PhysicsServer2D.pin_joint_get_param(RID(), PhysicsServer2D.PinJointParam.PIN_JOINT_MOTOR_TARGET_VELOCITY)
	assert(motor_velocity == 0.0)
	var softness = PhysicsServer2D.pin_joint_get_param(RID(), PhysicsServer2D.PinJointParam.PIN_JOINT_SOFTNESS)
	assert(softness == 0.0)
	
	PhysicsServer2D.pin_joint_set_flag(RID(), PhysicsServer2D.PinJointFlag.PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED, true)
	PhysicsServer2D.pin_joint_set_param(RID(), PhysicsServer2D.PinJointParam.PIN_JOINT_LIMIT_LOWER, 1.2)
	
	
	print("Pin joint empty tests passed.")
