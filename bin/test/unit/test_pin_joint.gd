extends TestBase

var body_a := PhysicsServer2D.body_create()
var body_b := PhysicsServer2D.body_create()
var body_a_with_space1 := PhysicsServer2D.body_create()
var body_b_with_space1 := PhysicsServer2D.body_create()
var body_a_with_space2 := PhysicsServer2D.body_create()
var body_b_with_space2 := PhysicsServer2D.body_create()

func _ready():
	var new_space1 := PhysicsServer2D.space_create()
	var new_space2 := PhysicsServer2D.space_create()
	PhysicsServer2D.body_set_space(body_a_with_space1, new_space1)
	PhysicsServer2D.body_set_space(body_a_with_space2, new_space2)
	PhysicsServer2D.body_set_space(body_b_with_space1, new_space1)
	PhysicsServer2D.body_set_space(body_b_with_space2, new_space2)
	test_pin_joint_empty()
	print("Pin joint tests passed.")

func test_pin_joint_empty():
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, RID(), RID())
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_a, RID())
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_a, body_a)
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_a_with_space1, body_a_with_space1)
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_b, body_a)
	PhysicsServer2D.joint_make_pin(RID(), Vector2.ZERO, body_b_with_space1, body_a_with_space1)
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
