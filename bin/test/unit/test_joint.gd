extends TestBase

func _ready():
	test_joint_functions()
	print("Joint tests passed.")

# Test functions for joint functions
func test_joint_functions():
	var joint_rid = RID()
	var enabled = true
	
	var created_joint_rid = PhysicsServer2D.joint_create()
	assert(created_joint_rid != null)

	var param_value = PhysicsServer2D.damped_spring_joint_get_param(joint_rid, PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_DAMPING)
	assert(param_value == 0.0)

	PhysicsServer2D.damped_spring_joint_set_param(joint_rid, PhysicsServer2D.DampedSpringParam.DAMPED_SPRING_DAMPING, 1.2)
	PhysicsServer2D.free_rid(joint_rid)
	
	PhysicsServer2D.joint_clear(joint_rid)
	PhysicsServer2D.joint_disable_collisions_between_bodies(joint_rid, enabled)
	
	var joint_type = PhysicsServer2D.joint_get_type(joint_rid)
	assert(joint_type == PhysicsServer2D.JointType.JOINT_TYPE_MAX)
	
	var disabled_collisions = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled_collisions == true)

	PhysicsServer2D.joint_make_damped_spring(joint_rid, Vector2.ZERO, Vector2.ZERO, RID())
	PhysicsServer2D.joint_make_groove(joint_rid, Vector2.ZERO, Vector2.ZERO, Vector2.ZERO, RID())
	PhysicsServer2D.joint_make_pin(joint_rid, Vector2.ZERO, RID())

	var pin_joint_flag = PhysicsServer2D.pin_joint_get_flag(joint_rid, PhysicsServer2D.PinJointFlag.PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED)
	assert(pin_joint_flag == false)

	var pin_joint_param = PhysicsServer2D.pin_joint_get_param(joint_rid, PhysicsServer2D.PinJointParam.PIN_JOINT_LIMIT_LOWER)
	assert(pin_joint_param == 0.0)
	
	PhysicsServer2D.pin_joint_set_flag(joint_rid, PhysicsServer2D.PinJointFlag.PIN_JOINT_FLAG_ANGULAR_LIMIT_ENABLED, true)
	PhysicsServer2D.pin_joint_set_param(joint_rid, PhysicsServer2D.PinJointParam.PIN_JOINT_LIMIT_LOWER, 1.2)
	print("Joint functions test passed")
