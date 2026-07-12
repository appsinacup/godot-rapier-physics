extends TestBase

# 3D has no damped-spring joint (that is a 2D-only joint). The closest structural
# analogue that Rapier implements in 3D is the hinge (revolute) joint, so the 3D
# suite exercises that instead.

@onready var body_a := PhysicsServer3D.body_create()
@onready var body_b := PhysicsServer3D.body_create()

func _ready() -> void:
	var space := PhysicsServer3D.space_create()
	PhysicsServer3D.body_set_space(body_a, space)
	PhysicsServer3D.body_set_space(body_b, space)
	test_hinge_joint_empty()
	test_hinge_joint()
	print("Hinge joint tests passed.")

func test_hinge_joint_empty():
	print("test_hinge_joint_empty")

	# Invalid RIDs must never crash and must return safe defaults.
	assert(PhysicsServer3D.hinge_joint_get_param(RID(), PhysicsServer3D.HINGE_JOINT_LIMIT_UPPER) == 0.0)
	assert(PhysicsServer3D.hinge_joint_get_param(RID(), PhysicsServer3D.HINGE_JOINT_LIMIT_LOWER) == 0.0)
	assert(PhysicsServer3D.hinge_joint_get_param(RID(), PhysicsServer3D.HINGE_JOINT_MOTOR_TARGET_VELOCITY) == 0.0)
	assert(PhysicsServer3D.hinge_joint_get_flag(RID(), PhysicsServer3D.HINGE_JOINT_FLAG_USE_LIMIT) == false)
	assert(PhysicsServer3D.hinge_joint_get_flag(RID(), PhysicsServer3D.HINGE_JOINT_FLAG_ENABLE_MOTOR) == false)

	PhysicsServer3D.hinge_joint_set_param(RID(), PhysicsServer3D.HINGE_JOINT_LIMIT_UPPER, 1.0)
	PhysicsServer3D.hinge_joint_set_flag(RID(), PhysicsServer3D.HINGE_JOINT_FLAG_USE_LIMIT, true)

	# Making a hinge from invalid bodies must not crash and must not become a hinge.
	PhysicsServer3D.joint_make_hinge(RID(), RID(), Transform3D.IDENTITY, RID(), Transform3D.IDENTITY)
	PhysicsServer3D.joint_make_hinge(RID(), body_a, Transform3D.IDENTITY, RID(), Transform3D.IDENTITY)

	print("Hinge joint empty tests passed.")

func test_hinge_joint():
	print("test_hinge_joint")

	var joint := PhysicsServer3D.joint_create()
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)

	# A hinge needs two valid bodies; with a missing body it stays an empty joint.
	PhysicsServer3D.joint_make_hinge(joint, body_a, Transform3D.IDENTITY, RID(), Transform3D.IDENTITY)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)

	# Valid hinge joint between two real bodies.
	PhysicsServer3D.joint_make_hinge(joint, body_a, Transform3D.IDENTITY, body_b, Transform3D(Basis.IDENTITY, Vector3(0, 1, 0)))
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_HINGE)

	# Flags default to false and round-trip.
	assert(PhysicsServer3D.hinge_joint_get_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_USE_LIMIT) == false)
	assert(PhysicsServer3D.hinge_joint_get_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_ENABLE_MOTOR) == false)
	PhysicsServer3D.hinge_joint_set_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_USE_LIMIT, true)
	PhysicsServer3D.hinge_joint_set_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_ENABLE_MOTOR, true)
	assert(PhysicsServer3D.hinge_joint_get_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_USE_LIMIT) == true)
	assert(PhysicsServer3D.hinge_joint_get_flag(joint, PhysicsServer3D.HINGE_JOINT_FLAG_ENABLE_MOTOR) == true)

	# Params round-trip.
	PhysicsServer3D.hinge_joint_set_param(joint, PhysicsServer3D.HINGE_JOINT_LIMIT_LOWER, -1.0)
	PhysicsServer3D.hinge_joint_set_param(joint, PhysicsServer3D.HINGE_JOINT_LIMIT_UPPER, 1.5)
	PhysicsServer3D.hinge_joint_set_param(joint, PhysicsServer3D.HINGE_JOINT_MOTOR_TARGET_VELOCITY, 3.0)
	assert_eq(PhysicsServer3D.hinge_joint_get_param(joint, PhysicsServer3D.HINGE_JOINT_LIMIT_LOWER), -1.0)
	assert_eq(PhysicsServer3D.hinge_joint_get_param(joint, PhysicsServer3D.HINGE_JOINT_LIMIT_UPPER), 1.5)
	assert_eq(PhysicsServer3D.hinge_joint_get_param(joint, PhysicsServer3D.HINGE_JOINT_MOTOR_TARGET_VELOCITY), 3.0)

	# joint_clear resets the joint back to the base type.
	PhysicsServer3D.joint_clear(joint)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)

	# Params on the freed joint fall back to defaults and do not crash.
	PhysicsServer3D.free_rid(joint)
	assert(PhysicsServer3D.hinge_joint_get_param(joint, PhysicsServer3D.HINGE_JOINT_LIMIT_UPPER) == 0.0)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)
