extends TestBase

@onready var body_a := PhysicsServer3D.body_create()
@onready var body_b := PhysicsServer3D.body_create()

func _ready():
	var space := PhysicsServer3D.space_create()
	PhysicsServer3D.body_set_space(body_a, space)
	PhysicsServer3D.body_set_space(body_b, space)
	test_pin_joint_empty()
	test_pin_joint()
	print("Pin joint tests passed.")

func test_pin_joint_empty():
	print("test_pin_joint_empty")

	# Invalid RIDs must never crash and must return safe defaults.
	assert(PhysicsServer3D.pin_joint_get_param(RID(), PhysicsServer3D.PIN_JOINT_BIAS) == 0.0)
	assert(PhysicsServer3D.pin_joint_get_param(RID(), PhysicsServer3D.PIN_JOINT_DAMPING) == 0.0)
	assert(PhysicsServer3D.pin_joint_get_param(RID(), PhysicsServer3D.PIN_JOINT_IMPULSE_CLAMP) == 0.0)
	assert(PhysicsServer3D.pin_joint_get_local_a(RID()) == Vector3.ZERO)
	assert(PhysicsServer3D.pin_joint_get_local_b(RID()) == Vector3.ZERO)

	PhysicsServer3D.pin_joint_set_param(RID(), PhysicsServer3D.PIN_JOINT_BIAS, 1.0)
	PhysicsServer3D.pin_joint_set_local_a(RID(), Vector3.ONE)
	PhysicsServer3D.pin_joint_set_local_b(RID(), Vector3.ONE)

	# Making a pin from invalid bodies must not crash and must not become a pin joint.
	PhysicsServer3D.joint_make_pin(RID(), RID(), Vector3.ZERO, RID(), Vector3.ZERO)
	PhysicsServer3D.joint_make_pin(RID(), body_a, Vector3.ZERO, RID(), Vector3.ZERO)
	PhysicsServer3D.joint_make_pin(RID(), body_a, Vector3.ZERO, body_b, Vector3.ZERO)

	print("Pin joint empty tests passed.")

func test_pin_joint():
	print("test_pin_joint")

	var joint := PhysicsServer3D.joint_create()
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)

	# A pin needs two valid bodies; with a missing body it stays an empty joint.
	PhysicsServer3D.joint_make_pin(joint, body_a, Vector3.ZERO, RID(), Vector3.ZERO)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)

	# Valid pin joint between two real bodies.
	var anchor_a := Vector3(1, 2, 3)
	var anchor_b := Vector3(4, 5, 6)
	PhysicsServer3D.joint_make_pin(joint, body_a, anchor_a, body_b, anchor_b)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_PIN)
	assert(PhysicsServer3D.pin_joint_get_local_a(joint) == anchor_a)
	assert(PhysicsServer3D.pin_joint_get_local_b(joint) == anchor_b)

	# Default solver params.
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_BIAS), 0.3)
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_DAMPING), 1.0)
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_IMPULSE_CLAMP), 0.0)

	# Params round-trip.
	PhysicsServer3D.pin_joint_set_param(joint, PhysicsServer3D.PIN_JOINT_BIAS, 0.5)
	PhysicsServer3D.pin_joint_set_param(joint, PhysicsServer3D.PIN_JOINT_DAMPING, 0.7)
	PhysicsServer3D.pin_joint_set_param(joint, PhysicsServer3D.PIN_JOINT_IMPULSE_CLAMP, 2.0)
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_BIAS), 0.5)
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_DAMPING), 0.7)
	assert_eq(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_IMPULSE_CLAMP), 2.0)

	# Anchors round-trip.
	PhysicsServer3D.pin_joint_set_local_a(joint, Vector3(7, 8, 9))
	PhysicsServer3D.pin_joint_set_local_b(joint, Vector3(-1, -2, -3))
	assert(PhysicsServer3D.pin_joint_get_local_a(joint) == Vector3(7, 8, 9))
	assert(PhysicsServer3D.pin_joint_get_local_b(joint) == Vector3(-1, -2, -3))

	# joint_clear resets the joint back to the base type.
	PhysicsServer3D.joint_clear(joint)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)
	assert(PhysicsServer3D.joint_is_disabled_collisions_between_bodies(joint) == true)

	# Params on the freed joint fall back to defaults and do not crash.
	PhysicsServer3D.free_rid(joint)
	assert(PhysicsServer3D.pin_joint_get_param(joint, PhysicsServer3D.PIN_JOINT_BIAS) == 0.0)
	assert(PhysicsServer3D.joint_get_type(joint) == PhysicsServer3D.JOINT_TYPE_MAX)
