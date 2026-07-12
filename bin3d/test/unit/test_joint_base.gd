extends Node3D


func _ready() -> void:
	test_base_joint_empty()
	test_base_joint()

func test_base_joint_empty():
	print("test_base_joint_empty")
	PhysicsServer3D.joint_disable_collisions_between_bodies(RID(), false)
	PhysicsServer3D.joint_disable_collisions_between_bodies(RID(), true)
	PhysicsServer3D.joint_clear(RID())
	var disabled = PhysicsServer3D.joint_is_disabled_collisions_between_bodies(RID())
	assert(disabled == true)
	var joint_type := PhysicsServer3D.joint_get_type(RID())
	assert(joint_type == PhysicsServer3D.JOINT_TYPE_MAX)

func test_base_joint():
	print("test_base_joint")
	var joint_rid = PhysicsServer3D.joint_create()
	# A freshly created joint has no specific type yet.
	assert(PhysicsServer3D.joint_get_type(joint_rid) == PhysicsServer3D.JOINT_TYPE_MAX)
	var disabled = PhysicsServer3D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer3D.joint_disable_collisions_between_bodies(joint_rid, false)
	disabled = PhysicsServer3D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == false)
	PhysicsServer3D.joint_disable_collisions_between_bodies(joint_rid, true)
	disabled = PhysicsServer3D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer3D.joint_clear(RID())
	disabled = PhysicsServer3D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer3D.joint_clear(joint_rid)
	PhysicsServer3D.free_rid(joint_rid)
