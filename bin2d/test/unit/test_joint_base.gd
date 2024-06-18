extends Node2D


func _ready() -> void:
	test_base_joint_empty()
	test_base_joint()

func test_base_joint_empty():
	PhysicsServer2D.joint_disable_collisions_between_bodies(RID(), false)
	PhysicsServer2D.joint_disable_collisions_between_bodies(RID(), true)
	PhysicsServer2D.joint_clear(RID())
	var disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(RID())
	assert(disabled == true)
	print("Base joint empty tests passed.")

func test_base_joint():
	var joint_rid = PhysicsServer2D.joint_create()
	var disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer2D.joint_disable_collisions_between_bodies(joint_rid, false)
	disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == false)
	PhysicsServer2D.joint_disable_collisions_between_bodies(joint_rid, true)
	disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer2D.joint_clear(RID())
	disabled = PhysicsServer2D.joint_is_disabled_collisions_between_bodies(joint_rid)
	assert(disabled == true)
	PhysicsServer2D.joint_clear(joint_rid)
	print("Base joint tests passed.")
