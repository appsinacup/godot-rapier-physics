class_name RapierFixedJoint2D
extends Node2D

## Locks all relative motion between two bodies so they move as one rigid assembly.

## First body attached to the joint.
@export var node_a: NodePath
## Second body attached to the joint.
@export var node_b: NodePath
## Disable to let the joint keep the bodies' collision with each other.
@export var disable_collision: bool = true:
	set(value):
		disable_collision = value
		_rebuild()

var _joint: RID

func _ready() -> void:
	_rebuild()

func _exit_tree() -> void:
	_destroy()

func _rebuild() -> void:
	_destroy()
	if not is_inside_tree():
		return
	var a := get_node_or_null(node_a) as PhysicsBody2D
	var b := get_node_or_null(node_b) as PhysicsBody2D
	if a == null or b == null:
		return
	_joint = PhysicsServer2D.joint_create()
	PhysicsServer2D.joint_disable_collisions_between_bodies(_joint, disable_collision)
	RapierPhysicsServer2D.joint_make_fixed(_joint, global_position, a.get_rid(), b.get_rid())

func _destroy() -> void:
	if _joint.is_valid():
		PhysicsServer2D.free_rid(_joint)
		_joint = RID()
