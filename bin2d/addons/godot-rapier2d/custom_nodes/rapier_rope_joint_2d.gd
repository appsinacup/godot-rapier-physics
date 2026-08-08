class_name RapierRopeJoint2D
extends Node2D

## Keeps two bodies within [member max_distance] of each other.
## Attaches to both bodies at this node's global position.

## First body attached to the joint.
@export var node_a: NodePath
## Second body attached to the joint.
@export var node_b: NodePath
## Rope length: how far apart the attachment points may drift.
@export_range(1.0, 100000.0, 0.1, "or_greater", "suffix:px") var max_distance: float = 100.0:
	set(value):
		max_distance = value
		_rebuild()
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
	RapierPhysicsServer2D.joint_make_rope(
		_joint, global_position, global_position, max_distance, a.get_rid(), b.get_rid())

func _destroy() -> void:
	if _joint.is_valid():
		PhysicsServer2D.free_rid(_joint)
		_joint = RID()
