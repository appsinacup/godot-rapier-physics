extends Node2D
class_name ChainDemo

## Demonstrates a programmatic Rapier multibody chain with IK, matching the Rapier testbed.
## Attach this script to a Node2D in your scene.
## Requires the godot-rapier-physics plugin with multibody support (PR #452).

@export var num_segments: int = 10 : set = set_num_segments
@export var segment_size: float = 0.1 : set = set_segment_size
@export var joint_type: int = 2 : set = set_joint_type  ## 1=Multibody (dynamic), 2=Multibody Kinematic (stable)
@export var use_mouse_target: bool = true : set = set_use_mouse_target

var root: StaticBody2D
var segments: Array[RigidBody2D] = []
var joints: Array[RapierPinJoint2D] = []
var ik_target: Node2D
var ground: StaticBody2D

func set_num_segments(value: int) -> void:
	num_segments = max(1, value)
	_recreate_chain()

func set_segment_size(value: float) -> void:
	segment_size = max(0.01, value)
	_recreate_chain()

func set_joint_type(value: int) -> void:
	joint_type = value
	_update_joint_types()

func set_use_mouse_target(value: bool) -> void:
	use_mouse_target = value

func _ready() -> void:
	_recreate_chain()
	_update_joint_types()
	set_process(true)

func _physics_process(delta: float) -> void:
	if use_mouse_target and ik_target:
		ik_target.global_position = get_global_mouse_position()

func _recreate_chain() -> void:
	# Clear existing
	for child in get_children():
		child.queue_free()
	segments.clear()
	joints.clear()
	
	# Ground (matches testbed)
	ground = StaticBody2D.new()
	ground.name = "Ground"
	ground.position = Vector2(0, -0.5)
	var ground_coll = CollisionShape2D.new()
	ground_coll.shape = RectangleShape2D.new()
	ground_coll.shape.size = Vector2(2.0, 0.02)  # Wide thin ground
	ground.add_child(ground_coll)
	add_child(ground)
	
	# Root (fixed, like testbed's fixed root body)
	root = StaticBody2D.new()
	root.name = "ChainRoot"
	root.position = Vector2.ZERO
	var root_coll = CollisionShape2D.new()
	root_coll.shape = RectangleShape2D.new()
	root_coll.shape.size = Vector2(segment_size / 4.0, segment_size)  # Matches segment for vis
	root.add_child(root_coll)
	add_child(root)
	
	# Build chain upward (initial rest pose for gravity to pull down)
	var prev_body: Node2D = root
	var y_offset: float = 0.0
	for i in range(num_segments):
		# Segment body (dynamic)
		var body = RigidBody2D.new()
		body.name = "Segment%d" % i
		
		# Initial position for straight-up chain
		var anchor1_y: float = 0.0 if i == 0 else segment_size / 2.0
		body.position = Vector2(0, prev_body.position.y + anchor1_y + segment_size / 2.0)
		y_offset += segment_size
		
		# Collider (sensor-like thin tall; density default >0 for mass)
		var coll = CollisionShape2D.new()
		coll.shape = RectangleShape2D.new()
		coll.shape.size = Vector2(segment_size / 4.0, segment_size)  # full extents: narrow width, full height
		body.add_child(coll)
		
		# Dynamic settings: prevent sleeping, no damping
		body.sleeping = false
		body.gravity_scale = 1.0
		body.linear_damp = 0.0
		body.angular_damp = 0.0
		
		add_child(body)
		body.owner = self
		segments.append(body)
		
		# Joint (Pin = Revolute in 2D)
		var joint = RapierPinJoint2D.new()
		joint.name = "Joint%d" % i
		joint.node_a = prev_body.get_path()
		joint.node_b = body.get_path()
		
		# World anchor position = prev center + local_anchor1
		joint.position = prev_body.position + Vector2(0, anchor1_y)
		
		add_child(joint)
		joints.append(joint)
		
		prev_body = body
	
	# IK target (follows mouse)
	ik_target = Node2D.new()
	ik_target.name = "IKTarget"
	ik_target.position = Vector2(2.0, 2.0)  # Initial away
	add_child(ik_target)
	
	# Assign IK only to LAST joint
	if joints.size() > 0:
		joints[-1].ik_target = ik_target

func _update_joint_types() -> void:
	for joint in joints:
		joint.joint_type = joint_type
