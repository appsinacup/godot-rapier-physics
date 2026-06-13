# Example of using RapierPinJoint2D motor position features 
# to simulate a cantilever beam
extends StaticBody2D

@export var number_of_segments : int = 10	
@export var starting_stiffness: float = 100000.0
@export var damping : float = 500.0
@export var segment_length : float = 100.0
@export var starting_segment_height : float = 50.0
@export var starting_segment_mass : float = 1.0
@export_range(0.5,1.0) var segment_narrowing_factor : float = 0.9

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var rb : RigidBody2D
	var cs : CollisionShape2D
	var sp : RectangleShape2D
	var pj : RapierPinJoint2D
	
	var height := starting_segment_height
	var mass := starting_segment_mass
	var last_path := get_path()
	var originx := 0.0
	
	for i in range(number_of_segments):
		rb = RigidBody2D.new()
		cs = CollisionShape2D.new()
		sp = RectangleShape2D.new()
		sp.size.x = segment_length
		sp.size.y = height
		cs.shape = sp
		rb.transform.origin.x = originx + segment_length*0.5
		rb.mass = mass
		rb.add_child(cs)
		add_child(rb)

		pj = RapierPinJoint2D.new()
		pj.node_a = last_path
		last_path = rb.get_path()
		pj.node_b = last_path
		
		# Check physics texts for correct stiffness proportionality
		# It's just a workable guess that angular stiffness should reduce 
		# proportional to beam height. 
		pj.motor_position_stiffness = starting_stiffness * height / starting_segment_height
		pj.motor_position_damping = damping
		pj.motor_position_target_angle = deg_to_rad(0.0)
		pj.motor_position_enabled = true
		pj.transform.origin.x = originx
		
		# joint_type = 0 - Impulse
		# joint_type = 1 - Multibody
		# joint_type = 2 - Multibody Kinematic
		# Multibody acheives a better result in this case
		pj.joint_type = 0 
		add_child(pj)
		
		height = height * segment_narrowing_factor
		mass = mass * segment_narrowing_factor * segment_narrowing_factor
		originx += segment_length
			
