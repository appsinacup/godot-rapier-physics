extends Node2D

@onready var beam := %CantileveredBeam as StaticBody2D
@onready var lever_joint := %RapierPinJoint2D as RapierPinJoint2D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _process(_delta: float)	->void:
	if Input.is_action_just_pressed("ui_accept"):
		for node in beam.get_children():
			var pj := node as RapierPinJoint2D
			if pj:
				pj.motor_position_enabled = not pj.motor_position_enabled
		lever_joint.motor_position_enabled = not lever_joint.motor_position_enabled	
