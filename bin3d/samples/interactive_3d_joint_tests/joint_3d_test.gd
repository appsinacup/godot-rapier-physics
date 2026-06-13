extends Node3D

@onready var static_body_a := %StaticBody3DBodyA as StaticBody3D
@onready var rigid_body_b := %RigidBody3DBodyB as RigidBody3D

# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(_delta: float) -> void:
	var move_in_plane := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var move := Vector3(move_in_plane.x, -move_in_plane.y, 0.0)

	if Input.is_key_pressed(Key.KEY_L):
		move.z = 1.0
	elif Input.is_key_label_pressed(Key.KEY_K):
		move.z = -1.0

	rigid_body_b.apply_central_force(move * 4.0)

	var torque := Vector3.ZERO
	if Input.is_key_pressed(Key.KEY_W):
		torque.x = 1.0
	elif Input.is_key_label_pressed(Key.KEY_S):
		torque.x = -1.0
	if Input.is_key_pressed(Key.KEY_A):
		torque.y = -1.0
	elif Input.is_key_label_pressed(Key.KEY_D):
		torque.y = 1.0
	if Input.is_key_pressed(Key.KEY_Q):
		torque.z = -1.0
	elif Input.is_key_pressed(Key.KEY_E):
		torque.z = 1.0
	
	# convert torque from local -> global
	torque = rigid_body_b.global_transform.basis * torque
	rigid_body_b.apply_torque(torque * 4.0)
