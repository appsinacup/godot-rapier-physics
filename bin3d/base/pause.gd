extends Node

var paused = false
var step_once = false

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	process_mode = PROCESS_MODE_ALWAYS
	get_tree().paused = paused

func _physics_process(_delta):
	if Input.is_action_just_pressed('pause'):
		paused = !paused
		get_tree().paused = paused
		step_once = false
	
	if paused:
		if step_once:
			get_tree().paused = true
			step_once = false
		elif Input.is_action_just_pressed('pause_next_frame'):
			# step once
			get_tree().paused = false
			step_once = true

