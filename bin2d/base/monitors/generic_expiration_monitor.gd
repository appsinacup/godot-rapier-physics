extends Monitor
class_name GenericExpirationMonitor

var first_iteration = true
var test_lambda: Callable
var physics_step_cbk = null
var test_name := "Generic Expiration monitor"
var target: Node
var data := {} # Dictionnary used to pass data to the monitor
	
func monitor_name() -> String:
	return test_name
	
func setup(p_test_lambda: Callable, p_physics_step_cbk, p_maximum_duration := 5.0):
	test_lambda = p_test_lambda
	physics_step_cbk = p_physics_step_cbk
	monitor_maximum_duration = p_maximum_duration

# Overload the method to avoid [error_message] for [EXPIRATION] mode
func _process(delta: float) -> void:
	# Maximum duration
	monitor_duration += delta
	if monitor_duration > monitor_maximum_duration:
		success = test_lambda.call(target, self)
		monitor_completed()
		return
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(_delta: float) -> void:
	frame += 1
	if physics_step_cbk:
		physics_step_cbk.call(target, self)

	first_iteration = false	

