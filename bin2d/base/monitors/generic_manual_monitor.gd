extends Monitor
class_name GenericManualMonitor

var first_iteration = true
var test_lambda: Callable
var test_name := "Generic Manual Monitor"
var fail_on_expiration := true

var target: Node
var data := {} # Dictionnary used to pass data to the monitor
	
func monitor_name() -> String:
	return test_name
	
func setup(p_test_lambda: Callable, p_maximum_duration := 5.0):
	test_lambda = p_test_lambda
	monitor_maximum_duration = p_maximum_duration

# Overload the method to avoid [error_message] for [EXPIRATION] mode
func _process(delta: float) -> void:
	# Maximum duration
	monitor_duration += delta
	if monitor_duration > monitor_maximum_duration:
		if fail_on_expiration:
			failed("The maximum duration has been exceeded (> %.1f s)" % [monitor_maximum_duration])
		else:
			passed()
		return
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(_delta: float) -> void:
	frame += 1
	test_lambda.call(target, self)
	first_iteration = false	

func add_test(p_name: String):
	add_sub_test(p_name)
