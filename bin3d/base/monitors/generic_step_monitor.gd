extends Monitor
class_name GenericStepMonitor

var multi_test_auto_step = []
var first_iteration = true
var current_step := 0
var total_step := 0
var test_lambda: Callable
var physics_step_cbk = null
var test_name := "Auto Step Monitor"

var target: Node
var data := {} # Dictionnary used to pass data to the monitor
	
func monitor_name() -> String:
	return test_name
	
func setup(p_test_step_lamba: Callable,  p_physics_step_cbk = null, p_maximum_duration := 5.0):
	test_lambda = p_test_step_lamba
	physics_step_cbk = p_physics_step_cbk
	monitor_maximum_duration = p_maximum_duration

	var cpt = 0
	while(p_test_step_lamba.call(cpt, target, self) != null):
		cpt += 1
	total_step = cpt

# Overload the method to avoid [error_message] for [EXPIRATION] mode
func _process(delta: float) -> void:
	# Maximum duration
	monitor_duration += delta
	if monitor_duration > monitor_maximum_duration:
		error_message = "The maximum duration has been exceeded (> %.1f s)" % [monitor_maximum_duration]
		monitor_completed()
		return
		
# Called every frame. 'delta' is the elapsed time since the previous frame.
func _physics_process(_delta: float) -> void:
	frame += 1
	# If all steps are completed
	var last_step = total_step - 1
	if current_step == last_step:
		passed()
		return
	
	if physics_step_cbk:
		physics_step_cbk.call(current_step, target, first_iteration, self)

	# Test according to the lambda provided
	var result = test_lambda.call(current_step, target, self)
	if first_iteration and not result:
		failed("The verification of the first step has failed")
		return
	first_iteration = false	
	
	# If the test is false, we checks if we can pass to the next step
	if not result:
		var is_next = test_lambda.call(current_step + 1, target, self)
		
		if not is_next:
			failed("Error during the transition from step %d to step %d" % [current_step, current_step + 1])
			return
		else:
			current_step += 1
			if physics_step_cbk:
				physics_step_cbk.call(current_step, target, true, self)


func add_test(p_step_should_reach:int, p_name: String):
	add_sub_test(p_name)
	multi_test_auto_step.append(p_step_should_reach)
	
func monitor_completed() -> void:
	if multi_test_auto_step.size() > 0:
		for i in range(0, multi_test_auto_step.size()):
			var result = current_step >= multi_test_auto_step[i]
			add_test_result(result)
	super()
