class_name PhysicsUnitTest2D
extends PhysicsTest2D

var monitors: Array[Monitor]
var monitor_completed := 0
var start_time := 0.0

func _ready() -> void:
	super()
	start_time = Time.get_unix_time_from_system()

func register_monitors(p_monitors: Array[Monitor], p_owner: Node, p_start:= true):
	for monitor in p_monitors:
		monitors.append(monitor)
		monitor.completed.connect(self.on_monitor_completed)
		p_owner.add_child(monitor)
		monitor.owner = p_owner
		monitor.target = p_owner
		if p_start:
			monitor.test_start()

func on_monitor_completed() -> void:
	monitor_completed += 1
	if monitor_completed == monitors.size():
		test_completed()

func test_completed() -> void:
	super()
	var contain_error = false
	for monitor in monitors:
		if not contain_error and not monitor.call("is_test_passed"):
			contain_error = true
			output += "\t\t > %s" % [test_description()]
	
	for monitor in monitors:
		if monitor.has_method("is_test_passed"):
			if monitor.call("is_test_passed") == null:
				var bad_test_name = monitor.monitor_name() if monitor.monitor_name() else test_name()
				print_rich("[color=red]STATUS: FAILED - FATAL ERROR: The monitor [%s] returns null as result, please check the Callable arguments, arguments type[/color]" % [bad_test_name])
				get_tree().quit(1)
				return
			# multi_test_case
			if not monitor.multi_test_list.is_empty():
				for sub_test in monitor.multi_test_list:
					var subs_result = "[color=red]✗[/color]" if not sub_test.result else "[color=green]✓[/color]"
					if sub_test.result:
						Global.MONITOR_PASSED += 1
						if monitor.is_sub_test_expected_to_fail(sub_test):
							Global.MONITOR_IMRPOVEMENT.append("%s > %s" % [test_name(), sub_test.name])
							subs_result += " ❤️ (improvement)"
					else:
						Global.MONITOR_FAILED += 1
						if not monitor.is_sub_test_expected_to_fail(sub_test):
							Global.MONITOR_REGRESSION.append("%s > %s" % [test_name(), sub_test.name])
							subs_result += " ☹ (regression)"
						else:
							Global.MONITOR_EXPECTED_TO_FAIL.append("%s > %s" % [test_name(), sub_test.name])
					
					output += "[indent][indent][indent] → %s : %s[/indent][/indent][/indent]\n" % [sub_test.name, subs_result]
					for sub_test_error in sub_test.errors:
						output += "[indent][indent][indent][indent][color=red]Error: %s[/color][/indent][/indent][/indent][/indent]\n" % sub_test_error
			else:
				var passed:bool = monitor.call("is_test_passed")
				var result =  "[color=green]✓[/color]" if passed else "[color=red]✗[/color]"
				if passed:
					Global.MONITOR_PASSED += 1
					if monitor.is_expected_to_fail():
						Global.MONITOR_IMRPOVEMENT.append("%s > %s" % [test_name(), monitor.monitor_name()])
						result += " ❤️ (improvement)"
				else:
					Global.MONITOR_FAILED += 1
					if not monitor.is_expected_to_fail():
						Global.MONITOR_REGRESSION.append("%s > %s" % [test_name(), monitor.monitor_name()])
						result += " ☹ (regression)"
					else:
						Global.MONITOR_EXPECTED_TO_FAIL.append("%s > %s" % [test_name(), monitor.monitor_name()])
						
				output += "[indent][indent][indent] → %s : %s[/indent][/indent][/indent]\n" % [monitor.monitor_name(), result]
				if not passed and monitor.error_message != "" :
					output += "[color=red][indent][indent][indent] %s [/indent][/indent][/indent][/color]\n" % [monitor.error_message]
		elif monitor.has_method("get_score"):
			output += "[indent][indent][indent] → %s : score [b]%f[/b][/indent][/indent][/indent]\n" % [monitor.monitor_name(), monitor.call("get_score")]
		else:
			@warning_ignore("assert_always_false")
			assert(false, "Monitor without is_test_passed or get_score method")
	if Global.VERBOSE:
		print_rich(output)
	process_mode = PROCESS_MODE_DISABLED
	completed.emit()
	if get_tree().get_root() == get_parent(): # autostart is the scene is alone
		var label := Label.new()
		label.position = TOP_RIGHT + Vector2(-270,30)
		label.text = "Test completed | status: %s" % ["SUCCESS" if Global.MONITOR_FAILED == 0 else "FAILED"]
		add_child(label)
		var duration := Time.get_unix_time_from_system() - start_time
		Global.print_summary(duration)
	else:
		queue_free()

func create_generic_step_monitor(p_target: Node, p_test_lambda: Callable,  p_physics_step_cbk = null, p_maximum_duration := 5.0, p_auto_start:= true) -> GenericStepMonitor:
	var instance: GenericStepMonitor = load("res://base/monitors/generic_step_monitor.gd").new()
	register_monitors([instance as Monitor], p_target, p_auto_start)
	instance.setup(p_test_lambda, p_physics_step_cbk, p_maximum_duration)
	return instance

func create_generic_manual_monitor(p_target: Node, p_test_lambda: Callable, p_maximum_duration := 5.0, p_fail_on_expiration := true, p_auto_start:= true) -> GenericManualMonitor:
	var instance: GenericManualMonitor = load("res://base/monitors/generic_manual_monitor.gd").new()
	instance.fail_on_expiration = p_fail_on_expiration
	register_monitors([instance as Monitor], p_target, p_auto_start)
	instance.setup(p_test_lambda, p_maximum_duration)
	return instance

func create_generic_expiration_monitor(p_target: Node, p_test_lambda: Callable, p_physics_step_cbk = null, p_maximum_duration := 5.0, p_auto_start:= true) -> GenericExpirationMonitor:
	var instance: GenericExpirationMonitor = load("res://base/monitors/generic_expiration_monitor.gd").new()
	register_monitors([instance as Monitor], p_target, p_auto_start)
	instance.setup(p_test_lambda, p_physics_step_cbk, p_maximum_duration)
	return instance
