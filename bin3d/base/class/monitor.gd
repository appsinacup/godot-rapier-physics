extends Node
class_name Monitor
signal completed

var multi_test_list: Array[Dictionary] = []
var multi_test_current := 0
var monitor_duration := 0.0
var monitor_maximum_duration := 10.0
var error_message := ""
var success := false
var started := false
var frame := 0 # physics frame
var expected_to_fail := false
var engine_expected_to_fail: Array[String] = []

var text: Dictionary:
	set(value):
		text = value

func _init() -> void:
	process_mode = Node.PROCESS_MODE_DISABLED

func test_start() -> void:
	process_mode = Node.PROCESS_MODE_INHERIT
	started = true

func is_test_passed() -> bool:
	return success

func is_expected_to_fail() -> bool:
	if engine_expected_to_fail.size() > 0:
		if Global.engine_2d in engine_expected_to_fail or Global.engine_3d in engine_expected_to_fail:
			return true
	return expected_to_fail

func is_sub_test_expected_to_fail(p_test:Dictionary) -> bool:
	if p_test["engine_expected_to_fail"].size() > 0:
		if Global.engine_2d in p_test["engine_expected_to_fail"] or Global.engine_3d in p_test["engine_expected_to_fail"]:
			return true
	return p_test["expected_to_fail"]
	
func _process(delta: float) -> void:
	# Maximum duration
	monitor_duration += delta
	if monitor_duration > monitor_maximum_duration:
		error_message = "The maximum duration has been exceeded (> %.1f s)" % [monitor_maximum_duration]
		monitor_completed()
		return

func monitor_name() -> String:
	@warning_ignore("assert_always_false")
	assert(false, "ERROR: You must implement monitor_name()")
	return ""

func monitor_completed() -> void:
	completed.emit()
	process_mode = PROCESS_MODE_DISABLED
	
func failed(p_message = ""):
	error_message = p_message
	success = false
	monitor_completed()

func passed():
	success = true
	monitor_completed()

func add_sub_test(p_name: String, p_expected_to_fail := false) -> void:
	multi_test_list.append({
		"name": p_name,
		"result": false,
		"errors": [],
		"expected_to_fail": p_expected_to_fail,
		"engine_expected_to_fail": []
	})

func add_test_expected_to_fail():
	multi_test_list[multi_test_current].expected_to_fail = true
	
func add_test_engine_expected_to_fail(p_engine: Array[String]):
	multi_test_list[multi_test_current].engine_expected_to_fail.append_array(p_engine)
	
func add_test_error(p_error: String):
	multi_test_list[multi_test_current].errors.append(p_error)

func add_test_result(p_result: bool):
	multi_test_list[multi_test_current].result = p_result
	multi_test_current += 1
