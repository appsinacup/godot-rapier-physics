class_name PhysicsPerformanceTest2D
extends PhysicsTest2D

var NB_FRAME_SMOOTHING = 5
var WARMING_SKIPPED_FRAMES = 20

var _total_frame := 0
var _fps_label : Label

var _current_fps := 60.0
var _prev_tick_ms := -1.0

var _max_fps := 0.0
var _min_fps := 9999.0
var _average_fps := 0.0
var _average_record := 0
var _smoothed_fps := 60.0

var _frame_cpt := 0
var _fps_buffer := 0.0

var extra_text := []

var _warming := true # wait few frame before start monitoring

func _init() -> void:
	process_mode = PROCESS_MODE_DISABLED

func _process(_delta: float) -> void:
	
	_frame_cpt += 1
	_total_frame += 1
		
	# Skip Frames
	if _warming and _frame_cpt == WARMING_SKIPPED_FRAMES:
		_warming = false
	
	if _warming:
		return
		
	# Start Computing FPS
	if _prev_tick_ms == -1.0:
		_prev_tick_ms = Time.get_ticks_usec()
		_frame_cpt = 0
		return
	
	var new_tick: float = Time.get_ticks_usec()
	_current_fps = 1000000.0 / (new_tick - _prev_tick_ms)
	_prev_tick_ms = new_tick
	
	# Smooth FPS and compute average
	_fps_buffer += _current_fps

	if _frame_cpt == NB_FRAME_SMOOTHING:
		_smoothed_fps = _fps_buffer / _frame_cpt
		_frame_cpt = 0
		_fps_buffer = 0.0
		_average_record += 1
		_average_fps += _smoothed_fps
		
		if _smoothed_fps > _max_fps:
			_max_fps = _smoothed_fps
		if _smoothed_fps < _min_fps:
			_min_fps = _smoothed_fps

	if _total_frame % 30 ==0 and _fps_label:
		_fps_label.text = "%d FPS" % [_smoothed_fps] 

func get_smoothed_fps():
	return _smoothed_fps
	
func get_fps():
	return _current_fps

func test_start() -> void:
	super()
	
	_fps_label = Label.new()
	_fps_label.position = Vector2(20,40)
	_fps_label.set("theme_override_font_sizes/font_size", 18)
	add_child(_fps_label)
	
	process_mode = Node.PROCESS_MODE_INHERIT

func register_result(p_name: String, result: String):
	if not Global.PERFORMANCE_RESULT.has(get_name()):
		Global.PERFORMANCE_RESULT[get_name()] = []
	Global.PERFORMANCE_RESULT[get_name()].append([p_name, _min_fps, _max_fps, _average_fps / _average_record, result])

func test_completed(delay := 0) -> void:
	super()
	if not extra_text.is_empty():
		for s in extra_text:
			output += "[indent][indent][color=green]%s[/color][/indent][/indent]\n" % [s]
	if Global.PERFORMANCE_RESULT.has(get_name()):
		for result in Global.PERFORMANCE_RESULT[get_name()]:
			output += "[indent][indent][color=orange] → %s : [b]%s[/b][/color] - [color=purple][b]FPS[/b] | min: [b]%d[/b] max: [b]%d[/b] avg: [b]%d[/b][/color][/indent][/indent]\n" % [result[0], result[4], result[1], result[2], result[3]]
	else:
			output += "[indent][indent][color=orange] Simulation completed[/color][/indent][/indent]\n"
	print_rich(output)
	process_mode = PROCESS_MODE_DISABLED
	if delay != 0:
		await get_tree().create_timer(delay).timeout 

	if has_method("clean"):
		call("clean")

	completed.emit()
	queue_free()

