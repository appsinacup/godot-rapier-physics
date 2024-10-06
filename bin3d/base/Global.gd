extends Node

enum TEST_MODE {
	REGRESSION,
	QUALITY,
	PERFORMANCE
}

# View
var WINDOW_SIZE := Vector2(1152,648)
var NUMBER_TEST_PER_ROW := 2
var MAXIMUM_PARALLEL_TESTS := NUMBER_TEST_PER_ROW * NUMBER_TEST_PER_ROW

# Output
var DEBUG := false
var VERBOSE := true
var NB_TESTS_COMPLETED := 0
var MONITOR_PASSED := 0
var MONITOR_FAILED := 0
var MONITOR_EXPECTED_TO_FAIL: Array[String] = []
var MONITOR_REGRESSION: Array[String] = []
var MONITOR_IMRPOVEMENT: Array[String] = []
var TEST_PASSED := 0

var RUN_2D_TEST := false
var RUN_3D_TEST := true

var engine_2d = "GodotPhysics2D"
var engine_3d = "GodotPhysics3D"

var PERFORMANCE_RESULT := {}

func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("ui_cancel"):
		exit()

func _ready() -> void:
	get_tree().debug_collisions_hint = true
	
	var setting_2d_engine = ProjectSettings.get("physics/2d/physics_engine")
	if setting_2d_engine != "DEFAULT" and setting_2d_engine != "GodotPhysics2D":
		engine_2d = setting_2d_engine
	
	var setting_3d_engine = ProjectSettings.get("physics/3d/physics_engine")
	if setting_3d_engine != "DEFAULT" and setting_3d_engine != "GodotPhysics3D":
		engine_3d = setting_3d_engine

func exit(p_code := 0) -> void:
	await get_tree().create_timer(1).timeout # sometimes the application quits before printing everything in the output
	get_tree().quit(p_code)

func print_summary(duration: float) -> void:
	var status = "FAILED" if Global.MONITOR_REGRESSION.size() != 0 else "SUCCESS"
	var color = "red" if Global.MONITOR_REGRESSION.size() != 0 else "green"

	if Global.VERBOSE and Global.MONITOR_EXPECTED_TO_FAIL.size() != 0:
		print_rich("\n[indent]%d Monitor(s) expected to fail:[/indent]" % [Global.MONITOR_EXPECTED_TO_FAIL.size()])
		var cpt := 0
		for expected in Global.MONITOR_EXPECTED_TO_FAIL:
			cpt += 1
			print_rich("[indent][indent]%d. %s[/indent][/indent]" % [cpt, expected])
	
	if Global.MONITOR_REGRESSION.size() != 0:
		print_rich("\n[indent]%d Regression(s):[/indent]" % [Global.MONITOR_REGRESSION.size()])
		var cpt := 0
		for regression in Global.MONITOR_REGRESSION:
			cpt += 1
			print_rich("[indent][indent][color=red]%d. %s[/color][/indent][/indent]" % [cpt, regression])
	if Global.MONITOR_IMRPOVEMENT.size() != 0:
		print_rich("\n[indent]%d Improvement(s):[/indent]" % [Global.MONITOR_IMRPOVEMENT.size()])
		var cpt := 0
		for improvement in Global.MONITOR_IMRPOVEMENT:
			cpt += 1
			print_rich("[indent][indent][color=green]%d. %s[/color][/indent][/indent]" % [cpt, improvement])

	var extra = ""
	if Global.MONITOR_REGRESSION.size() != 0 or Global.MONITOR_IMRPOVEMENT.size() != 0:
		extra = " | "
		if Global.MONITOR_REGRESSION.size() != 0:
			extra += "☹ %d regression(s) " % Global.MONITOR_REGRESSION.size()
		if Global.MONITOR_IMRPOVEMENT.size() != 0:
			extra += "❤️ %d improvement(s)" % Global.MONITOR_IMRPOVEMENT.size()	

	print_rich("\n[color=%s] > COMPLETED IN %.2fs | STATUS: %s (PASSED MONITORS: %d/%d)%s[/color]" % [color, duration, status, Global.MONITOR_PASSED, Global.MONITOR_PASSED + Global.MONITOR_FAILED, extra])
	
func print_engine() -> void:
	if Global.VERBOSE:
		var engine_txt := ""
		if Global.RUN_2D_TEST:
			engine_txt += " | 2D → %s" % [Global.engine_2d]
		if Global.RUN_3D_TEST:
			engine_txt += " | 3D → %s" % [Global.engine_3d]

		print_rich("[color=orange] > ENGINE:%s[/color]\n" % engine_txt)
