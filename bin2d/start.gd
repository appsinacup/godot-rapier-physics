extends Node2D

@export var mode: Global.TEST_MODE = Global.TEST_MODE.REGRESSION
var runner: TestRunner
var start_time := 0.0

func _init():
	runner = TestRunner.new(self)
	runner.completed.connect(self.completed)
	## Get Arguments
	var arguments = {}
	for argument in OS.get_cmdline_args():
		if argument.find("=") > -1:
			var key_value = argument.split("=")
			arguments[key_value[0].lstrip("--")] = key_value[1]
		else: # Options without an argument 
			arguments[argument.lstrip("--")] = ""
	
# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var test_folder := []
	if mode == Global.TEST_MODE.REGRESSION:
		test_folder.append_array(["nodes", "features"])
	elif mode == Global.TEST_MODE.PERFORMANCE:
		test_folder.append("nodes")
		Global.MAXIMUM_PARALLEL_TESTS = 1
		Global.NUMBER_TEST_PER_ROW = 1
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	elif mode == Global.TEST_MODE.QUALITY:
		test_folder.append("quality")

	var result = {}
	for folder in test_folder:
		find_test(result, folder)
	
	for key in result:
		for scene_file in result[key]:
			var scene: TestScene = load(scene_file).instantiate()	
			for child in scene.get_children():
				if (Global.RUN_2D_TEST and child is PhysicsTest2D) or (Global.RUN_3D_TEST and child is PhysicsTest3D):
					runner.add_test(child)
			scene.queue_free()
	
	if Global.VERBOSE:
		var engine_txt := ""
		if Global.RUN_2D_TEST:
			engine_txt += " | 2D → %s" % [Global.engine_2d]
		if Global.RUN_3D_TEST:
			engine_txt += " | 3D → %s" % [Global.engine_3d]

		print_rich("[color=orange] > MODE: [b]%s[/b] ([b]%d[/b] SCENES FOUND) - ENGINE:%s[/color]\n" % [Global.TEST_MODE.keys()[mode], runner.total_tests, engine_txt])
	start_time = Time.get_unix_time_from_system()
	runner.run()

func find_test(result: Dictionary, folder: String) -> void:
	
	var path = "res://tests/" + folder  + "/"
	var dir = DirAccess.open(path)

	if dir:
		dir.list_dir_begin()
		var file_name = dir.get_next()
	
		while file_name != "":
			var dir_path = dir.get_current_dir().path_join(file_name)
			var test_dir =  DirAccess.open(dir_path)
		
			if test_dir:
				test_dir.list_dir_begin()
				var test_file_name = test_dir.get_next()
				var test_scene_list = []
				while test_file_name != "":
					var test_path = test_dir.get_current_dir().path_join(test_file_name)
					if test_path.ends_with(".tscn"):
						test_scene_list.append(test_path)
					test_file_name = test_dir.get_next()
				if not test_scene_list.is_empty():
					result[file_name] = test_scene_list
			else:
				print_rich("[color:red]Failed to open the directory: [/color] % [dir_path]")
				@warning_ignore("assert_always_false")
				assert(false, "Failed to read the directory")
			file_name = dir.get_next()

func completed() -> void:
	var duration := Time.get_unix_time_from_system() - start_time

	Global.print_summary(duration)
		
	var error_code = 1 if Global.MONITOR_REGRESSION.size() != 0 else 0
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()
	#Global.exit(error_code)
