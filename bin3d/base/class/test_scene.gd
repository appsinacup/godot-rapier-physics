class_name TestScene
extends Node

var runner: TestRunner
var start_time := 0.0
var is_performance := false

func _ready() -> void:
	Global.print_engine()
	if get_tree().get_root() == get_parent(): # autostart if the scene is executed alone
		test_start()

func test_start() -> void:
	start_time = Time.get_unix_time_from_system() 
	runner = TestRunner.new(self, true)
	runner.completed.connect(self.completed)
	for child in get_children():
		if child is PhysicsPerformanceTest2D or child is PhysicsPerformanceTest3D:
			is_performance = true
		if child is PhysicsTest2D or child is PhysicsTest3D:
			runner.add_test(child)
	for child in get_children():
		if child is PhysicsTest2D or child is PhysicsTest3D:
			remove_child(child)

	if is_performance:
		Global.NUMBER_TEST_PER_ROW = 1
		Global.MAXIMUM_PARALLEL_TESTS = 1
		DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	runner.run()
			
func completed() -> void:
	var duration := Time.get_unix_time_from_system() - start_time
	if is_performance:
		print_rich("[color=green] > COMPLETED IN %.2fs[/color]" % duration)
	else:
		Global.print_summary(duration)
	
	Global.exit()
