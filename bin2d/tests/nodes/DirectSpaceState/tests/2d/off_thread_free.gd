extends PhysicsUnitTest2D

var simulation_duration := 1.0

func test_description() -> String:
	return """Checks that a RID freed from another thread, as the .NET finalizer does, stays alive
	until the next step instead of touching the physics state from that thread.
	"""

func test_name() -> String:
	return "PhysicsServer2D | freeing a RID from another thread"

func test_start() -> void:
	var space := PhysicsServer2D.space_create()
	PhysicsServer2D.space_set_active(space, true)
	var worker := Thread.new()
	worker.start(PhysicsServer2D.free_rid.bind(space))
	worker.wait_to_finish()
	var alive_until_step := PhysicsServer2D.space_is_active(space)

	var run = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 3:
			return
		p_monitor.add_test("Stays alive until the next step")
		p_monitor.add_test_result(alive_until_step)
		p_monitor.add_test("Is freed by the step")
		p_monitor.add_test_result(not PhysicsServer2D.space_is_active(space))
		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
