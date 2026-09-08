extends PhysicsUnitTest3D


# The plugin prints its version on startup, and get_version() is how a running game reads the same
# number back -- so a bug report can name the build it came from. It is answered from the compiled
# plugin, so it has to agree with the version the addon ships as, and it must not need the project
# to be running on Rapier to answer.

var simulation_duration := 2

func test_description() -> String:
	return """Checks that RapierPhysicsServer3D.get_version() reports the version the addon ships
	as.
	"""

func test_name() -> String:
	return "RapierPhysicsServer3D | testing [get_version]"

func addon_version() -> String:
	var config := ConfigFile.new()
	if config.load("res://addons/godot-rapier3d/plugin.info.cfg") != OK:
		return ""
	return str(config.get_value("plugin", "version", ""))

func test_start() -> void:
	var checks = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame != 1:
			return

		var version := str(RapierPhysicsServer3D.get_version())

		p_monitor.add_test("Reports a version")
		if version.is_empty():
			p_monitor.add_test_error("get_version() returned an empty string.")
		p_monitor.add_test_result(not version.is_empty())

		p_monitor.add_test("Reports it as major.minor.patch")
		var parts := version.split(".")
		var numeric := parts.size() == 3
		for part in parts:
			numeric = numeric and part.is_valid_int()
		if not numeric:
			p_monitor.add_test_error("Reported '%s', expected three numbers separated by dots."
				% [version])
		p_monitor.add_test_result(numeric)

		# The compiled plugin and the addon it is shipped inside are versioned in two different
		# places, and a released build once carried the previous number because only one of them
		# was bumped. Reading them back apart is exactly what this API is for.
		p_monitor.add_test("Agrees with the version the addon declares")
		var declared := addon_version()
		if declared != version:
			p_monitor.add_test_error("get_version() says '%s', plugin.info.cfg says '%s'."
				% [version, declared])
		p_monitor.add_test_result(declared == version)

		p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
