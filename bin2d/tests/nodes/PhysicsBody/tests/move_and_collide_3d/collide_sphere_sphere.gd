extends PhysicsUnitTest3D

# Tolerances
const POSITION_TOLERANCE := 0.01
const NORMAL_TOLERANCE := 0.001

var simulation_duration := 20
var test_failed := false

func test_description() -> String:
	return """Checks whether the move_and_collide between a CharacterBody sphere and a big static sphere works
	correctly."""
	
func test_name() -> String:
	return "PhysicsBody3D | testing the collision between a CharacterBody sphere and a big static sphere"
	
var tested_body: CharacterBody3D
var static_body: StaticBody3D

func test_start() -> void:
	tested_body = $Tested
	static_body = $Static
	
	var test_lambda: Callable = func(_p_target, p_monitor: GenericManualMonitor):
		# Get the test block and index
		var test_block := int(p_monitor.frame / 100.0)
		var test_index := int(p_monitor.frame % 100)

		# Get the test raster positions X/Y/Theta
		var r1 : float = floor(test_index / 10.0) - 5.0
		var r2 : float = (test_index % 10) - 5.0
		var rt : float = test_index * TAU / 100.0

		# Pick the start position and move direction
		var start : Vector3
		var move : Vector3
		match test_block:
			0:		# Collide from X+ direction
				start = Vector3(103, r1, r2)
				move = Vector3(-10, 0, 0)

			1:		# Collide from X- direction
				start = Vector3(-103, r1, r2)
				move = Vector3(10, 0, 0)

			2:		# Collide from Y+ direction
				start = Vector3(r1, 103, r2)
				move = Vector3(0, -10, 0)

			3:		# Collide from Y- direction
				start = Vector3(r1, -103, r2)
				move = Vector3(0, 10, 0)

			4:		# Collide from Z+ direction
				start = Vector3(r1, r2, 103)
				move = Vector3(0, 0, -10)

			5:		# Collide from Z- direction
				start = Vector3(r1, r2, -103)
				move = Vector3(0, 0, 10)
			
			6:		# Collide from Y/Z orbit
				start = Vector3(0, sin(rt) * 103, cos(rt) * 103)
				move = Vector3(0, sin(rt) * -10, cos(rt) * -10)

			7:		# Collide from X/Z orbit
				start = Vector3(sin(rt) * 103, 0, cos(rt) * 103)
				move = Vector3(sin(rt) * -10, 0, cos(rt) * -10)

			8:		# Collide from X/Y orbit
				start = Vector3(sin(rt) * 103, cos(rt) * 103, 0)
				move = Vector3(sin(rt) * -10, cos(rt) * -10, 0)

			_:		# End of test
				p_monitor.passed()
				return

		# Set the position and perform the move_and_collide
		tested_body.global_position = start
		var collision : KinematicCollision3D = tested_body.move_and_collide(move)
		if !collision:
			p_monitor.failed()
			return

		# Check the collision is on ths sphere
		var position_actual := collision.get_position()
		var distance := position_actual.length()
		if abs(distance - 100) > POSITION_TOLERANCE:
			p_monitor.failed()
			return

		# Verify collision normal matches expected
		var normal_expected := position_actual.normalized()
		var normal_actual := collision.get_normal()
		var normal_error := normal_expected.distance_to(normal_actual)
		if normal_error > NORMAL_TOLERANCE:
			p_monitor.failed()
			return

	var collision_monitor := create_generic_manual_monitor(self, test_lambda, simulation_duration)
	collision_monitor.test_name = "move_and_collide for sphere and sphere are detected correctly"
