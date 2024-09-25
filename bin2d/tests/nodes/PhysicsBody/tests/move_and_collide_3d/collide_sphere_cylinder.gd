extends PhysicsUnitTest3D

# Tolerances
const POSITION_TOLERANCE := 0.01
const NORMAL_TOLERANCE := 0.005

var simulation_duration := 20
var test_failed := false

func test_description() -> String:
	return """Checks whether the move_and_collide between a CharacterBody sphere and a big static cylinder works
	correctly."""
	
func test_name() -> String:
	return "PhysicsBody3D | testing the collision between a CharacterBody sphere and a big static cylinder"

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
		var position_expected : Vector3
		var normal_expected : Vector3
		match test_block:
			0:		# Collide from X+ direction
				start = Vector3(103, r1, r2)
				move = Vector3(-10, 0, 0)
				position_expected = Vector3(100, r1, r2)
				normal_expected = Vector3(1, 0, 0)

			1:		# Collide from X- direction
				start = Vector3(-103, r1, r2)
				move = Vector3(10, 0, 0)
				position_expected = Vector3(-100, r1, r2)
				normal_expected = Vector3(-1, 0, 0)

			2:		# Collide from Y+ direction
				start = Vector3(r1, 53, r2)
				move = Vector3(0, -10, 0)

			3:		# Collide from Y- direction
				start = Vector3(r1, -53, r2)
				move = Vector3(0, 10, 0)

			4:		# Collide from Z+ direction
				start = Vector3(r1, r2, 53)
				move = Vector3(0, 0, -10)

			5:		# Collide from Z- direction
				start = Vector3(r1, r2, -53)
				move = Vector3(0, 0, 10)

			6:		# Collide from ring in middle
				start = Vector3(0, sin(rt) * 53, cos(rt) * 53)
				move = Vector3(0, sin(rt) * -10, cos(rt) * -10)
				position_expected = Vector3(0, sin(rt) * 50, cos(rt) * 50)
				normal_expected = Vector3(0, sin(rt) * 1, cos(rt) * 1)

			7:		# Collide from ring near X+ end
				start = Vector3(95, sin(rt) * 53, cos(rt) * 53)
				move = Vector3(0, sin(rt) * -10, cos(rt) * -10)
				position_expected = Vector3(95, sin(rt) * 50, cos(rt) * 50)
				normal_expected = Vector3(0, sin(rt) * 1, cos(rt) * 1)

			8:		# Collide from ring near X- end
				start = Vector3(-95, sin(rt) * 53, cos(rt) * 53)
				move = Vector3(0, sin(rt) * -10, cos(rt) * -10)
				position_expected = Vector3(-95, sin(rt) * 50, cos(rt) * 50)
				normal_expected = Vector3(0, sin(rt) * 1, cos(rt) * 1)
			
			9:		# Collide from X+ ring
				start = Vector3(103, sin(rt) * 53, cos(rt) * 53)
				move = Vector3(-10, sin(rt) * -10, cos(rt) * -10)
				position_expected = Vector3(100, sin(rt) * 50, cos(rt) * 50)
				normal_expected = Vector3(0.7071, sin(rt) * 0.7071, cos(rt) * 0.7071)

			10:		# Collide from X- ring
				start = Vector3(-103, sin(rt) * 53, cos(rt) * 53)
				move = Vector3(10, sin(rt) * -10, cos(rt) * -10)
				position_expected = Vector3(-100, sin(rt) * 50, cos(rt) * 50)
				normal_expected = Vector3(-0.7071, sin(rt) * 0.7071, cos(rt) * 0.7071)

			_:		# End of test
				p_monitor.passed()
				return

		# If expected position or normal are blank then deduce from cross-section collision
		if position_expected.is_zero_approx() or normal_expected.is_zero_approx():
			var c := Geometry3D.segment_intersects_sphere(
				start,
				start + move,
				start * Vector3.RIGHT,
				51)
			if c.size() < 2:
				p_monitor.failed()
				return

			position_expected = c[0] - c[1]
			normal_expected = c[1]

		# Set the position and perform the move_and_collide
		tested_body.global_position = start
		var collision : KinematicCollision3D = tested_body.move_and_collide(move)
		if !collision:
			p_monitor.failed()
			return

		# Check the collision is on ths sphere
		var position_actual := collision.get_position()
		var position_error := position_expected.distance_to(position_actual)
		if position_error > POSITION_TOLERANCE:
			p_monitor.failed()
			return

		# Verify collision normal matches expected
		var normal_actual := collision.get_normal()
		var normal_error := normal_expected.distance_to(normal_actual)
		if normal_error > NORMAL_TOLERANCE:
			p_monitor.failed()
			return

	var collision_monitor := create_generic_manual_monitor(self, test_lambda, simulation_duration)
	collision_monitor.test_name = "move_and_collide for sphere and cylinder are detected correctly"
