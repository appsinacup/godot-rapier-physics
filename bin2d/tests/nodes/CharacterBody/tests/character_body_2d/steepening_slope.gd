extends PhysicsUnitTest2D

var simulation_duration := 6
# Frames to walk before judging, comfortably inside the monitor's own lifetime.
const WALK_FRAMES := 300
const SPEED := 300.0
# The body starts on the flat run and walks right into the curve. Before the fix it stopped dead
# at the first vertex where the slope steepens, around x=812.
const START_X := 700.0
const MUST_PASS_X := 880.0

var body: CharacterBody2D
var furthest_x := 0.0
var stalled_frames := 0
var worst_stall := 0

func test_description() -> String:
	return """Checks that a body walking up a slope of increasing steepness does not stick at the
	vertices of the [CollisionPolygon2D], where the face it rests on and the steeper face ahead
	both touch it at once.
	"""

func test_name() -> String:
	return "CharacterBody2D | walking up a slope of increasing steepness"

func test_start() -> void:
	var ground := StaticBody2D.new()
	var poly := CollisionPolygon2D.new()
	poly.polygon = PackedVector2Array([
		Vector2(-5, 652), Vector2(-7, 637), Vector2(644, 637), Vector2(705, 632),
		Vector2(764, 614), Vector2(819, 577), Vector2(865, 523), Vector2(900, 461),
		Vector2(933, 361), Vector2(955, 251), Vector2(965, 146), Vector2(967, 69),
		Vector2(968, 43), Vector2(970, 38), Vector2(974, 33), Vector2(980, 31),
		Vector2(986, 30), Vector2(1160, 30), Vector2(1160, 656)])
	ground.add_child(poly)
	add_child(ground)

	body = CharacterBody2D.new()
	body.floor_max_angle = PI
	var col := CollisionShape2D.new()
	var capsule := CapsuleShape2D.new()
	capsule.radius = 5.0
	capsule.height = 18.0
	col.shape = capsule
	col.rotation = -PI / 2.0
	body.add_child(col)
	body.position = Vector2(START_X, 600)
	add_child(body)
	furthest_x = body.position.x

	var checks = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		body.velocity.x = SPEED
		body.velocity.y += 980.0 * get_physics_process_delta_time()
		body.move_and_slide()

		var x: float = body.global_position.x
		# 300 px/s at 60 Hz is 5 px per frame on the flat; the slope legitimately slows that
		# down, so only a complete halt counts as a stall.
		if x - furthest_x < 0.01:
			stalled_frames += 1
			worst_stall = maxi(worst_stall, stalled_frames)
		else:
			stalled_frames = 0
		furthest_x = maxf(furthest_x, x)

		if p_monitor.frame == WALK_FRAMES:
			p_monitor.add_test("Walks past the vertex where the slope steepens")
			if furthest_x <= MUST_PASS_X:
				p_monitor.add_test_error(
					"stopped at x=%.1f after stalling %d frames" % [furthest_x, worst_stall])
			p_monitor.add_test_result(furthest_x > MUST_PASS_X)

			p_monitor.add_test("Never comes to a complete stop on the way up")
			if worst_stall >= 30:
				p_monitor.add_test_error("stalled for %d frames" % worst_stall)
			p_monitor.add_test_result(worst_stall < 30)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, checks, simulation_duration)
