extends PhysicsUnitTest2D

var simulation_duration := 10

# A concave CollisionPolygon2D is decomposed by Godot into convex pieces, and the cuts between
# those pieces are interior to the outline: nothing should ever collide with them. A body sliding
# onto the ramp used to catch on one of those cuts and lose almost all of its speed, because the
# speculative contact margin was wide enough to reach it.

const FLAT_Y := 400.0
const RAMP_FOOT := Vector2(500, FLAT_Y)
const RAMP_TOP := Vector2(900, 100)
const LAUNCH_SPEED := 900.0
# Meeting the ramp legitimately costs the speed component normal to it, leaving about 64%.
# Catching on a seam stops the body dead, at 0%. The bar sits in the gap between the two.
const MIN_RETAINED_SPEED_RATIO := 0.35
const SEAM_WINDOW := 100.0

var slider: RigidBody2D
var speed_before_seam := 0.0
var slowest_at_seam := INF

func test_description() -> String:
	return """Checks that a body sliding onto a ramp does not catch on the internal cuts Godot
	leaves when it decomposes a concave [CollisionPolygon2D] into convex pieces.
	"""

func test_name() -> String:
	return "CollisionShape2D | sliding across a decomposed polygon seam"

func test_start() -> void:
	var ground := StaticBody2D.new()
	var outline := CollisionPolygon2D.new()
	outline.polygon = PackedVector2Array([
		Vector2(0, FLAT_Y), RAMP_FOOT, RAMP_TOP,
		Vector2(RAMP_TOP.x + 100, RAMP_TOP.y), Vector2(RAMP_TOP.x + 100, 900), Vector2(0, 900)])
	ground.add_child(outline)
	var ground_material := PhysicsMaterial.new()
	ground_material.friction = 0.0
	ground_material.bounce = 0.0
	ground.physics_material_override = ground_material
	add_child(ground)

	slider = RigidBody2D.new()
	var shape := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = Vector2(84, 40)
	shape.shape = box
	slider.add_child(shape)
	slider.position = Vector2(150, FLAT_Y - 20)
	slider.can_sleep = false
	var slider_material := PhysicsMaterial.new()
	slider_material.friction = 0.0
	slider_material.bounce = 0.0
	slider.physics_material_override = slider_material
	add_child(slider)

	var run = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 20:
			slider.linear_velocity = Vector2(LAUNCH_SPEED, 0)
			slider.angular_velocity = 0.0

		if p_monitor.frame > 20:
			var x := slider.global_position.x
			if x < RAMP_FOOT.x - SEAM_WINDOW:
				speed_before_seam = slider.linear_velocity.length()
			elif x < RAMP_FOOT.x + SEAM_WINDOW:
				slowest_at_seam = min(slowest_at_seam, slider.linear_velocity.length())

		if p_monitor.frame == 90:
			if true:
				p_monitor.add_test("Keeps its speed across the seam at the ramp foot")
				var retained := slowest_at_seam / speed_before_seam if speed_before_seam > 0.0 else 0.0
				if retained < MIN_RETAINED_SPEED_RATIO:
					p_monitor.add_test_error("retained only %.0f%% of %.0f px/s" % [retained * 100.0, speed_before_seam])
				p_monitor.add_test_result(retained >= MIN_RETAINED_SPEED_RATIO)

			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
