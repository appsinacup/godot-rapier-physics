extends PhysicsUnitTest3D

var simulation_duration := 10

# A concave CollisionPolygon3D is decomposed by Godot into convex pieces, and the cuts between
# those pieces are interior to the outline: nothing should ever collide with them. A body sliding
# onto the ramp used to catch on one of those cuts and lose almost all of its speed, because the
# speculative contact margin was wide enough to reach it.

const RAMP_FOOT_X := 5.0
const RAMP_TOP := Vector2(9.0, 3.0)
const LAUNCH_SPEED := 9.0
const SEAM_WINDOW := 1.0
# Meeting the ramp legitimately costs the speed component normal to it, leaving about 38%.
# Catching on a seam leaves under 10%. The bar sits in the gap between the two.
const MIN_RETAINED_SPEED_RATIO := 0.25

var slider: RigidBody3D
var speed_before_seam := 0.0
var slowest_at_seam := INF

func test_description() -> String:
	return """Checks that a body sliding onto a ramp does not catch on the internal cuts Godot
	leaves when it decomposes a concave [CollisionPolygon3D] into convex pieces.
	"""

func test_name() -> String:
	return "CollisionShape3D | sliding across a decomposed polygon seam"

func test_start() -> void:
	var ground := StaticBody3D.new()
	var outline := CollisionPolygon3D.new()
	outline.depth = 6.0
	outline.polygon = PackedVector2Array([
		Vector2(-6, 0), Vector2(RAMP_FOOT_X, 0), RAMP_TOP,
		Vector2(RAMP_TOP.x + 2, RAMP_TOP.y), Vector2(RAMP_TOP.x + 2, -6), Vector2(-6, -6)])
	ground.add_child(outline)
	var ground_material := PhysicsMaterial.new()
	ground_material.friction = 0.0
	ground_material.bounce = 0.0
	ground.physics_material_override = ground_material
	add_child(ground)

	slider = RigidBody3D.new()
	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.84, 0.4, 0.84)
	shape.shape = box
	slider.add_child(shape)
	slider.position = Vector3(-3, 0.21, 0)
	slider.can_sleep = false
	var slider_material := PhysicsMaterial.new()
	slider_material.friction = 0.0
	slider_material.bounce = 0.0
	slider.physics_material_override = slider_material
	add_child(slider)

	var run = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if p_monitor.frame == 20:
			slider.linear_velocity = Vector3(LAUNCH_SPEED, 0, 0)
			slider.angular_velocity = Vector3.ZERO

		if p_monitor.frame > 20:
			var x := slider.global_position.x
			if x < RAMP_FOOT_X - SEAM_WINDOW:
				speed_before_seam = slider.linear_velocity.length()
			elif x < RAMP_FOOT_X + SEAM_WINDOW:
				slowest_at_seam = min(slowest_at_seam, slider.linear_velocity.length())

		if p_monitor.frame == 90:
			p_monitor.add_test("Keeps its speed across the seam at the ramp foot")
			var retained: float = slowest_at_seam / speed_before_seam if speed_before_seam > 0.0 else 0.0
			if retained < MIN_RETAINED_SPEED_RATIO:
				p_monitor.add_test_error("retained only %.0f%% of %.2f m/s" % [retained * 100.0, speed_before_seam])
			p_monitor.add_test_result(retained >= MIN_RETAINED_SPEED_RATIO)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
