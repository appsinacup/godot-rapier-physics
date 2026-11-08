extends PhysicsUnitTest3D

const COIN_COUNT := 200
const COINS_PER_RING := 24
const CAPSULE_RADIUS := 0.4
const CAPSULE_HEIGHT := 1.8
const GRAVITY := 9.8
const SETTLE_FRAMES := 10
const CHECKED_FRAMES := 120

var simulation_duration := 4.0
var frames_off_floor := 0

func test_description() -> String:
	return """Checks that a [CharacterBody3D] keeps its floor when many rigid bodies pile up
	around it without touching it.
	"""

func test_name() -> String:
	return "CharacterBody3D | keeps its floor among many nearby bodies"

func test_start() -> void:
	var ground := StaticBody3D.new()
	var ground_shape := CollisionShape3D.new()
	var ground_box := BoxShape3D.new()
	ground_box.size = Vector3(20, 1, 20)
	ground_shape.shape = ground_box
	ground.add_child(ground_shape)
	ground.position = Vector3(0, -0.5, 0)
	add_child(ground)

	var character := CharacterBody3D.new()
	var character_shape := CollisionShape3D.new()
	var capsule := CapsuleShape3D.new()
	capsule.radius = CAPSULE_RADIUS
	capsule.height = CAPSULE_HEIGHT
	character_shape.shape = capsule
	character.add_child(character_shape)
	character.position = Vector3(0, CAPSULE_HEIGHT / 2.0, 0)
	add_child(character)

	var coin_shape := CylinderShape3D.new()
	coin_shape.radius = 0.25
	coin_shape.height = 0.08
	for i in COIN_COUNT:
		var coin := RigidBody3D.new()
		var coin_collision := CollisionShape3D.new()
		coin_collision.shape = coin_shape
		coin.add_child(coin_collision)
		var ring := i / COINS_PER_RING
		var angle := TAU * (i % COINS_PER_RING) / COINS_PER_RING + 0.13 * ring
		var distance := 1.7 + 0.35 * (ring % 4)
		coin.position = Vector3(cos(angle) * distance, 0.3 + 0.2 * ring, sin(angle) * distance)
		add_child(coin)

	var run = func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		character.velocity.y -= GRAVITY * get_physics_process_delta_time()
		character.move_and_slide()
		if p_monitor.frame > SETTLE_FRAMES and not character.is_on_floor():
			frames_off_floor += 1
		if p_monitor.frame == SETTLE_FRAMES + CHECKED_FRAMES:
			p_monitor.add_test("Stays on the floor")
			if frames_off_floor > 0:
				p_monitor.add_test_error("left the floor for %d frames" % frames_off_floor)
			p_monitor.add_test_result(frames_off_floor == 0)
			p_monitor.monitor_completed()

	create_generic_manual_monitor(self, run, simulation_duration)
