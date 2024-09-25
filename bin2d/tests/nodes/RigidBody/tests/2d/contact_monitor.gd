extends PhysicsUnitTest2D

@export var body_shape: PhysicsTest2D.TestCollisionShape = TestCollisionShape.CIRCLE
var simulation_duration := 4
var speed := 50000

func test_description() -> String:
	return """Checks if the contact is correctly reported when [contact_monitor] is turn ON"""
	
func test_name() -> String:
	return "RigidBody2D | testing [contact_monitor]"
	
func test_start() -> void:
	var wall_top = PhysicsTest2D.get_static_body_with_collision_shape(Rect2(Vector2(0,0), Vector2(2, Global.WINDOW_SIZE.y/2)), PhysicsTest2D.TestCollisionShape.RECTANGLE, true)
	wall_top.position = CENTER + Vector2(100, 0)
	wall_top.rotate(deg_to_rad(45))
	wall_top.set_collision_layer_value(1, true)
	wall_top.set_collision_layer_value(2, true)
	
	var wall_bot = PhysicsTest2D.get_static_body_with_collision_shape(Rect2(Vector2(0,0), Vector2(2, Global.WINDOW_SIZE.y/2)), PhysicsTest2D.TestCollisionShape.RECTANGLE, true)
	wall_bot.position = CENTER + Vector2(100, 0)
	wall_bot.rotate(deg_to_rad(135))
	wall_bot.set_collision_layer_value(1, true)
	wall_bot.set_collision_layer_value(2, true)
	
	add_child(wall_bot)
	add_child(wall_top)

	var body := create_rigid_body(1)
	body.position = CENTER - Vector2(100, 0)
	body.linear_damp = 1
	var contact_lambda = func(p_step: int, _p_target: PhysicsTest2D, _p_monitor: GenericStepMonitor):
		if p_step == 0:
			return body.get_colliding_bodies().size() == 0 
		elif p_step == 1:
			return body.get_colliding_bodies().size() > 0 or not body.sleeping
		elif p_step == 2:
			return body.sleeping

	var contact_monitor := create_generic_step_monitor(self, contact_lambda, null, simulation_duration)
	contact_monitor.add_test(1, "Collisions are reported")
	contact_monitor.add_test(2, "The body sleep")

	var body_no_contact := create_rigid_body(2, 0)
	body_no_contact.position = CENTER - Vector2(100, 0)
	body_no_contact.linear_damp = 1

	var lambda_no_contact = func(p_body, p_monitor: GenericManualMonitor):
		if p_body.get_colliding_bodies().size() != 0:
			p_monitor.failed("%d collision(s) detected" % [p_body.get_colliding_bodies().size()])

	var no_contact_monitor := create_generic_manual_monitor(body_no_contact, lambda_no_contact, simulation_duration, false)
	no_contact_monitor.test_name = "No contact reported when [contacts_reported] is 0"

func create_rigid_body(p_layer := 1, p_report_contact := 20) -> RigidBody2D:
	var player := RigidBody2D.new()
	player.add_child(PhysicsTest2D.get_default_collision_shape(body_shape))
	player.gravity_scale = 0
	player.contact_monitor = true
	player.max_contacts_reported = p_report_contact
	player.collision_mask = 0
	player.collision_layer = 0
	player.set_collision_mask_value(p_layer, true)
	add_child(player)
	player.apply_force(Vector2(speed, 0))
	return player
