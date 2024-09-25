extends PhysicsUnitTest2D

enum WallType {
	BOX,
	CONCAVE_BOX
}

@export var wall_type: WallType = WallType.BOX
@export var speed: float = 25000
@export var simulation_duration : float = 0.1

func test_description() -> String:
	return """Checks if the Continuous Collision Detection (CCD) is working, it must ensure that moving
	objects does not go through objects (tunnelling).
	"""
	
func test_name() -> String:
	return "RigidBody2D | testing Continuous Collision Detection (CCD)"

var detect_x_collision := false
var detect_y_collision := false

func test_start() -> void:
	var vertical_wall := StaticBody2D.new()
	var vertical_wall_shape: Node2D = null
	if wall_type == WallType.BOX:
		vertical_wall_shape = PhysicsTest2D.get_collision_shape(Rect2(Vector2(0,0), Vector2(2, Global.WINDOW_SIZE.y)), PhysicsTest2D.TestCollisionShape.RECTANGLE, true)
	elif wall_type == WallType.CONCAVE_BOX:
		vertical_wall_shape = PhysicsTest2D.get_collision_shape(PhysicsTest2D.get_box_segments(2, Global.WINDOW_SIZE.y), PhysicsTest2D.TestCollisionShape.CONCAVE_POLYGON)
		vertical_wall_shape.position += Vector2(1, 0.5 * Global.WINDOW_SIZE.y)
	vertical_wall.add_child(vertical_wall_shape)
	vertical_wall.position = Vector2(TOP_RIGHT.x - Global.WINDOW_SIZE.y/2 -1, 0)
	add_child(vertical_wall)

	var horizontal_wall := StaticBody2D.new()
	var horizontal_wall_shape: Node2D = null
	if wall_type == WallType.BOX:
		horizontal_wall_shape = PhysicsTest2D.get_collision_shape(Rect2(Vector2(0,0), Vector2(0.5 * Global.WINDOW_SIZE.y, 2)), PhysicsTest2D.TestCollisionShape.RECTANGLE, true)
	elif wall_type == WallType.CONCAVE_BOX:
		horizontal_wall_shape = PhysicsTest2D.get_collision_shape(PhysicsTest2D.get_box_segments(0.5 * Global.WINDOW_SIZE.y, 2), PhysicsTest2D.TestCollisionShape.CONCAVE_POLYGON)
		horizontal_wall_shape.position += Vector2(0.25 * Global.WINDOW_SIZE.y, 1)
	horizontal_wall.add_child(horizontal_wall_shape)
	horizontal_wall.position = Vector2(BOTTOM_RIGHT.x - Global.WINDOW_SIZE.y/2, BOTTOM_RIGHT.y -50)
	add_child(horizontal_wall)

	var rigid_x_ccd_ray := create_rigid_body(RigidBody2D.CCD_MODE_CAST_RAY)
	rigid_x_ccd_ray.position = Vector2(50, 150)
	rigid_x_ccd_ray.body_entered.connect(x_collide.bind(rigid_x_ccd_ray))
	
	var rigid_x_ccd_shape := create_rigid_body(RigidBody2D.CCD_MODE_CAST_SHAPE)
	rigid_x_ccd_shape.position = Vector2(50, 250)
	
	var rigid_y_ccd_ray := create_rigid_body(RigidBody2D.CCD_MODE_CAST_RAY, false)
	rigid_y_ccd_ray.position = Vector2(vertical_wall.position.x + 150, 50)
	rigid_y_ccd_ray.body_entered.connect(y_collide.bind(rigid_y_ccd_ray))
	
	var rigid_y_ccd_shape := create_rigid_body(RigidBody2D.CCD_MODE_CAST_SHAPE, false)
	rigid_y_ccd_shape.position = Vector2(vertical_wall.position.x + 250, 50)
	
	var x_lambda = func(p_target: RigidBody2D, _p_monitor: GenericExpirationMonitor):
		return p_target.position.x <= vertical_wall.position.x # good

	var y_lambda = func(p_target: RigidBody2D, _p_monitor: GenericExpirationMonitor):
		return p_target.position.y <= horizontal_wall.position.y # good
	
	var collide_x_lambda = func(_p_target: RigidBody2D, _p_monitor: GenericExpirationMonitor):
		return detect_x_collision

	var collide_y_lambda = func(_p_target: RigidBody2D, _p_monitor: GenericExpirationMonitor):
		return detect_y_collision

	var x_ray_ccd_monitor := create_generic_expiration_monitor(rigid_x_ccd_ray, x_lambda, null, simulation_duration)
	x_ray_ccd_monitor.test_name = "Rigid moving in x with CCD Ray does not pass through the wall"
	x_ray_ccd_monitor.expected_to_fail = true
	
	var x_shape_ccd_monitor := create_generic_expiration_monitor(rigid_x_ccd_shape, x_lambda, null, simulation_duration)
	x_shape_ccd_monitor.test_name = "Rigid moving in x with CCD Cast shape does not pass through the wall"
	x_shape_ccd_monitor.expected_to_fail = true

	var y_ray_ccd_monitor := create_generic_expiration_monitor(rigid_y_ccd_ray, y_lambda, null, simulation_duration)
	y_ray_ccd_monitor.test_name = "Rigid moving in y with CCD Ray does not pass through the wall"
	
	var y_shape_ccd_monitor := create_generic_expiration_monitor(rigid_y_ccd_shape, y_lambda, null, simulation_duration)
	y_shape_ccd_monitor.test_name = "Rigid moving in y with CCD Cast shape does not pass through the wall"
	y_shape_ccd_monitor.expected_to_fail = true
	
	var x_collision_monitor := create_generic_expiration_monitor(rigid_x_ccd_ray, collide_x_lambda, null, simulation_duration)
	x_collision_monitor.test_name = "Rigid moving in x with CCD detects collision"

	var y_collision_monitor := create_generic_expiration_monitor(rigid_y_ccd_ray, collide_y_lambda, null, simulation_duration)
	y_collision_monitor.test_name = "Rigid moving in y with CCD detects collision"
	
	process_mode = Node.PROCESS_MODE_DISABLED # to be able to see something
	await get_tree().create_timer(.5).timeout
	process_mode = Node.PROCESS_MODE_INHERIT

func create_rigid_body(p_ccd_mode: RigidBody2D.CCDMode, p_horizontal := true, p_shape: PhysicsTest2D.TestCollisionShape = TestCollisionShape.RECTANGLE) -> RigidBody2D:
	var player = RigidBody2D.new()
	player.add_child(PhysicsTest2D.get_default_collision_shape(p_shape))
	player.gravity_scale = 0
	player.continuous_cd = p_ccd_mode
	player.contact_monitor = true
	player.max_contacts_reported = 2
	player.rotation = 90 # Case where the movement vector was not properly being transformed into local space, see #69934
	var force = Vector2(speed, 0) if p_horizontal else Vector2(0, speed)
	add_child(player)
	player.apply_central_impulse(force)
	return player

func x_collide(_body, _player):
	detect_x_collision = true
	
func y_collide(_body, _player):
	detect_y_collision = true
