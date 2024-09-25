extends PhysicsUnitTest2D

@export var body_shape: PhysicsTest2D.TestCollisionShape = TestCollisionShape.CAPSULE
var speed := 55
var simulation_duration := 1

func test_description() -> String:
	return """Checks if [One Way Collision] works properly with RigidBody2D,
	it must only collide on the edge that face up (relative to CollisionPolygon2D's rotation).
	In this test, the platform should only collide when the angle is > 180°.
	"""
	
func test_name() -> String:
	return "CollisionShape2D | testing [One Way Collision] with RigidBody2D"

var bodies = []
var platforms_center_pos = []
var platforms = []
var labels = []
	
func test_start() -> void:
	var offset_x = (Global.WINDOW_SIZE.x - 57) / 29.0 # 30 columns
	var offset_y = (Global.WINDOW_SIZE.y - 72) / 11.0  # 12 rows - 72 + 528
	var deg := 0
	for y in range(12):
		for x in range(30):
			var wall_on_way = PhysicsTest2D.get_static_body_with_collision_shape(Rect2(Vector2(0,0), Vector2(20,20)), TestCollisionShape.RECTANGLE)
			var center = Vector2(28 + offset_x * x , 36 + offset_y * y)
			platforms_center_pos.append(center)
			wall_on_way.position = center
			var collision_shape: CollisionShape2D = wall_on_way.get_child(0)
			collision_shape.one_way_collision = true
			wall_on_way.rotate(deg_to_rad(deg))
			wall_on_way.set_collision_layer_value(1, true)
			wall_on_way.set_collision_layer_value(2, true)
			add_child(wall_on_way)
			platforms.append(wall_on_way)
			
			var label := Label.new()
			label.text = "%d°" % [deg]
			label.position = center + Vector2(0, -28)
			label.set("theme_override_font_sizes/font_size", 8)
			add_child(label)
			labels.append(label)
			
			var rigid = RigidBody2D.new()
			rigid.add_child(PhysicsTest2D.get_default_collision_shape(body_shape, 0.5))
			rigid.gravity_scale = 0
			rigid.position = center + Vector2(-20, 0)
			bodies.append(rigid)
			add_child(rigid)
			rigid.add_constant_force(Vector2(speed, 0))
			
			wall_on_way.collision_layer = 0
			wall_on_way.collision_mask = 0
			wall_on_way.set_collision_layer_value(x + 1, true)
			
			rigid.collision_layer = 0
			rigid.collision_mask = 0
			rigid.set_collision_mask_value(x + 1, true)
			
			deg += 1
	
	var lambda: Callable = func(_p_target: PhysicsUnitTest2D, p_monitor: GenericExpirationMonitor):
		var error_cpt := 0
		for body in bodies as Array[RigidBody2D]:
			var idx = bodies.find(body)
			var should_collide = fposmod(idx, 360) > 180
			var collided = body.position.x < platforms_center_pos[idx].x
		
			if(should_collide == collided):
				remove_child(bodies[idx])
				remove_child(platforms[idx])
				remove_child(labels[idx])
			else:
				labels[idx].set("theme_override_colors/font_color", Color.RED)
				bodies[idx].freeze = true
				error_cpt += 1

		if error_cpt == 0:
			return true
		p_monitor.error_message = "%d angles failed" % [error_cpt]
		return false
		
	var monitor := create_generic_expiration_monitor(self, lambda, null, simulation_duration)
	monitor.test_name = "Only collide if the platform rotation > 180°"
	monitor.expected_to_fail = true
