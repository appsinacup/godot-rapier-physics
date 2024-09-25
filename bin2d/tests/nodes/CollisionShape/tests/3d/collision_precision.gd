extends PhysicsUnitTest3D
enum TestType {
	LEFT_TO_RIGHT,
	RIGHT_TO_LEFT,
	UP_TO_BOTTOM,
	BOTTOM_TO_UP,
	DIAGONAL_TOP_LEFT,
	DIAGONAL_BOTTOM_LEFT,
	DIAGONAL_TOP_RIGHT,
	DIAGONAL_BOTTOM_RIGHT,
	BOX_SEGMENT_TO_SEGMENT,
	BOX_SEGMENT_TO_FACE,
	BOX_VERTEX_TO_FACE,
	BOX_VERTEX_TO_VERTEX,
	LEFT_TO_RIGHT_GIANT_SPHERE
}

@export var shape_tested: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var static_shape: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.BOX
@export var type: TestType = TestType.LEFT_TO_RIGHT
@export var control_value := 0.0

var simulation_duration := 4

func test_description() -> String:
	return """Checks if the collision result returned by the convex algorithm
	if on a collision with the reference, the same shape in primitive.
	"""
	
func test_name() -> String:
	return "CollisionShape3D | testing precision moving [%s] on static [%s] mouvement [%s]" % [PhysicsUnitTest3D.shape_name(shape_tested).to_upper(), PhysicsUnitTest3D.shape_name(static_shape).to_upper(), TestType.keys()[type]]

var tested_body: CharacterBody3D
var reference_body: CharacterBody3D
var static_body: StaticBody3D

var tested_done := false
var tested_result := []
var ref_done := false
var ref_result := []

func test_start() -> void:
	tested_body = create_body(1, shape_tested)
	if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON:
		reference_body = create_body(2, PhysicsTest3D.TestCollisionShape.BOX)
	elif (shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX or shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX):
		reference_body = create_body(2, PhysicsTest3D.TestCollisionShape.SPHERE)
	else:
		@warning_ignore("assert_always_false")
		assert(false)
	# static body
	static_body = StaticBody3D.new()
	static_body.set_collision_layer_value(1, true)
	static_body.set_collision_layer_value(2, true)
	static_body.set_collision_mask_value(1, true)
	static_body.set_collision_mask_value(2, true)
	if type == TestType.LEFT_TO_RIGHT_GIANT_SPHERE:
		var giant_sphere_col := CollisionShape3D.new()
		var giant_sphere_shape := SphereShape3D.new()
		giant_sphere_shape.radius = 500
		giant_sphere_col.shape = giant_sphere_shape
		static_body.add_child(giant_sphere_col)
		static_body.position.x = 500
	else:
		static_body.add_child(PhysicsTest3D.get_default_collision_shape(static_shape))
	add_child(static_body)

	for body in [tested_body, reference_body]:
		if type == TestType.LEFT_TO_RIGHT or type == TestType.LEFT_TO_RIGHT_GIANT_SPHERE:
			body.position = Vector3(-2, 0 ,0)
			body.velocity = Vector3(2, 0 ,0)
		elif type == TestType.RIGHT_TO_LEFT:
			body.position = Vector3(2, 0 ,0)
			body.velocity = Vector3(-2, 0 ,0)
		elif type == TestType.UP_TO_BOTTOM:
			body.position = Vector3(0, 2 ,0)
			body.velocity = Vector3(0, -2 ,0)
		elif type == TestType.BOTTOM_TO_UP:
			body.position = Vector3(0, -2 ,0)
			body.velocity = Vector3(0, 2 ,0)
		elif type == TestType.DIAGONAL_TOP_LEFT:
			body.position = Vector3(-2, 2 ,0)
			body.velocity = Vector3(2, -2 ,0)
		elif type == TestType.DIAGONAL_BOTTOM_LEFT:
			body.position = Vector3(-2, -2 ,0)
			body.velocity = Vector3(2, 2 ,0)
		elif type == TestType.DIAGONAL_TOP_RIGHT:
			body.position = Vector3(2, 2 ,0)
			body.velocity = Vector3(-2, -2 ,0)
		elif type == TestType.DIAGONAL_BOTTOM_RIGHT:
			body.position = Vector3(1.2, -1.2 ,0)
			body.velocity = Vector3(-2, 2, 0)
		elif type == TestType.BOX_SEGMENT_TO_FACE:
			body.position = Vector3(-2, 0 ,0)
			body.velocity = Vector3(2, 0 ,0)
			body.rotation = Vector3(0, deg_to_rad(45), 0)
		elif type == TestType.BOX_VERTEX_TO_FACE:
			body.position = Vector3(-2, 0 ,0)
			body.velocity = Vector3(2, 0 ,0)
			body.rotation = Vector3(deg_to_rad(45), deg_to_rad(45), 0)
		elif type == TestType.BOX_SEGMENT_TO_SEGMENT:
			body.position = Vector3(-2, 0 ,0)
			body.velocity = Vector3(2, 0 ,0)
			body.rotation = Vector3(0, deg_to_rad(45), 0)
			static_body.rotation = Vector3(0, deg_to_rad(45), 0)
		elif type == TestType.BOX_VERTEX_TO_VERTEX:
			body.position = Vector3(-2, 0 ,0)
			body.velocity = Vector3(2, 0 ,0)
			body.rotation = Vector3(deg_to_rad(45), deg_to_rad(-45), deg_to_rad(-90))
			static_body.rotation = Vector3(deg_to_rad(45), deg_to_rad(45), 0)

	var maximum_bodies_supported := func(_p_target: PhysicsUnitTest3D, p_monitor: GenericManualMonitor):
		if tested_done and ref_done:
			
			$Draw.normal = tested_result[1]
			$Draw.point = tested_result[0]
			$Draw.ref_normal = ref_result[1]
			$Draw.ref_point = ref_result[0]
			$Draw.camera = $Camera

			var normal_dot: float = ref_result[1].dot(tested_result[1])
			
			if Global.DEBUG:	
				var pos_diff: Vector3 = ref_result[0] - tested_result[0]
				output += "[indent][indent][color=purple]Position obtained: %v, expected %v, diff %v (length %f)[/color][/indent][/indent]\n" % [tested_result[0], ref_result[0], pos_diff, pos_diff.length()]
				output += "[indent][indent][color=purple]Normal obtained: %v, expected %v, dot %f,[/color][/indent][/indent]\n" % [tested_result[1], ref_result[1], normal_dot]
			
			var failed := false
			var expected_dot := get_dot_reference()
			var perfect_expected = expected_dot == 1.0
			var difference = abs(normal_dot - expected_dot)
			if (perfect_expected and not is_zero_approx(difference)) or (not perfect_expected and difference > 0.001):
				if normal_dot > expected_dot:
					output += "[indent][indent][color=purple]Normal obtained: %v, expected %v, dot %f, ref dot %f[/color][/indent][/indent]\n" % [tested_result[1], ref_result[1], normal_dot, expected_dot]
					output += "[indent][indent][color=green]Normal improvement[/color][/indent][/indent]\n"
				elif normal_dot < expected_dot:
					output += "[indent][indent][color=purple]Normal obtained: %v, expected %v, dot %f, ref dot %f[/color][/indent][/indent]\n" % [tested_result[1], ref_result[1], normal_dot, expected_dot]
					output += "[indent][indent][color=red]Normal regression[/color][/indent][/indent]\n"
					p_monitor.failed()
					return
			p_monitor.passed()
			return

		var collide: KinematicCollision3D = tested_body.move_and_collide(tested_body.velocity * 0.008)
		if collide:
			tested_result.append(collide.get_position())
			tested_result.append(collide.get_normal())
			tested_body.velocity = Vector3.ZERO
			tested_done = true
			
		var collide_ref: KinematicCollision3D = reference_body.move_and_collide(reference_body.velocity * 0.008)
		if collide_ref:
			ref_result.append(collide_ref.get_position())
			ref_result.append(collide_ref.get_normal())
			reference_body.velocity = Vector3.ZERO
			ref_done = true
	
	var check_max_stability_monitor := create_generic_manual_monitor(self, maximum_bodies_supported, simulation_duration)
	check_max_stability_monitor.test_name = "Testing stability"

func create_body(p_layer: int, p_shape: PhysicsTest3D.TestCollisionShape):
	var _body := CharacterBody3D.new()
	var _shape = PhysicsTest3D.get_default_collision_shape(p_shape)
	_body.add_child(_shape)
	for i in range(1, 17):
		_body.set_collision_layer_value(i, false)
		_body.set_collision_mask_value(i, false)
	_body.set_collision_layer_value(p_layer, true)
	_body.set_collision_mask_value(p_layer, true)
	add_child(_body)
	return _body

func get_dot_reference() -> float:
	
	if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON and static_shape == PhysicsTest3D.TestCollisionShape.BOX:
		return 1.0
	
	if type == 	TestType.LEFT_TO_RIGHT:
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 1.0
		
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 1.0
			
	if type == 	TestType.DIAGONAL_TOP_LEFT:
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 0.999
		
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 0.999
			
	if type == 	TestType.UP_TO_BOTTOM:
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 1.000000
		
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 0.999
	
	if type == 	TestType.LEFT_TO_RIGHT_GIANT_SPHERE:
		if shape_tested == PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX and static_shape == PhysicsTest3D.TestCollisionShape.SPHERE:
			return 1.000000
	
	print("No reference for %s vs %s with type %s" % [PhysicsUnitTest3D.shape_name(shape_tested), PhysicsUnitTest3D.shape_name(static_shape),TestType.keys()[type]])
	@warning_ignore("assert_always_false")
	assert(false)
	return 100.0 
