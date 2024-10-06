extends PhysicsPerformanceTest3D

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
	BOX_VERTEX_TO_VERTEX
}

@export var shape_tested: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var static_shape: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.BOX
@export var type: TestType = TestType.BOX_VERTEX_TO_VERTEX

var tested_body: CharacterBody3D
var reference_body: CharacterBody3D
var static_body: StaticBody3D

#var ref_body

func test_name() -> String:
	return "Testing precision for %s | type: %s" % [PhysicsTest3D.shape_name(shape_tested), TestType.keys()[type]]
	
func test_start() -> void:
	
	tested_body = create_body(1, shape_tested)
	reference_body = create_body(2, shape_tested)
	
	# static body
	static_body = StaticBody3D.new()
	static_body.set_collision_layer_value(1, true)
	static_body.set_collision_layer_value(2, true)
	static_body.set_collision_mask_value(1, true)
	static_body.set_collision_mask_value(2, true)
	static_body.add_child(PhysicsTest3D.get_default_collision_shape(static_shape))
	add_child(static_body)

	for body in [tested_body, reference_body]:
		if type == TestType.LEFT_TO_RIGHT:
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
			body.position = Vector3(-1, 1 ,0)
			body.velocity = Vector3(1, -1 ,0)
		elif type == TestType.DIAGONAL_BOTTOM_LEFT:
			body.position = Vector3(-1, -1 ,0)
			body.velocity = Vector3(1, 1 ,0)
		elif type == TestType.DIAGONAL_TOP_RIGHT:
			body.position = Vector3(1, 1 ,0)
			body.velocity = Vector3(-1, -1 ,0)
		elif type == TestType.DIAGONAL_BOTTOM_RIGHT:
			body.position = Vector3(1, -1 ,0)
			body.velocity = Vector3(-1, 1 ,0)
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
	super()

var ctp := 0
var tested_done := false
var tested_result := []
var ref_done := false
var ref_result := []

func _physics_process(delta: float) -> void:
	if tested_done and tested_done:
		
		$Draw.normal = tested_result[1]
		$Draw.point = tested_result[0]
		$Draw.camera = $Camera
		
		var pos_diff: Vector3 = tested_result[0] - ref_result[0]
		var normal_diff: Vector3 = tested_result[1] - ref_result[1]
		extra_text.append("Position obtained: %v, expected %v, diff %v, diff length %f" % [tested_result[0], ref_result[0], pos_diff, pos_diff.length()])
		extra_text.append("Normal obtained: %v, expected %v, diff %v, diff length %f " % [tested_result[1], ref_result[1], normal_diff, normal_diff.length()])
		test_completed(1)
		return
	ctp += 1
	var collide: KinematicCollision3D = tested_body.move_and_collide(tested_body.velocity * delta)
	if collide:
		tested_result.append(collide.get_position())
		tested_result.append(collide.get_normal())
		tested_body.velocity = Vector3.ZERO
		tested_done = true
		
	var collide_ref: KinematicCollision3D = reference_body.move_and_collide(reference_body.velocity * delta)
	if collide_ref:
		ref_result.append(collide_ref.get_position())
		ref_result.append(collide_ref.get_normal())
		reference_body.velocity = Vector3.ZERO
		ref_done = true

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
