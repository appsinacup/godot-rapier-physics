class_name PhysicsTest2D
extends Node2D

signal completed

var CENTER := Global.WINDOW_SIZE/2
var CENTER_LEFT := Vector2(0, CENTER.y)
var CENTER_RIGHT := Vector2(Global.WINDOW_SIZE.x, CENTER.y)

var TOP_LEFT := Vector2(0,0)
var TOP_CENTER := Vector2(CENTER.x, 0)
var TOP_RIGHT := Vector2(Global.WINDOW_SIZE.x, 0)

var BOTTOM_LEFT := Vector2(0, Global.WINDOW_SIZE.y)
var BOTTOM_CENTER := Vector2(CENTER.x, Global.WINDOW_SIZE.y)
var BOTTOM_RIGHT := Vector2(Global.WINDOW_SIZE.x, Global.WINDOW_SIZE.y)

var output := ""

@export_flags_2d_physics var collision_layer: int

enum TestCollisionShape {
	CAPSULE = PhysicsServer2D.SHAPE_CAPSULE,
	CONCAVE_POLYGON = PhysicsServer2D.SHAPE_CONCAVE_POLYGON,
	CONVEX_POLYGON = PhysicsServer2D.SHAPE_CONVEX_POLYGON,
	RECTANGLE = PhysicsServer2D.SHAPE_RECTANGLE,
	CIRCLE = PhysicsServer2D.SHAPE_CIRCLE,
	WORLD_BOUNDARY = PhysicsServer2D.SHAPE_WORLD_BOUNDARY,
	CONCAVE_SEGMENT = 100
}

func _ready() -> void:
	if get_tree().get_root() == get_parent(): # autostart is the scene is alone
		Global.print_engine()
		test_start()
	var center_layout = CenterContainer.new()
	center_layout.size = Vector2(Global.WINDOW_SIZE.x, 40)
	var text_label = Label.new()
	text_label.text = test_name()
	text_label.set("theme_override_font_sizes/font_size", 20)
	center_layout.add_child(text_label)
	add_child(center_layout)

func test_name() -> String:
	@warning_ignore("assert_always_false")
	assert(false, "ERROR: You must give implement test_name()")
	return ""

func test_start() -> void:
	pass

func test_completed() -> void:
	Global.NB_TESTS_COMPLETED += 1
	output += "[indent] %d. [b]%s[/b][/indent]\n" % [Global.NB_TESTS_COMPLETED, test_name()]

func test_description() -> String:
	return ""

func add_collision_bottom(p_width:= 20, p_layers := [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]):
	var surfaces: Array[StaticBody2D]= []
	# Bottom Wall
	surfaces.append(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(BOTTOM_LEFT - Vector2(10000,p_width), Vector2(20000, p_width)), TestCollisionShape.RECTANGLE, true))
	
	for wall in surfaces:
		wall.collision_layer = 0
		wall.collision_mask = 0
		for layer in p_layers:
			wall.set_collision_layer_value(layer, true)
			wall.set_collision_mask_value(layer, true)
		add_child(wall)

func add_collision_boundaries(p_width:= 20, p_add_ceiling := true,  p_layers := [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]):
	var surfaces: Array[StaticBody2D]= []
	# Left wall
	surfaces.append(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(TOP_LEFT, Vector2(p_width, Global.WINDOW_SIZE.y)), TestCollisionShape.RECTANGLE, true))
	# Right wall
	surfaces.append(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(TOP_RIGHT - Vector2(p_width,0), Vector2(p_width, Global.WINDOW_SIZE.y)), TestCollisionShape.RECTANGLE, true))
	# Bottom Wall
	surfaces.append(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(BOTTOM_LEFT - Vector2(0,p_width), Vector2(Global.WINDOW_SIZE.x, p_width)), TestCollisionShape.RECTANGLE, true))
	if p_add_ceiling:
		surfaces.append(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(TOP_LEFT, Vector2(Global.WINDOW_SIZE.x, p_width)), TestCollisionShape.RECTANGLE, true))
	
	for wall in surfaces:
		wall.collision_layer = 0
		wall.collision_mask = 0
		for layer in p_layers:
			wall.set_collision_layer_value(layer, true)
			wall.set_collision_mask_value(layer, true)
		add_child(wall)

static func get_static_body_with_collision_shape(p_shape_definition, p_shape_type := TestCollisionShape.RECTANGLE, p_top_left_position := false) -> StaticBody2D:
	var body = StaticBody2D.new()
	body.position = Vector2(0, 0)
	var body_col = get_collision_shape(p_shape_definition, p_shape_type, p_top_left_position)
	body.add_child(body_col)
	return body
	
# Get CollisionShape2D or CollisionPolygon2D that fits in the rectangle.
static func get_collision_shape(p_shape_definition, p_shape_type := TestCollisionShape.RECTANGLE, p_top_left_position := false) -> Node2D:
	if p_top_left_position and not p_shape_type in [TestCollisionShape.CIRCLE, TestCollisionShape.RECTANGLE]:
		@warning_ignore("assert_always_false")
		assert(false, "Top left position not supported for this shape")
	var col = null
	if p_shape_type == TestCollisionShape.CAPSULE:
		assert(p_shape_definition is Vector2, "Shape definition for a Capsule must be a Vector2")
		col = CollisionShape2D.new()
		col.shape = CapsuleShape2D.new()
		col.shape.radius = p_shape_definition.x
		col.shape.height = p_shape_definition.y
	elif p_shape_type == TestCollisionShape.CIRCLE:
		assert(p_shape_definition is float, "Shape definition for a Circle must be a float")
		col = CollisionShape2D.new()
		col.shape = CircleShape2D.new()
		col.shape.radius = p_shape_definition
		if p_top_left_position:
			col.position = p_shape_definition.position + Vector2(p_shape_definition, p_shape_definition)
	elif p_shape_type == TestCollisionShape.CONCAVE_POLYGON:
		assert(p_shape_definition is PackedVector2Array, "Shape definition for a Concave Polygon must be a PackedVector2Array")
		col = CollisionPolygon2D.new()
		col.polygon = p_shape_definition
	elif p_shape_type == TestCollisionShape.CONCAVE_SEGMENT:
		assert(p_shape_definition is PackedVector2Array, "Shape definition for a Concave Polygon must be a PackedVector2Array")
		col = CollisionShape2D.new()
		col.shape = ConcavePolygonShape2D.new()
		col.shape.segments = p_shape_definition
	elif p_shape_type == TestCollisionShape.CONVEX_POLYGON:
		assert(p_shape_definition is PackedVector2Array, "Shape definition for a Concave Polygon must be a PackedVector2Array")
		col = CollisionPolygon2D.new()
		col.polygon = p_shape_definition
	elif p_shape_type == TestCollisionShape.RECTANGLE:
		assert(p_shape_definition is Rect2, "Shape definition for a Rectangle must be a Rect2")
		col = CollisionShape2D.new()
		col.shape = RectangleShape2D.new()
		col.shape.size = p_shape_definition.size
		if p_top_left_position:
			col.position = p_shape_definition.position + 0.5 * p_shape_definition.size
	return col

static func get_default_collision_shape(p_shape_type : TestCollisionShape, p_scale := 1.0) -> Node2D:
	return get_collision_shape(get_default_shape_definition(p_shape_type, p_scale), p_shape_type)
	
static func get_default_shape_definition(p_shape_type : TestCollisionShape, p_scale := 1.0):
	if p_shape_type == TestCollisionShape.RECTANGLE:
		return Rect2(0, 0, 25 * p_scale, 25 * p_scale)
	if p_shape_type == TestCollisionShape.CIRCLE:
		return 12.5 * p_scale
	if p_shape_type == TestCollisionShape.CAPSULE:
		return Vector2(8,25) * p_scale
	if p_shape_type == TestCollisionShape.CONCAVE_POLYGON:
		var concave: PackedVector2Array = []
		for v in concave_array():
			concave.append(v * p_scale)
		return concave
	if p_shape_type == TestCollisionShape.CONCAVE_SEGMENT:
		var concave: PackedVector2Array = []
		for v in concave_array():
			concave.append(v * p_scale)
			if v != concave_array()[0]:
				concave.append(v * p_scale)
		concave.append(concave_array()[0] * p_scale)
		return concave
	if p_shape_type == TestCollisionShape.CONVEX_POLYGON:
		var convex: PackedVector2Array = []
		for v in convex_array():
			convex.append(v * p_scale)
		return convex
	
	@warning_ignore("assert_always_false")
	assert(false, "No default shape for this shape type")

static func shape_name(p_shape_type : TestCollisionShape) -> String:
	match p_shape_type:
		TestCollisionShape.CAPSULE: return "Capsule"
		TestCollisionShape.CONCAVE_POLYGON: return "Concave Polygon"
		TestCollisionShape.CONCAVE_SEGMENT: return "Concave Segment"
		TestCollisionShape.CONVEX_POLYGON: return "Convex Polygon"
		TestCollisionShape.RECTANGLE: return "Rectangle"
#		TestCollisionShape.WORLD_BOUNDARY: return "World Boundary"
		TestCollisionShape.CIRCLE: return "Circle"
		_:
			@warning_ignore("assert_always_false")
			assert(false, "TestCollisionShape %d name is not implemented")
			return "Not implemented"

static func get_box_segments(width: float, height: float) -> PackedVector2Array:
	return [Vector2(+0.5 * width, +0.5 * height),
			Vector2(-0.5 * width, +0.5 * height),
			Vector2(-0.5 * width, -0.5 * height),
			Vector2(+0.5 * width, -0.5 * height)]

static func concave_array():
	return [
		Vector2(6.0, -10.0),
		Vector2(1.0, -1.0),
		Vector2(10.0, -6.0),
		Vector2(10.0, 6.0),
		Vector2(1.0, 1.0),
		Vector2(6.0, 10.0),
		Vector2(-6.0, 10.0),
		Vector2(-1.0, 1.0),
		Vector2(-10.0, 6.0),
		Vector2(-10.0, -6.0),
		Vector2(-1.0, -1.0),
		Vector2(-6.0, -10.0),
	]

static func convex_array():
	return [
		Vector2(0.0, -11.0),
		Vector2(5.0, -9.0),
		Vector2(10.0, -5.0),
		Vector2(11.0, 0.0),
		Vector2(10.0, 5.0),
		Vector2(5.0, 9.0),
		Vector2(0.0, 11.0),
		Vector2(-5.0, 9.0),
		Vector2(-10.0, 5.0),
		Vector2(-11.0, 0.0),
		Vector2(-10.0, -5.0),
		Vector2(-5.0, -9.0),
]
