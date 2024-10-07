class_name PhysicsTest3D
extends Node3D

signal completed

var output := ""

const cached_poly_convex: ConvexPolygonShape3D = preload("res://base/mesh/convex_8v_box_polygon_shape_3d.tres")
const cached_medium_poly_convex: ConvexPolygonShape3D = preload("res://base/mesh/convex_32v_box_polygon_shape_3d.tres")
const cached_high_poly_convex: ConvexPolygonShape3D = preload("res://base/mesh/convex_146v_sphere_polygon_3d.tres")
const cached_ultra_high_poly_convex: ConvexPolygonShape3D = preload("res://base/mesh/convex_2050v_sphere_polygon_3d.tres")

enum TestCollisionShape {
	CAPSULE = PhysicsServer3D.SHAPE_CAPSULE,
	CONCAVE_POLYGON = PhysicsServer3D.SHAPE_CONCAVE_POLYGON,
	CONVEX_POLYGON = PhysicsServer3D.SHAPE_CONVEX_POLYGON,
	CONVEX_POLYGON_MEDIUM_VERTEX = 99,
	CONVEX_POLYGON_HIGH_VERTEX = 100,
	CONVEX_POLYGON_ULTRA_HIGH_VERTEX = 101,
	BOX = PhysicsServer3D.SHAPE_BOX,
	SPHERE = PhysicsServer3D.SHAPE_SPHERE,
	CYLINDER = PhysicsServer3D.SHAPE_CYLINDER,
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

static func get_static_body_with_collision_shape(p_shape_definition, p_shape_type := TestCollisionShape.BOX) -> StaticBody3D:
	var body = StaticBody3D.new()
	body.position = Vector3(0, 0, 0)
	var body_col = get_collision_shape(p_shape_definition, p_shape_type)
	body.add_child(body_col)
	return body

static func get_collision_shape(p_shape_definition, p_shape_type := TestCollisionShape.BOX) -> Node3D:
	var col = null
	if p_shape_type == TestCollisionShape.CAPSULE:
		assert(p_shape_definition is Vector2, "Shape definition for a Capsule must be a Vector2")
		col = CollisionShape3D.new()
		col.shape = CapsuleShape3D.new()
		col.shape.radius = p_shape_definition.x
		col.shape.height = p_shape_definition.y
	elif p_shape_type == TestCollisionShape.CYLINDER:
		assert(p_shape_definition is Vector2, "Shape definition for a Cylinder must be a Vector2")
		col = CollisionShape3D.new()
		col.shape = CylinderShape3D.new()
		col.shape.radius = p_shape_definition.x
		col.shape.height = p_shape_definition.y
	elif p_shape_type == TestCollisionShape.SPHERE:
		assert(p_shape_definition is float, "Shape definition for a Sphere must be a float")
		col = CollisionShape3D.new()
		col.shape = SphereShape3D.new()
		col.shape.radius = p_shape_definition
	elif p_shape_type == TestCollisionShape.BOX:
		assert(p_shape_definition is Vector3, "Shape definition for a Box must be a Vector3")
		col = CollisionShape3D.new()
		col.shape = BoxShape3D.new()
		col.shape.size = p_shape_definition
	elif p_shape_type == TestCollisionShape.CONCAVE_POLYGON:
		assert(p_shape_definition is PackedVector3Array, "Shape definition for a Concave Polygon must be a PackedVector3Array")
		col = CollisionShape3D.new()
		col.shape = ConcavePolygonShape3D.new()
		col.shape.set_faces(p_shape_definition)
	elif p_shape_type == TestCollisionShape.CONVEX_POLYGON_MEDIUM_VERTEX:
		col = CollisionShape3D.new()
		col.shape = cached_medium_poly_convex
	elif p_shape_type == TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX:
		col = CollisionShape3D.new()
		col.shape = cached_high_poly_convex
	elif p_shape_type == TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX:
		col = CollisionShape3D.new()
		col.shape = cached_ultra_high_poly_convex
	elif p_shape_type == TestCollisionShape.CONVEX_POLYGON:
		col = CollisionShape3D.new()
		col.shape = cached_poly_convex
	return col

static func get_default_collision_shape(p_shape_type : TestCollisionShape, p_scale := 1.0):
	return get_collision_shape(get_default_shape_definition(p_shape_type, p_scale), p_shape_type)

static func get_default_shape_definition(p_shape_type : TestCollisionShape, p_scale := 1.0):
	if p_shape_type == TestCollisionShape.BOX:
		return Vector3(1 * p_scale, 1 * p_scale, 1 * p_scale)
	if p_shape_type == TestCollisionShape.SPHERE:
		return 0.5 * p_scale
	if p_shape_type == TestCollisionShape.CAPSULE:
		return Vector2(1,2) * p_scale
	if p_shape_type == TestCollisionShape.CYLINDER:
		return Vector2(.5, 1.0) * p_scale
	if p_shape_type == TestCollisionShape.CONCAVE_POLYGON:
		return get_trimesh_box_faces(p_scale, p_scale, p_scale)
	if p_shape_type == TestCollisionShape.CONVEX_POLYGON_MEDIUM_VERTEX  or p_shape_type == TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX or p_shape_type == TestCollisionShape.CONVEX_POLYGON or p_shape_type == TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX:
		return null
	
	@warning_ignore("assert_always_false")
	assert(false, "No default shape for this shape type")

static func shape_name(p_shape_type : TestCollisionShape) -> String:
	match p_shape_type:
		TestCollisionShape.CAPSULE: return "Capsule"
		TestCollisionShape.CYLINDER: return "Cylinder"
		TestCollisionShape.CONCAVE_POLYGON: return "Concave Polygon"
		TestCollisionShape.CONVEX_POLYGON: return "Convex 8v"
		TestCollisionShape.CONVEX_POLYGON_MEDIUM_VERTEX: return "Convex 146v"
		TestCollisionShape.CONVEX_POLYGON_HIGH_VERTEX: return "Convex 1026v"
		TestCollisionShape.CONVEX_POLYGON_ULTRA_HIGH_VERTEX: return "Convex 2050v"
		TestCollisionShape.BOX: return "Box"
#		TestCollisionShape.WORLD_BOUNDARY: return "World Boundary"
		TestCollisionShape.SPHERE: return "Sphere"
		_:
			@warning_ignore("assert_always_false")
			assert(false, "TestCollisionShape %d name is not implemented")
			return "Not implemented"

static func get_trimesh_box_faces(width: float, height: float, depth: float) -> PackedVector3Array:
	return [Vector3(-0.5 * width, +0.5 * height, +0.5 * depth), Vector3(+0.5 * width, +0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, +0.5 * depth),
			Vector3(+0.5 * width, +0.5 * height, +0.5 * depth), Vector3(+0.5 * width, -0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, +0.5 * depth),
			Vector3(+0.5 * width, +0.5 * height, -0.5 * depth), Vector3(-0.5 * width, +0.5 * height, -0.5 * depth), Vector3(+0.5 * width, -0.5 * height, -0.5 * depth),
			Vector3(-0.5 * width, +0.5 * height, -0.5 * depth), Vector3(-0.5 * width, -0.5 * height, -0.5 * depth), Vector3(+0.5 * width, -0.5 * height, -0.5 * depth),
			Vector3(+0.5 * width, +0.5 * height, +0.5 * depth), Vector3(+0.5 * width, +0.5 * height, -0.5 * depth), Vector3(+0.5 * width, -0.5 * height, +0.5 * depth),
			Vector3(+0.5 * width, +0.5 * height, -0.5 * depth), Vector3(+0.5 * width, -0.5 * height, -0.5 * depth), Vector3(+0.5 * width, -0.5 * height, +0.5 * depth),
			Vector3(-0.5 * width, +0.5 * height, -0.5 * depth), Vector3(-0.5 * width, +0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, -0.5 * depth),
			Vector3(-0.5 * width, +0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, -0.5 * depth),
			Vector3(+0.5 * width, +0.5 * height, +0.5 * depth), Vector3(-0.5 * width, +0.5 * height, +0.5 * depth), Vector3(+0.5 * width, +0.5 * height, -0.5 * depth),
			Vector3(-0.5 * width, +0.5 * height, +0.5 * depth), Vector3(-0.5 * width, +0.5 * height, -0.5 * depth), Vector3(+0.5 * width, +0.5 * height, -0.5 * depth),
			Vector3(-0.5 * width, -0.5 * height, +0.5 * depth), Vector3(+0.5 * width, -0.5 * height, +0.5 * depth), Vector3(-0.5 * width, -0.5 * height, -0.5 * depth),
			Vector3(+0.5 * width, -0.5 * height, +0.5 * depth), Vector3(+0.5 * width, -0.5 * height, -0.5 * depth), Vector3(-0.5 * width, -0.5 * height, -0.5 * depth)]

var zoom_factor := 5
var dragging = false
func _unhandled_input(event):
	var camera = get_node("Camera")
	if camera:
		if event.is_action_pressed("zoom_in"):
			create_tween().tween_property(camera, "position:z", camera.position.z + zoom_factor, 0.1)
		if event.is_action_pressed("zoom_out"):
			create_tween().tween_property(camera, "position:z", camera.position.z - zoom_factor, 0.1)
		if event is InputEventMouseButton:
			if event.is_pressed():
				dragging = true
			else:
				dragging = false
		elif event is InputEventMouseMotion and dragging:
			var offset: Vector2 = -event.relative.normalized()
			camera.position.y -= offset.y * 0.3
			camera.position.x += offset.x * 0.3
