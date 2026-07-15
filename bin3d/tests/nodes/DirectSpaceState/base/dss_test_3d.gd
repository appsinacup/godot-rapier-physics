class_name DSSTest3D
extends PhysicsUnitTest3D

# Shared helpers for the 3D DirectSpaceState query tests. Each test builds a couple of
# visible static bodies / areas at known world positions, then exercises one query method
# of PhysicsDirectSpaceState3D (intersect_ray / intersect_point / intersect_shape /
# collide_shape / cast_motion) across its parameters (collide_with_bodies/areas, exclude,
# collision_mask, result limits). Each test runs in its own SubViewport world.

const HALF := 1.0                       # box half-extent -> BoxShape3D size = 2
const LEFT := Vector3(-4, 0, 0)
const RIGHT := Vector3(4, 0, 0)
const CENTER := Vector3(0, 0, 0)

var _view_ready := false

func _ensure_view() -> void:
	if _view_ready:
		return
	_view_ready = true
	var cam := Camera3D.new()
	cam.position = Vector3(0, 2.5, 13)
	cam.look_at_from_position(cam.position, CENTER, Vector3.UP)
	add_child(cam)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	add_child(light)

func _box_col(p_color: Color) -> Node3D:
	var col := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = Vector3(2, 2, 2)
	col.shape = shape
	var mesh := MeshInstance3D.new()
	var bm := BoxMesh.new()
	bm.size = Vector3(2, 2, 2)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = p_color
	bm.material = mat
	mesh.mesh = bm
	col.add_child(mesh)
	return col

func add_static_body(p_pos: Vector3, p_layer := 1) -> StaticBody3D:
	_ensure_view()
	var body := StaticBody3D.new()
	body.position = p_pos
	body.collision_layer = 0
	body.collision_mask = 0
	body.set_collision_layer_value(p_layer, true)
	body.add_child(_box_col(Color(0.36, 0.66, 1.0)))
	add_child(body)
	return body

func add_area(p_pos: Vector3, p_layer := 1) -> Area3D:
	_ensure_view()
	var area := Area3D.new()
	area.position = p_pos
	area.collision_layer = 0
	area.collision_mask = 0
	area.set_collision_layer_value(p_layer, true)
	area.add_child(_box_col(Color(1.0, 0.6, 0.3)))
	add_child(area)
	return area

func query_box_shape() -> RID:
	var s := PhysicsServer3D.box_shape_create()
	PhysicsServer3D.shape_set_data(s, Vector3(HALF, HALF, HALF))
	return s
