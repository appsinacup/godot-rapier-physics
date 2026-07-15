class_name JointTest3D
extends PhysicsUnitTest3D

# Shared helpers for the 3D joint e2e tests. Same philosophy as JointTest2D: build a small,
# VISIBLE scene of RigidBody3D boxes wired with a joint, then assert functional, no-explosion
# and stability (settled, thanks to damping) properties over time. A camera and a light are
# added automatically so the tiles are actually visible when the suite is run windowed.

const BOX := Vector3(0.6, 0.6, 0.6)
const SETTLE_FRAME := 320
const MAX_DURATION := 30.0
const POSITION_BOUND := 1000.0
# Generous "at rest" threshold: robust to the threaded solver's small run-to-run variation
# (the bodies are heavily damped, so a settled body sits far below this).
const SETTLED_SPEED := 0.6

var _view_ready := false
var _colors := [
	Color(0.36, 0.66, 1.0), Color(1.0, 0.6, 0.3), Color(0.5, 0.85, 0.45),
	Color(0.85, 0.5, 0.85), Color(0.9, 0.8, 0.35), Color(0.5, 0.8, 0.85),
]

func _ensure_view() -> void:
	if _view_ready:
		return
	_view_ready = true
	var cam := Camera3D.new()
	cam.position = Vector3(3.5, 0.0, 9.0)
	cam.look_at_from_position(cam.position, Vector3(0, -1.0, 0), Vector3.UP)
	add_child(cam)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -35, 0)
	add_child(light)

func make_box(p_pos: Vector3, p_static := false, p_index := 0) -> RigidBody3D:
	_ensure_view()
	var body := RigidBody3D.new()
	body.position = p_pos
	body.linear_damp = 3.0
	body.angular_damp = 3.0
	if p_static:
		body.freeze = true
		body.freeze_mode = RigidBody3D.FREEZE_MODE_STATIC
	var col := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = BOX
	col.shape = shape
	body.add_child(col)
	var mesh := MeshInstance3D.new()
	var box_mesh := BoxMesh.new()
	box_mesh.size = BOX
	var mat := StandardMaterial3D.new()
	mat.albedo_color = _colors[p_index % _colors.size()] if not p_static else Color(0.5, 0.5, 0.5)
	box_mesh.material = mat
	mesh.mesh = box_mesh
	body.add_child(mesh)
	add_child(body)
	return body

static func finite_v(v: Vector3) -> bool:
	return is_finite(v.x) and is_finite(v.y) and is_finite(v.z)

static func in_bounds(v: Vector3) -> bool:
	return absf(v.x) < POSITION_BOUND and absf(v.y) < POSITION_BOUND and absf(v.z) < POSITION_BOUND
