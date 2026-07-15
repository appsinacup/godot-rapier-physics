class_name JointTest2D
extends PhysicsUnitTest2D

# Shared helpers for the 2D joint e2e tests. Every joint test builds a small, VISIBLE
# scene of RigidBody2D boxes wired together with a joint, then runs it and asserts three
# kinds of properties over time:
#   * functional  - the joint actually constrains the bodies (a joint-specific invariant).
#   * no explosion - positions and velocities stay finite and bounded for the whole run.
#   * stability    - with damping the bodies come to rest (final speed near zero).
# The bodies are given damping so "stable" simply means "settled", which is a robust,
# joint-agnostic check that still fails loudly if a joint injects energy and blows up.

const BOX := Vector2(30, 30)
const SETTLE_FRAME := 320        # ~5.3s at 60Hz: long enough for a firmly-damped rig to settle.
const MAX_DURATION := 30.0       # wall-clock budget before the monitor times out.
const POSITION_BOUND := 4000.0   # any |coordinate| beyond this counts as "exploded".
# Velocity threshold for "at rest". Kept generous so the check is robust to the small
# run-to-run variation of the threaded solver (the bodies are heavily damped, so a settled
# body sits far below this).
const SETTLED_SPEED := 25.0

var _colors := [
	Color(0.36, 0.66, 1.0), Color(1.0, 0.6, 0.3), Color(0.5, 0.85, 0.45),
	Color(0.85, 0.5, 0.85), Color(0.9, 0.8, 0.35), Color(0.5, 0.8, 0.85),
]

func make_box(p_pos: Vector2, p_static := false, p_index := 0) -> RigidBody2D:
	var body := RigidBody2D.new()
	body.position = p_pos
	body.linear_damp = 3.0
	body.angular_damp = 3.0
	if p_static:
		body.freeze = true
		body.freeze_mode = RigidBody2D.FREEZE_MODE_STATIC
	var col := CollisionShape2D.new()
	var shape := RectangleShape2D.new()
	shape.size = BOX
	col.shape = shape
	body.add_child(col)
	# Explicit visible polygon so the body is clearly seen even without debug collision draw.
	var poly := Polygon2D.new()
	poly.polygon = PackedVector2Array([
		Vector2(-BOX.x, -BOX.y) * 0.5, Vector2(BOX.x, -BOX.y) * 0.5,
		Vector2(BOX.x, BOX.y) * 0.5, Vector2(-BOX.x, BOX.y) * 0.5,
	])
	poly.color = _colors[p_index % _colors.size()] if not p_static else Color(0.5, 0.5, 0.5)
	body.add_child(poly)
	add_child(body)
	return body

static func finite_v(v: Vector2) -> bool:
	return is_finite(v.x) and is_finite(v.y)

static func in_bounds(v: Vector2) -> bool:
	return absf(v.x) < POSITION_BOUND and absf(v.y) < POSITION_BOUND
