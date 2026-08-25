extends Node2D

var frames := 0
var ground: StaticBody2D

func _ready() -> void:
	ground = StaticBody2D.new()
	var poly := CollisionPolygon2D.new()
	poly.polygon = PackedVector2Array([
		Vector2(0, 507), Vector2(0, 0), Vector2(64, 0), Vector2(64, 507),
		Vector2(502, 155), Vector2(502, 0), Vector2(634, 0), Vector2(634, 155),
		Vector2(1072, 507), Vector2(1072, 0), Vector2(1152, 0), Vector2(1152, 648),
		Vector2(0, 648)])
	ground.add_child(poly)
	add_child(ground)

func _physics_process(_d) -> void:
	frames += 1
	if frames != 5:
		if frames > 5: get_tree().quit()
		return
	var space := get_world_2d().direct_space_state
	print("shapes: %d" % PhysicsServer2D.body_get_shape_count(ground.get_rid()))

	var ray_shapes := {}
	for x in [30.0, 200.0, 560.0, 900.0, 1120.0]:
		var q := PhysicsRayQueryParameters2D.create(Vector2(x, -50), Vector2(x, 700))
		var hit := space.intersect_ray(q)
		if hit: ray_shapes[hit.shape] = true
	print("ray distinct: %d" % ray_shapes.size())

	var pt_shapes := {}
	for p in [Vector2(30, 300), Vector2(200, 600), Vector2(560, 100), Vector2(1120, 300)]:
		var q := PhysicsPointQueryParameters2D.new()
		q.position = p
		for hit in space.intersect_point(q):
			pt_shapes[hit.shape] = true
	print("point distinct: %d" % pt_shapes.size())

	var circle := CircleShape2D.new(); circle.radius = 10.0
	var sh_shapes := {}
	for p in [Vector2(30, 300), Vector2(200, 600), Vector2(560, 100), Vector2(1120, 300)]:
		var q := PhysicsShapeQueryParameters2D.new()
		q.shape = circle
		q.transform = Transform2D(0.0, p)
		for hit in space.intersect_shape(q):
			sh_shapes[hit.shape] = true
	print("shape distinct: %d" % sh_shapes.size())

	var ri_shapes := {}
	for p in [Vector2(30, 300), Vector2(200, 600), Vector2(560, 100), Vector2(1120, 300)]:
		var q := PhysicsShapeQueryParameters2D.new()
		q.shape = circle
		q.transform = Transform2D(0.0, p)
		var info := space.get_rest_info(q)
		if info: ri_shapes[info.shape] = true
	print("rest_info distinct: %d" % ri_shapes.size())
	print("(pre-fix: point/shape/rest_info always 1)")
