extends PhysicsUnitTest3D

# Per-shape drop-and-rest test: one dynamic body of each collision shape type is dropped
# onto a static floor (top surface at y = 0). Each shape must collide with the floor (not
# tunnel through), come to rest on top of it, and never explode. This exercises collision +
# resting for every 3D primitive shape the plugin supports.

var simulation_duration := 12.0
const DROP_FRAME := 300

var names := ["Sphere", "Box", "Capsule", "Cylinder"]
var bodies: Array[RigidBody3D] = []

func test_description() -> String:
	return "Drops a body of each 3D shape type onto a floor; each must collide, rest on top, and not explode."

func test_name() -> String:
	return "Shape3D | drop and rest (sphere/box/capsule/cylinder)"

func _shape_and_mesh(p_kind: int) -> Array:
	match p_kind:
		0:
			var s := SphereShape3D.new(); s.radius = 0.6
			var m := SphereMesh.new(); m.radius = 0.6; m.height = 1.2
			return [s, m]
		1:
			var s := BoxShape3D.new(); s.size = Vector3(1.2, 1.2, 1.2)
			var m := BoxMesh.new(); m.size = Vector3(1.2, 1.2, 1.2)
			return [s, m]
		2:
			var s := CapsuleShape3D.new(); s.radius = 0.4; s.height = 1.6
			var m := CapsuleMesh.new(); m.radius = 0.4; m.height = 1.6
			return [s, m]
		_:
			var s := CylinderShape3D.new(); s.radius = 0.5; s.height = 1.2
			var m := CylinderMesh.new(); m.top_radius = 0.5; m.bottom_radius = 0.5; m.height = 1.2
			return [s, m]

func test_start() -> void:
	var cam := Camera3D.new()
	cam.position = Vector3(0, 3, 14)
	cam.look_at_from_position(cam.position, Vector3(0, 1, 0), Vector3.UP)
	add_child(cam)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	add_child(light)

	# Static floor with its top surface at y = 0.
	var floor_body := StaticBody3D.new()
	var floor_col := CollisionShape3D.new()
	floor_col.shape = BoxShape3D.new()
	floor_col.shape.size = Vector3(20, 2, 20)
	floor_col.position = Vector3(0, -1, 0)
	floor_body.add_child(floor_col)
	add_child(floor_body)

	for i in range(names.size()):
		var sm := _shape_and_mesh(i)
		var body := RigidBody3D.new()
		body.position = Vector3(-4.5 + i * 3.0, 5.0, 0)
		var col := CollisionShape3D.new()
		col.shape = sm[0]
		body.add_child(col)
		var mesh := MeshInstance3D.new()
		mesh.mesh = sm[1]
		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color(0.36, 0.66, 1.0)
		mesh.material_override = mat
		body.add_child(mesh)
		add_child(body)
		bodies.append(body)

	var state := {"finite": true}
	var checks := func(_t, m: GenericManualMonitor):
		for b in bodies:
			if not (is_finite(b.global_position.x) and is_finite(b.global_position.y) and is_finite(b.global_position.z)):
				state.finite = false
		if m.frame >= DROP_FRAME:
			m.add_test("No NaN/Inf in any dropped shape")
			m.add_test_result(state.finite)

			for i in range(bodies.size()):
				var b := bodies[i]
				var y := b.global_position.y
				var rested: bool = b.linear_velocity.length() < 0.5 and y > 0.1 and y < 3.0
				m.add_test("%s rests on the floor (no tunneling)" % names[i])
				if not rested:
					m.add_test_error("y=%.2f v=%.3f" % [y, b.linear_velocity.length()])
				m.add_test_result(rested)

			m.monitor_completed()
	create_generic_manual_monitor(self, checks, simulation_duration)
