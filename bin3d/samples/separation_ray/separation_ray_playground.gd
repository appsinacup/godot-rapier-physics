extends Node3D

const RAY_LENGTH := 0.6
const BODY_HALF_HEIGHT := 0.5
const BODY_LIFT := 0.6
const RAY_POINTS_DOWN := Vector3(90, 0, 0)

const STEP_HEIGHT := 0.3
const STEP_DEPTH := 1.4
const STEP_COUNT := 5

const SPEED := 4.0
const JUMP_VELOCITY := 6.0
const RAMP_DEGREES := 60.0
const FALL_OUT_OF_WORLD_Y := -20.0

var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")

var player: CharacterBody3D
var ray_shape: SeparationRayShape3D
var ray_collider: CollisionShape3D
var info: Label
var spawn := Vector3(0, RAY_LENGTH, 4)

func _ready() -> void:
	build_level()
	build_player()
	build_hud()
	build_light()

func _physics_process(delta: float) -> void:
	if not player.is_on_floor():
		player.velocity.y -= gravity * delta

	var input := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	player.velocity.x = input.x * SPEED
	player.velocity.z = input.y * SPEED
	if Input.is_action_just_pressed("ui_accept") and player.is_on_floor():
		player.velocity.y = JUMP_VELOCITY

	player.move_and_slide()

	if player.position.y < FALL_OUT_OF_WORLD_Y:
		respawn()

	update_hud()

func _unhandled_key_input(event: InputEvent) -> void:
	var key := event as InputEventKey
	if key == null or not key.pressed or key.echo:
		return
	match key.keycode:
		KEY_R:
			ray_collider.disabled = not ray_collider.disabled
		KEY_S:
			ray_shape.slide_on_slope = not ray_shape.slide_on_slope
		KEY_ESCAPE:
			respawn()

func respawn() -> void:
	player.position = spawn
	player.velocity = Vector3.ZERO

func update_hud() -> void:
	info.text = "\n".join([
		"arrows: move    space: jump    R: toggle ray    S: toggle slide_on_slope    esc: respawn",
		"separation ray: %s" % ("OFF" if ray_collider.disabled else "ON"),
		"slide_on_slope: %s" % ray_shape.slide_on_slope,
		"on_floor: %s    floor_normal: %.2v" % [player.is_on_floor(), player.get_floor_normal()],
	])

func build_player() -> void:
	player = CharacterBody3D.new()
	player.position = spawn
	player.floor_snap_length = RAY_LENGTH

	var body_col := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(0.8, BODY_HALF_HEIGHT * 2.0, 0.8)
	body_col.shape = box
	body_col.position = Vector3(0, BODY_LIFT + BODY_HALF_HEIGHT, 0)
	player.add_child(body_col)

	var mesh := MeshInstance3D.new()
	var box_mesh := BoxMesh.new()
	box_mesh.size = box.size
	mesh.mesh = box_mesh
	mesh.position = body_col.position
	player.add_child(mesh)

	ray_shape = SeparationRayShape3D.new()
	ray_shape.length = RAY_LENGTH
	ray_shape.slide_on_slope = false
	ray_collider = CollisionShape3D.new()
	ray_collider.shape = ray_shape
	ray_collider.rotation_degrees = RAY_POINTS_DOWN
	player.add_child(ray_collider)

	add_child(player)

	var camera := Camera3D.new()
	camera.position = Vector3(0, 6, 10)
	camera.rotation_degrees = Vector3(-25, 0, 0)
	player.add_child(camera)

func build_level() -> void:
	add_child(make_box(Vector3(0, -0.5, 2), Vector3(20, 1, 12)))

	for i in STEP_COUNT:
		var center_z := -4.0 - i * STEP_DEPTH
		add_child(make_box(Vector3(0, step_top(i) - 0.5, center_z), Vector3(8, 1, STEP_DEPTH)))

	var top_y := step_top(STEP_COUNT - 1)
	var ramp := make_box(Vector3(0, top_y + 1.0, -6.0 - STEP_COUNT * STEP_DEPTH), Vector3(8, 1, 6))
	ramp.rotation_degrees = Vector3(RAMP_DEGREES, 0, 0)
	add_child(ramp)

func build_hud() -> void:
	var layer := CanvasLayer.new()
	info = Label.new()
	info.position = Vector2(12, 8)
	info.set("theme_override_font_sizes/font_size", 16)
	layer.add_child(info)
	add_child(layer)

func build_light() -> void:
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	add_child(light)

static func step_top(p_index: int) -> float:
	return (p_index + 1) * STEP_HEIGHT

static func make_box(p_center: Vector3, p_size: Vector3) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.position = p_center
	var col := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = p_size
	col.shape = shape
	body.add_child(col)
	var mesh := MeshInstance3D.new()
	var box_mesh := BoxMesh.new()
	box_mesh.size = p_size
	mesh.mesh = box_mesh
	body.add_child(mesh)
	return body
