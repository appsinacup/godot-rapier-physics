extends Node2D

const RAY_LENGTH := 30.0
const BODY_HALF_SIZE := 20.0
const BODY_LIFT := 30.0

const STEP_HEIGHT := 20.0
const STEP_WIDTH := 70.0
const STEP_COUNT := 5

const GROUND_Y := 400.0
const SPEED := 300.0
const JUMP_VELOCITY := -600.0
const RAMP_DEGREES := 60.0
const RAMP_RISE := 200.0
const FALL_OUT_OF_WORLD_Y := GROUND_Y + 600.0

var gravity: float = ProjectSettings.get_setting("physics/2d/default_gravity")

var player: CharacterBody2D
var ray_shape: SeparationRayShape2D
var ray_collider: CollisionShape2D
var info: Label
var spawn := Vector2(-450, GROUND_Y - RAY_LENGTH)

func _ready() -> void:
	build_level()
	build_player()
	build_hud()

func _physics_process(delta: float) -> void:
	if not player.is_on_floor():
		player.velocity.y += gravity * delta

	player.velocity.x = Input.get_axis("ui_left", "ui_right") * SPEED
	if Input.is_action_just_pressed("ui_accept") and player.is_on_floor():
		player.velocity.y = JUMP_VELOCITY

	player.move_and_slide()

	if player.position.y > FALL_OUT_OF_WORLD_Y:
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
	player.velocity = Vector2.ZERO

func update_hud() -> void:
	info.text = "\n".join([
		"arrows: move    space: jump    R: toggle ray    S: toggle slide_on_slope    esc: respawn",
		"separation ray: %s" % ("OFF" if ray_collider.disabled else "ON"),
		"slide_on_slope: %s" % ray_shape.slide_on_slope,
		"on_floor: %s    floor_normal: %.2v" % [player.is_on_floor(), player.get_floor_normal()],
	])

func build_player() -> void:
	player = CharacterBody2D.new()
	player.position = spawn
	player.floor_snap_length = RAY_LENGTH

	var body_col := CollisionShape2D.new()
	var box := RectangleShape2D.new()
	box.size = Vector2(BODY_HALF_SIZE, BODY_HALF_SIZE) * 2.0
	body_col.shape = box
	body_col.position = Vector2(0, -BODY_LIFT - BODY_HALF_SIZE)
	player.add_child(body_col)

	ray_shape = SeparationRayShape2D.new()
	ray_shape.length = RAY_LENGTH
	ray_shape.slide_on_slope = false
	ray_collider = CollisionShape2D.new()
	ray_collider.shape = ray_shape
	player.add_child(ray_collider)

	add_child(player)

	var camera := Camera2D.new()
	player.add_child(camera)

func build_level() -> void:
	add_child(make_box(Rect2(-700, GROUND_Y, 700, 300)))

	for i in STEP_COUNT:
		add_child(make_box(Rect2(i * STEP_WIDTH, step_top(i), STEP_WIDTH, 300)))

	var landing_top := step_top(STEP_COUNT - 1)
	var landing_left := STEP_COUNT * STEP_WIDTH
	add_child(make_box(Rect2(landing_left, landing_top, 200, 300)))

	add_child(build_ramp(Vector2(landing_left + 200, landing_top)))

func build_ramp(p_foot: Vector2) -> StaticBody2D:
	var run := RAMP_RISE / tan(deg_to_rad(RAMP_DEGREES))
	var ramp := StaticBody2D.new()
	var poly := CollisionPolygon2D.new()
	poly.polygon = PackedVector2Array([
		p_foot,
		p_foot + Vector2(run, -RAMP_RISE),
		p_foot + Vector2(run + 200, -RAMP_RISE),
		p_foot + Vector2(run + 200, 300),
		p_foot + Vector2(0, 300),
	])
	ramp.add_child(poly)
	return ramp

func build_hud() -> void:
	var layer := CanvasLayer.new()
	info = Label.new()
	info.position = Vector2(12, 8)
	info.set("theme_override_font_sizes/font_size", 16)
	layer.add_child(info)
	add_child(layer)

static func step_top(p_index: int) -> float:
	return GROUND_Y - (p_index + 1) * STEP_HEIGHT

static func make_box(p_rect: Rect2) -> StaticBody2D:
	var body := StaticBody2D.new()
	var col := CollisionShape2D.new()
	var shape := RectangleShape2D.new()
	shape.size = p_rect.size
	col.shape = shape
	col.position = p_rect.position + p_rect.size * 0.5
	body.add_child(col)
	return body
