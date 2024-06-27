class_name CollideShapeTest
extends Marker2D

var point0: Vector2
var point1: Vector2
var state2d: PhysicsDirectSpaceState2D
var params := PhysicsShapeQueryParameters2D.new()

var strings: Array[String] = ["circle", "rectangle", "capsule", "segment"]
var shapes: Array[Shape2D] = [CircleShape2D.new(), RectangleShape2D.new(), CapsuleShape2D.new(), SegmentShape2D.new()]
var idx := 0

func _ready() -> void:
	state2d = get_world_2d().direct_space_state

	var _transform = Transform2D.IDENTITY
	_transform.origin = global_position

	params.transform = transform
	params.shape = shapes[idx]

func _physics_process(_delta: float) -> void:
	queue_redraw()

	if Input.is_action_just_pressed("ui_select"):
		idx += 1
		if idx == len(shapes): idx = 0
		params.shape = shapes[idx]

	var results = state2d.collide_shape(params, 1)
	if len(results) == 2:
		point0 = results[0]
		point1 = results[1]

func _draw() -> void:
	draw_string(ThemeDB.fallback_font, Vector2(-15, 100), strings[idx],)
	draw_circle(point1 - global_position, 3, Color.GREEN)
	draw_circle(point0 - global_position, 2, Color.RED)
