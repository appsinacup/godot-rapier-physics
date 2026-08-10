extends Node2D

## Same pile as scaling_bench, but engine-agnostic: reports only TIME_PHYSICS_PROCESS, which
## every physics server feeds. Lets Rapier2D and GodotPhysics2D be compared on identical work,
## which is the only way to tell an inherent API cost from one this extension introduces.

@export var body_count := 1500
@export var frames := 600
@export var columns := 50
@export var warmup := 120

var _frame := 0
var _samples: Array[float] = []
var _paddles: Array[AnimatableBody2D] = []


func _ready() -> void:
	var ground := StaticBody2D.new()
	var ground_shape := CollisionShape2D.new()
	var ground_rect := RectangleShape2D.new()
	ground_rect.size = Vector2(6000, 100)
	ground_shape.shape = ground_rect
	ground.add_child(ground_shape)
	ground.position = Vector2(0, 400)
	add_child(ground)

	for side in [-1.0, 1.0]:
		var wall := StaticBody2D.new()
		var wall_shape := CollisionShape2D.new()
		var wall_rect := RectangleShape2D.new()
		wall_rect.size = Vector2(100, 2000)
		wall_shape.shape = wall_rect
		wall.add_child(wall_shape)
		wall.position = Vector2(side * (columns * 16.0 + 60.0), -600)
		add_child(wall)

	var box := RectangleShape2D.new()
	box.size = Vector2(24, 24)
	for i in body_count:
		var body := RigidBody2D.new()
		var shape := CollisionShape2D.new()
		shape.shape = box
		body.add_child(shape)
		var col := i % columns
		var row := i / columns
		body.position = Vector2((col - columns / 2.0) * 28.0, 300.0 - row * 30.0)
		add_child(body)

	# Rotating paddles keep the pile churning. Stirring by looping over every body in
	# GDScript instead would put 1500 script->engine calls per frame inside
	# TIME_PHYSICS_PROCESS, which swamps the physics signal and adds most of its variance.
	for i in 2:
		var paddle := AnimatableBody2D.new()
		var paddle_shape := CollisionShape2D.new()
		var paddle_rect := RectangleShape2D.new()
		paddle_rect.size = Vector2(700, 40)
		paddle_shape.shape = paddle_rect
		paddle.add_child(paddle_shape)
		paddle.position = Vector2(0, 100 - i * 500)
		add_child(paddle)
		_paddles.append(paddle)


func _physics_process(delta: float) -> void:
	_frame += 1
	for i in _paddles.size():
		_paddles[i].rotation += delta * (2.5 if i % 2 == 0 else -2.5)
	if _frame > warmup:
		_samples.append(Performance.get_monitor(Performance.TIME_PHYSICS_PROCESS) * 1000.0)
	if _frame >= frames:
		_samples.sort()
		var sum := 0.0
		for s in _samples:
			sum += s
		print(
			(
				"[compare] engine=%s bodies=%d frames=%d mean=%.3fms median=%.3fms"
				% [
					ProjectSettings.get_setting("physics/2d/physics_engine"),
					body_count,
					_samples.size(),
					sum / _samples.size(),
					_samples[_samples.size() / 2],
				]
			)
		)
		get_tree().quit()
