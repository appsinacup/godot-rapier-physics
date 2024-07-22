class_name Fluid2DRenderer
extends MultiMeshInstance2D

@export var fluid: Fluid2D
@export var color: Color

func _ready():
	multimesh = multimesh.duplicate(true)

func _process(_delta):
	global_position = fluid.global_position
	var index = 0
	multimesh.instance_count = fluid.points.size()
	var points = fluid.points
	for i in points.size():
		var point = points[i]
		var new_transform: Transform2D = Transform2D(0, Vector2(5, 5), 0, point)
		multimesh.set_instance_transform_2d(index, new_transform)
		multimesh.set_instance_color(index, color)
		index += 1
