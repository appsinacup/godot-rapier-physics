class_name Fluid2DRenderer
extends MultiMeshInstance2D

@onready var fluid :Fluid2D = get_parent()
@export var color: Color

func _ready():
	multimesh = multimesh.duplicate(true)

func _process(_delta):
	var index = 0
	multimesh.instance_count = fluid.points.size()
	var points = fluid.points
	var create_times = fluid.get_create_times()
	var current_time = Time.get_ticks_msec()
	for i in points.size():
		var point = points[i]
		var created_at = create_times[i]
		var new_transform: Transform2D = Transform2D()
		new_transform.origin = point - position
		multimesh.set_instance_transform_2d(index, new_transform)
		multimesh.set_instance_color(index, color)
		index += 1
