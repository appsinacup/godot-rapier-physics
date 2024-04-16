class_name Fluid2DRenderer
extends MultiMeshInstance2D

@onready var fluid :Fluid2D = get_parent()

func _process(delta):
	var index = 0
	multimesh.instance_count = fluid.points.size()
	for point in fluid.points:
		var new_transform: Transform2D = Transform2D()
		new_transform.origin = point - position
		multimesh.set_instance_transform_2d(index, new_transform)
		index += 1
