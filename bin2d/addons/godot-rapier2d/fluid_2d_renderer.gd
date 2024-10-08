@tool
class_name Fluid2DRenderer
extends MultiMeshInstance2D

@export var fluid: Fluid2D
@export var color: Color = Color(0.8, 0.8, 0.8, 0.3)
@export var mesh_scale: Vector2 = Vector2(5, 5)


func _ready():
	if multimesh == null:
		multimesh = MultiMesh.new()
		multimesh.mesh = load("res://addons/godot-rapier2d/circle_mesh.tres").duplicate()
		multimesh.use_colors = true
	if texture == null:
		texture = load("res://addons/godot-rapier2d/Radial2D.svg")


func _process(_delta):
	if fluid == null:
		return
	global_transform = fluid.global_transform
	var index = 0
	multimesh.instance_count = fluid.points.size()
	var points = fluid.points
	for i in points.size():
		var point = points[i]
		var new_transform: Transform2D = Transform2D(0, mesh_scale, 0, point)
		multimesh.set_instance_transform_2d(index, new_transform)
		multimesh.set_instance_color(index, color)
		index += 1
