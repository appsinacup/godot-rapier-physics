@tool
class_name Fluid3DRenderer
extends MultiMeshInstance3D

@export var fluid: Fluid3D


func _ready():
	if multimesh == null:
		multimesh = MultiMesh.new()
		multimesh.transform_format = MultiMesh.TRANSFORM_3D
		var sphere := SphereMesh.new()
		sphere.radius = ProjectSettings.get("physics/rapier/fluid/fluid_particle_radius_3d") / 2
		sphere.height = ProjectSettings.get("physics/rapier/fluid/fluid_particle_radius_3d")
		multimesh.mesh = sphere
		#multimesh.mesh = load("res://addons/godot-rapier2d/circle_mesh.tres").duplicate()
		multimesh.use_colors = true


func _process(_delta):
	if fluid == null || multimesh == null || multimesh.mesh == null:
		return
	global_transform = fluid.global_transform
	var index = 0
	multimesh.instance_count = fluid.points.size()
	var points = fluid.points
	for i in points.size():
		var point = points[i]
		var new_transform: Transform3D = Transform3D(Basis.IDENTITY, point)
		multimesh.set_instance_transform(index, new_transform)
		index += 1
