@tool
class_name Fluid2DRenderer
extends MultiMeshInstance2D

@export var fluid: Fluid2D:
	set(v): fluid = v; set_process(fluid != null)
@export var color: Color = Color(0.8, 0.8, 0.8, 0.3)
@export_custom(PROPERTY_HINT_LINK, "") var mesh_scale: Vector2 = Vector2(5, 5)

const RADIAL_2D_TEXTURE := preload("uid://cho3shol3rky2")
const CIRCLE_MESH := preload("uid://dahp28qij58i1")


func _ready():
	if multimesh == null:
		multimesh = MultiMesh.new()
		multimesh.mesh = CIRCLE_MESH.duplicate()
		multimesh.use_colors = true
	if texture == null:
		texture = RADIAL_2D_TEXTURE


func _process(_delta):
	if fluid == null:
		set_process(false)
		return
	global_transform = fluid.global_transform
	multimesh.instance_count = fluid.points.size()
	for i in fluid.points.size():
		multimesh.set_instance_transform_2d(i, Transform2D(0, mesh_scale, 0, fluid.points[i]))
		multimesh.set_instance_color(i, color)
