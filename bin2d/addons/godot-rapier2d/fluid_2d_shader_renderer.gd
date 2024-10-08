@tool
class_name Fluid2DShaderRenderer
extends CanvasLayer

@export var fluid: Fluid2D:
	set(value):
		fluid = value
		update_configuration_warnings()
@export var camera: Camera2D
@export var water_material: Material = load("res://addons/godot-rapier2d/water_shader.tres")
@export var mesh_scale: Vector2 = Vector2(5, 5)
var fluid_renderer: Fluid2DRenderer
var inside_camera: Camera2D:
	set(value):
		inside_camera = value
		update_configuration_warnings()
var sub_viewport_container: SubViewportContainer
var sub_viewport: SubViewport


func _get_configuration_warnings():
	var warnings = []
	if camera == null:
		warnings += ["Camera property is empty."]
	if fluid == null:
		warnings += ["Fluid property is empty."]
	return warnings


func _create_subviewport_container():
	sub_viewport_container = SubViewportContainer.new()
	sub_viewport_container.name = "SubViewportContainer"
	add_child(sub_viewport_container)
	sub_viewport_container.material = water_material
	sub_viewport_container.size = Vector2(
		ProjectSettings.get("display/window/size/viewport_width"),
		ProjectSettings.get("display/window/size/viewport_height")
	)


func _create_subviewport():
	sub_viewport = SubViewport.new()
	sub_viewport.name = "SubViewport"
	sub_viewport_container.add_child(sub_viewport)
	sub_viewport.transparent_bg = true
	sub_viewport.size = sub_viewport_container.size


func _create_fluid_renderer():
	fluid_renderer = Fluid2DRenderer.new()
	fluid_renderer.name = "Fluid2DRenderer"
	fluid_renderer.color = Color(255, 0, 255)
	fluid_renderer.mesh_scale = mesh_scale
	fluid_renderer.fluid = fluid
	sub_viewport.add_child(fluid_renderer)


func _create_inside_camera():
	inside_camera = Camera2D.new()
	inside_camera.name = "Camera2D"
	inside_camera.material = water_material
	sub_viewport.add_child(inside_camera)


func _ready() -> void:
	_create_subviewport_container()
	_create_subviewport()
	_create_fluid_renderer()
	_create_inside_camera()
	if fluid:
		fluid.debug_draw = false


func _process(_delta: float) -> void:
	if camera != null:
		inside_camera.offset = camera.offset
		inside_camera.zoom = camera.zoom
		inside_camera.transform = camera.transform
		sub_viewport_container.scale = Vector2(1.0 / camera.zoom.x, 1.0 / camera.zoom.y)
		sub_viewport_container.position = camera.global_position
		sub_viewport.size = sub_viewport_container.size
		if camera.anchor_mode == Camera2D.AnchorMode.ANCHOR_MODE_FIXED_TOP_LEFT:
			sub_viewport_container.position -= sub_viewport_container.size / 2
		if !camera.ignore_rotation:
			sub_viewport_container.rotation = camera.global_rotation
		else:
			sub_viewport_container.rotation = 0
