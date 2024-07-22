extends CanvasLayer

@export var camera: Camera2D
@export var fluid: Fluid2D
@export var water_material: Material = load("res://samples/godot-rapier2d/water_shader.tres")
@onready var fluid_renderer: Fluid2DRenderer = $SubViewportContainer/SubViewport/Fluid2DRenderer
@onready var inside_camera: Camera2D = $SubViewportContainer/SubViewport/Camera2D
@onready var subviewport_container = $SubViewportContainer

func _ready() -> void:
	inside_camera.offset = camera.offset
	inside_camera.zoom = camera.zoom
	inside_camera.transform = camera.transform
	fluid_renderer.fluid = fluid
	subviewport_container.material = water_material
