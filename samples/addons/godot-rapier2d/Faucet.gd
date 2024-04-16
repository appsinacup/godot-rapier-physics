extends Node2D

@export var fluid :Fluid2D

var points

func _ready():
	points = fluid.points

func _on_timer_timeout():
	fluid.points += points
	print(len(fluid.points))
