extends Area2D

@export var overlap_report : Label

func _physics_process(delta: float) -> void:
	overlap_report.text = str(get_overlapping_bodies().size())


func _on_body_exited(body: Node2D) -> void:
	print("BODY EXITED!!!!!!!!!!!!!!")


func _on_body_entered(body: Node2D) -> void:
	print("BODY ENTERRRRRRRRRRRRRRED!!!!!!!!!!")
