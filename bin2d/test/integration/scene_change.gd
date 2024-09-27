class_name Rapier2DState
extends Node

@export var state : Dictionary = {}

func is_physics_object(node: Node) -> bool:
	return node is CollisionObject2D or \
		node is CollisionShape2D or \
		node is CollisionPolygon2D or \
		node is Joint2D

func get_all_physics_nodes(p_node: Node, path: String = "/root/") -> Array[String]:
	var results : Array[String] = []
	if path == "/root/" && is_physics_object(p_node):
		results.append(path + p_node.name)
	path += p_node.name + "/"
	for node in p_node.get_children():
		if is_physics_object(node):
			results.append(path + node.name)
		if node.get_child_count() > 0:
			results.append_array(get_all_physics_nodes(node, path))
	return results


func save_state(save_json: bool = false) -> int:
	var physics_nodes := get_all_physics_nodes(get_tree().current_scene)
	for node_path in physics_nodes:
		var node := get_node(node_path)
		var node_state
		var rid : RID
		if node is CollisionObject2D:
			rid = node.get_rid()
		if node is CollisionShape2D:
			rid = node.shape.get_rid()
		if node is CollisionPolygon2D:
			node.get_canvas_item()
			var parent = node.get_parent()
			if parent is CollisionObject2D:
				var idx = 0
				for child in parent.get_children():
					if child is CollisionShape2D:
						if child.shape.disabled:
							continue
					if child is CollisionPolygon2D:
						if child.disabled:
							continue
					if child == node:
						break
					idx += 1
				rid = PhysicsServer2D.body_get_shape(parent.get_rid(), idx)
				#PhysicsServer2D.body_get_shape()
				#rid = node.get_rid()
			else:
				print("Cannot get RID of CollisionPolygon2D")
		if node is Joint2D:
			rid = node.get_rid()
		if save_json:
			node_state = RapierPhysicsServer2D.export_json(rid)
		else:
			node_state = RapierPhysicsServer2D.export_binary(rid)
		state[RapierPhysicsServer2D.get_handle(rid)] = {
			"state" : node_state,
			"path": node_path
		}
	var space_rid = get_viewport().world_2d.space
	var space_state
	if save_json:
		space_state = RapierPhysicsServer2D.export_json(space_rid)
	else:
		space_state = RapierPhysicsServer2D.export_binary(space_rid)
	state[RapierPhysicsServer2D.get_handle(space_rid)] = {
		"state" : space_state,
		"path": "space"
	}
	return hash(state)


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print(save_state(false))
	# Print the result as hex string and array.
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
