### Supports [CollisionObject2D], [Joint2D] and [CollisionShape2D].
@icon("res://addons/godot-rapier2d/logo_square_2d.png")
class_name Rapier2DState
extends Node

var state: Dictionary = {}


func _is_physics_object(node: Node) -> bool:
	return node is CollisionObject2D or node is Joint2D


func _get_all_physics_nodes(p_node: Node, path: String = "/root/") -> Array[String]:
	var results: Array[String] = []
	if path == "/root/" && _is_physics_object(p_node):
		results.append(path + p_node.name)
	path += p_node.name + "/"
	for node in p_node.get_children():
		if _is_physics_object(node):
			results.append(path + node.name)
		if node.get_child_count() > 0:
			results.append_array(_get_all_physics_nodes(node, path))
	return results


## Save a node's physics state
func save_node(rid: RID, save_json: bool):
	if save_json:
		return JSON.parse_string(RapierPhysicsServer2D.export_json(rid))
	return RapierPhysicsServer2D.export_binary(rid)


## Load a node's physics state
func load_node(rid: RID, data: PackedByteArray):
	RapierPhysicsServer2D.import_binary(rid, data)


## Save the state of whole world (single space)
func save_state(save_json: bool = false) -> int:
	var physics_nodes := _get_all_physics_nodes(get_tree().current_scene)
	for node_path in physics_nodes:
		var node := get_node(node_path)
		var rid: RID
		if node is CollisionObject2D:
			rid = node.get_rid()
			for owner_id in node.get_shape_owners():
				for owner_shape_id in node.shape_owner_get_shape_count(owner_id):
					var shape_rid = node.shape_owner_get_shape(owner_id, owner_shape_id).get_rid()
					state[node_path + "/" + str(owner_id) + "/" + str(owner_shape_id)] = save_node(
						shape_rid, save_json
					)
		if node is Joint2D:
			rid = node.get_rid()
		state[node_path] = save_node(rid, save_json)
	var space_rid = get_viewport().world_2d.space
	state["space"] = save_node(space_rid, save_json)
	state["id"] = RapierPhysicsServer2D.get_global_id()
	return hash(JSON.stringify(state))


## Load the state of whole world (single space)
func load_state() -> int:
	var physics_nodes := _get_all_physics_nodes(get_tree().current_scene)
	for node_path in physics_nodes:
		var node := get_node(node_path)
		var rid: RID
		if node is CollisionObject2D:
			rid = node.get_rid()
			for owner_id in node.get_shape_owners():
				for owner_shape_id in node.shape_owner_get_shape_count(owner_id):
					var shape_rid = node.shape_owner_get_shape(owner_id, owner_shape_id).get_rid()
					var shape_state = state[
						node_path + "/" + str(owner_id) + "/" + str(owner_shape_id)
					]
					load_node(shape_rid, JSON.parse_string(shape_state))
		if node is Joint2D:
			rid = node.get_rid()
		var node_state = state[node_path]
		load_node(rid, JSON.parse_string(node_state))
	var space_rid = get_viewport().world_2d.space
	load_node(space_rid, JSON.parse_string(state["space"]))
	RapierPhysicsServer2D.set_global_id(int(state["id"]))
	return hash(JSON.stringify(state))


## Export the state to file
func export_state(file_name: String = "user://state.json"):
	save_state(false)
	FileAccess.open(file_name, FileAccess.WRITE).store_string(JSON.stringify(state, " "))


## Import the state from file
func import_state(file_name: String = "user://state.json"):
	state = JSON.parse_string(FileAccess.open(file_name, FileAccess.READ).get_as_text())
	load_state()


func _notification(what: int) -> void:
	if what == NOTIFICATION_ENTER_TREE:
		print("enter tree")
	if what == NOTIFICATION_EXIT_TREE:
		save_state(false)
		FileAccess.open("user://save.json", FileAccess.WRITE).store_string(
			JSON.stringify(state, " ")
		)
