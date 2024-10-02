@icon("res://addons/godot-rapier2d/logo_square_2d.png")
class_name Rapier2DState
extends Node

@export var state : Dictionary = {}

func is_physics_object(node: Node) -> bool:
	return node is CollisionObject2D or \
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

func save_node(rid: RID, save_json: bool):
	if save_json:
		return JSON.parse_string(RapierPhysicsServer2D.export_json(rid))
	else:
		return RapierPhysicsServer2D.export_binary(rid)

func load_node(rid: RID, data: PackedByteArray):
	RapierPhysicsServer2D.import_binary(rid, data)

func save_state(save_json: bool = false) -> int:
	var physics_nodes := get_all_physics_nodes(get_tree().current_scene)
	for node_path in physics_nodes:
		var node := get_node(node_path)
		var rid : RID
		if node is CollisionObject2D:
			rid = node.get_rid()
			for owner_id in node.get_shape_owners():
				for owner_shape_id in node.shape_owner_get_shape_count(owner_id):
					var shape_rid = node.shape_owner_get_shape(owner_id, owner_shape_id).get_rid()
					state[node_path + "/" + str(owner_id) + "/" + str(owner_shape_id)] = {
						"state" : save_node(shape_rid, save_json),
						"id": RapierPhysicsServer2D.get_rapier_id(shape_rid)
					}
		if node is Joint2D:
			rid = node.get_rid()
		state[node_path] = {
			"state" : save_node(rid, save_json),
			"id": RapierPhysicsServer2D.get_rapier_id(rid)
		}
	var space_rid = get_viewport().world_2d.space
	state["space"] = {
		"state" : save_node(space_rid, save_json),
		"id": RapierPhysicsServer2D.get_rapier_id(space_rid)
	}
	return hash(state)

func load_state():
	var physics_nodes := get_all_physics_nodes(get_tree().current_scene)
	for node_path in physics_nodes:
		var node := get_node(node_path)
		var rid : RID
		if node is CollisionObject2D:
			rid = node.get_rid()
			for owner_id in node.get_shape_owners():
				for owner_shape_id in node.shape_owner_get_shape_count(owner_id):
					var shape_rid = node.shape_owner_get_shape(owner_id, owner_shape_id).get_rid()
					var shape_state = state[node_path + "/" + str(owner_id) + "/" + str(owner_shape_id)]
					load_node(shape_rid, JSON.parse_string(shape_state.state))
		if node is Joint2D:
			rid = node.get_rid()
		var node_state = state[node_path]
		load_node(rid, JSON.parse_string(node_state.state))
	var space_rid = get_viewport().world_2d.space
	load_node(space_rid, JSON.parse_string(state["space"].state))
