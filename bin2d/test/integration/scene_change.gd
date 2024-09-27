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
		return RapierPhysicsServer2D.export_json(rid)
	else:
		return RapierPhysicsServer2D.export_binary(rid)

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
					state[RapierPhysicsServer2D.get_handle(shape_rid)] = {
						"state" : save_node(shape_rid, save_json),
						"path": node_path,
						"owner_id": owner_id
					}
		if node is Joint2D:
			rid = node.get_rid()
		print("Processing ", node_path, " with handle ", RapierPhysicsServer2D.get_handle(rid))
		state[RapierPhysicsServer2D.get_handle(rid)] = {
			"state" : save_node(rid, save_json),
			"path": node_path
		}
	var space_rid = get_viewport().world_2d.space
	state[RapierPhysicsServer2D.get_handle(space_rid)] = {
		"state" : save_node(space_rid, save_json),
		"path": "space"
	}
	return hash(state)


# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print(save_state(false))
	FileAccess.open("user://save.json", FileAccess.WRITE).store_string(JSON.stringify(state, " "))
	# Print the result as hex string and array.
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
