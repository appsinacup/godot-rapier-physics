extends Node2D

@export var rigidbody: RigidBody2D
@onready var space := get_viewport().world_2d.space

func is_physics_object(node: Node) -> bool:
	return node is CollisionObject2D or \
		node is CollisionShape2D or \
		node is CollisionPolygon2D or \
		node is Joint2D

func get_all_physics_nodes(p_node: Node, path: String = "") -> Array[String]:
	var results : Array[String] = []
	if path == "" && is_physics_object(p_node):
		results.append(path + p_node.name)
	path += p_node.name + "/"
	for node in p_node.get_children():
		if is_physics_object(node):
			results.append(path + node.name)
		if node.get_child_count() > 0:
			results.append_array(get_all_physics_nodes(node, path))
	return results

func save_all_physics_objects(p_node: Node):
	var physics_nodes := get_all_physics_nodes(p_node)
	var physics_map = {}
	for node_path in physics_nodes:
		var node = get_node(node_path)
		physics_map[node_path] = RapierPhysicsServer2D.get_handle(node)
		if node is CollisionObject2D:
			RapierPhysicsServer2D.body_export_json(node.get_rid())

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	print(get_all_physics_nodes(get_tree().current_scene))
	FileAccess.open("user://rigidbody.json", FileAccess.WRITE).store_string(RapierPhysicsServer2D.body_export_json(rigidbody.get_rid()))
	FileAccess.open("user://space.json", FileAccess.WRITE).store_string(RapierPhysicsServer2D.space_export_json(space))
	var ctx = HashingContext.new()
	ctx.start(HashingContext.HASH_SHA256)
	ctx.update(RapierPhysicsServer2D.body_export_binary(rigidbody.get_rid()))
	var res = ctx.finish()
	# Print the result as hex string and array.
	print(res.hex_encode())
	await get_tree().create_timer(1.0).timeout
	get_tree().reload_current_scene()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	pass
