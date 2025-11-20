extends Node2D

@export var rapier_state : Rapier2DState
@export var label: Label
@export var rb: RigidBody2D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:	
	await run_for_1s()
	await save_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	get_tree().reload_current_scene()

func save_state():
	# Disable physics server for save
	PhysicsServer2D.set_active(false)
	# Save the state
	var hash_key = rapier_state.export_state()
	label.text = "Saving and waiting 1s... " + str(hash_key)
	print("Saving and waiting 1s... " + str(hash_key))
	print(rb.global_position)
	# Wait 1 second to view all
	await get_tree().create_timer(1).timeout
	# Re-enable it
	PhysicsServer2D.set_active(true)

func reload_state():
	# Disable physics server for load
	PhysicsServer2D.set_active(false)
	var hash_key = rapier_state.import_state()
	#hash_key = rapier_state.save_state()
	label.text = "Loading and waiting 1s... " + str(hash_key)
	print("Loading and waiting 1s... " + str(hash_key))
	# Wait 1 second to view all	
	print(rb.global_position)
	var _space_rid = get_viewport().world_2d.space;
	#RapierPhysicsServer2D.space_step(_space_rid, 0.0)
	#RapierPhysicsServer2D.space_flush_queries(_space_rid)
	await get_tree().create_timer(1).timeout
	PhysicsServer2D.set_active(true)

func run_for_1s():
	PhysicsServer2D.set_active(true)
	label.text = "Running for 1s..."
	# Run 1 second
	await get_tree().create_timer(1).timeout
	PhysicsServer2D.set_active(false)
	var hash_key = rapier_state.save_state()
	#label.text = "Hash for 1s... " + str(hash_key)
	#print("Hash for 1s... " + str(hash_key))
	await get_tree().create_timer(1).timeout
