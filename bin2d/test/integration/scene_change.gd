extends Node2D

@export var rapier_state_manager : StateManager
@export var label: Label

var saved_state: Variant

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	await run_for_1s()
	await save_state()
	await run_for_1s()
	await reload_state()
	await run_for_1s()
	get_tree().reload_current_scene()

func save_state():
	# Disable physics server for save
	PhysicsServer2D.set_active(false)
	# Save the state
	var space_rid = get_world_2d().space;
	saved_state = rapier_state_manager.export_state(space_rid, "Json")
	var as_json: String = saved_state
	var hashed = as_json.sha256_text()
	label.text = "Saving and waiting 1s... " + str(hashed)
	# Wait 1 second to view all
	await get_tree().create_timer(1).timeout
	# Re-enable it
	PhysicsServer2D.set_active(true)

func reload_state():
	# Disable physics server for load
	PhysicsServer2D.set_active(false)
	var space_rid = get_world_2d().space;
	rapier_state_manager.import_state(space_rid, saved_state)
	label.text = "Loading and waiting 1s... "
	await get_tree().create_timer(1).timeout
	PhysicsServer2D.set_active(true)

func run_for_1s():
	PhysicsServer2D.set_active(true)
	label.text = "Running for 1s..."
	# Run 1 second
	await get_tree().create_timer(1).timeout
	PhysicsServer2D.set_active(false)
	var space_rid = get_world_2d().space;
	var state = rapier_state_manager.export_state(space_rid, "Json")
	var as_json: String = state
	var hashed = as_json.sha256_text()
	print(hashed)
	label.text = "Hash for 1s... " + str(hashed)
	await get_tree().create_timer(1).timeout
