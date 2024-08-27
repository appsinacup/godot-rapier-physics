extends RefCounted

## Expected format of config is a JSON file like:
##	{
##		"plugin_name": "plugin_updater",
##		"secs_before_check_for_update": 5,
##		"github_repo": "myyk/godot-plugin-updater",
##		"editor_plugin_meta": "PluginUpdaterEditorPlugin"
##	}

const PLUGIN_NAME: String = "godot-rapier2d" # This is replaced when code is generated
const PLUGIN_MAKER_CONFIG_PATH = "res://plugin-updater.json"
const PLUGIN_USER_CONFIG_PATH_FORMAT = "res://addons/%s/generated/updater/plugin-updater.json"
const PLUGIN_SKIP_CONFIG_PATH = "res://.godot/plugin-updater-skip.json"

static func get_user_config() -> Dictionary:
	return _get_config(PLUGIN_USER_CONFIG_PATH_FORMAT % PLUGIN_NAME)

static func get_repo_config() -> Dictionary:
	return _get_config(PLUGIN_MAKER_CONFIG_PATH)

static func _get_config(path: String) -> Dictionary:
	var config = {
		secs_before_check_for_update = 5,
	}
	
	if !FileAccess.file_exists(path):
		push_error("plugin-updater: Needs a config at " + path)
		
	var file: FileAccess = FileAccess.open(path, FileAccess.READ)
	config.merge(JSON.parse_string(file.get_as_text()), true)
	
	return config

# Skip config is a JSON file like:
# {
#	"plugin_name_1": "1.0.0"
#	"plugin_name_2": "1.2.3"
# }
static func get_skip_config() -> Dictionary:
	if !FileAccess.file_exists(PLUGIN_SKIP_CONFIG_PATH):
		return {}
	
	var file: FileAccess = FileAccess.open(PLUGIN_SKIP_CONFIG_PATH, FileAccess.READ)
	if file == null:
		push_error("plugin-updater: Could not open file at " + PLUGIN_SKIP_CONFIG_PATH)
		return {}
	return JSON.parse_string(file.get_as_text())

static func save_skip_config(config: Dictionary) -> Error:
	var file: FileAccess = FileAccess.open(PLUGIN_SKIP_CONFIG_PATH, FileAccess.WRITE)
	if file == null:
		push_error("plugin-updater: Could not open file at " + PLUGIN_SKIP_CONFIG_PATH)
		return FileAccess.get_open_error()
	
	file.store_string(JSON.stringify(config))
	file.close()
	return OK
