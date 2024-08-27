@tool
extends Window

## Updater heaviliy inspired by GDUnit4's updater, but decoupled completely from GDUnit. Also did not
## include all the patching that included since it seemed to complicated to include for most projects.

signal updated

var config = UpdaterConfig.get_user_config()

var spinner_icon = "res://addons/%s/generated/updater/spinner.tres" % config.plugin_name

# Using this style of import avoids polluting user's namespaces
const UpdaterConfig = preload("updater_config.gd")
const SemVer = preload("updater_sem_ver.gd")
const HttpClient = preload("updater_http_client.gd")
const MarkDownReader = preload("updater_markdown_reader.gd")

const TEMP_FILE_NAME = "user://temp.zip"

@onready var _md_reader: MarkDownReader = MarkDownReader.new()
@onready var _http_client: HttpClient = $HttpClient
@onready var _header: Label = $Panel/GridContainer/PanelContainer/header
@onready var _content: RichTextLabel = $Panel/GridContainer/PanelContainer2/ScrollContainer/MarginContainer/content
@onready var _update_button: Button = $Panel/GridContainer/Panel/HBoxContainer/update

var _latest_version: SemVer
var _download_zip_url: String

func _ready():
	hide()
	_http_client.github_repo = config.github_repo
	
	title = "%s Plugin Update" % config.plugin_name
	var plugin :EditorPlugin = Engine.get_meta(config.editor_plugin_meta)
	
	# wait a bit to allow the editor to initialize itself
	await Engine.get_main_loop().create_timer(float(config.secs_before_check_for_update)).timeout
	
	_check_for_updater()

func _check_for_updater():
	var response = await _http_client.request_latest_version()
	if response.code() != 200:
		push_warning("Update information cannot be retrieved from GitHub! \n %s %s" % [response.response(), response.code()])
		return
	_latest_version = extract_latest_version(response)
	var current_version := extract_current_version()

	# if the current version is less than or equal to the skip version, skip the update
	var skip_config = UpdaterConfig.get_skip_config()
	if config.plugin_name in skip_config:
		var skip_version: SemVer = SemVer.parse(skip_config[config.plugin_name])
		if !_latest_version.is_greater(skip_version):
			return

	# if same version exit here no update need
	if _latest_version.is_greater(current_version):
		_download_zip_url = extract_zip_url(response)
		_header.text = "Current version '%s'. A new version '%s' is available" % [current_version, _latest_version]
		await show_update()

func show_update() -> void:
	message_h4("\n\n\nRequest release infos ... [img=24x24]%s[/img]" % spinner_icon, Color.SNOW)
	popup_centered_ratio(.5)
	var content :String

	var response: HttpClient.HttpResponse = await _http_client.request_releases()
	if response.code() == 200:
		content = await extract_releases(response, extract_current_version())
	else:
		message_h4("\n\n\nError checked request available releases!", Color.RED)
		return

	# finally force rescan to import images as textures
	if Engine.is_editor_hint():
		await rescan()
	message(content, Color.WHITE_SMOKE)
	_update_button.set_disabled(false)

func rescan() -> void:
	if Engine.is_editor_hint():
		if OS.is_stdout_verbose():
			prints(".. reimport release resources")
		var fs := EditorInterface.get_resource_filesystem()
		fs.scan()
		while fs.is_scanning():
			if OS.is_stdout_verbose():
				progress_bar(fs.get_scanning_progress() * 100 as int)
			await Engine.get_main_loop().process_frame
		await Engine.get_main_loop().process_frame
	await Engine.get_main_loop().create_timer(1).timeout

func extract_current_version() -> SemVer:
	var config_file = ConfigFile.new()
	config_file.load('addons/%s/plugin.cfg' % config.plugin_name)
	return SemVer.parse(config_file.get_value('plugin', 'version'))

static func extract_latest_version(response: HttpClient.HttpResponse) -> SemVer:
	var body :Array = response.response()
	return SemVer.parse(body[0]["name"])

func extract_current_flavour() -> String:
	var config_file = ConfigFile.new()
	config_file.load('addons/%s/plugin.cfg' % config.plugin_name)
	return config_file.get_value('plugin', 'flavour')

func extract_zip_url(response: HttpClient.HttpResponse) -> String:
	var flavour := extract_current_flavour()
	return 'https://github.com/%s/releases/download/%s/%s.zip' % [config.github_repo, _latest_version, flavour]

func extract_releases(response: HttpClient.HttpResponse, current_version) -> String:
	await get_tree().process_frame
	var result := ""
	for release in response.response():
		if SemVer.parse(release["tag_name"]).equals(current_version):
			break
		var release_description :String = release["body"]
		result += await _md_reader.to_bbcode(release_description)
	return result

func message_h4(message :String, color :Color, clear := true) -> void:
	if clear:
		_content.clear()
	_content.append_text("[font_size=36]%s[/font_size]" % _colored(message, color))

func message(message :String, color :Color) -> void:
	_content.clear()
	_content.append_text(_colored(message, color))

func progress_bar(p_progress :int, p_color :Color = Color.POWDER_BLUE):
	if p_progress < 0:
		p_progress = 0
	if p_progress > 100:
		p_progress = 100
	printraw("scan [%-50s] %-3d%%\r" % ["".lpad(int(p_progress/2.0), "#").rpad(50, "-"), p_progress])

func _colored(message :String, color :Color) -> String:
	return "[color=#%s]%s[/color]" % [color.to_html(), message]

func _skip_update():
	# Store a setting in the config.
	var skip_config = UpdaterConfig.get_skip_config() # Read this again in case it was changed
	skip_config[config.plugin_name] = str(_latest_version)
	# Write the config into the addons dir so it gets removed on update
	UpdaterConfig.save_skip_config(skip_config)

func _on_update_pressed():
	_update_button.set_disabled(true)
	
	var updater_http_request = HTTPRequest.new()
	updater_http_request.accept_gzip = true
	add_child(updater_http_request)
	message_h4("\n\n\nStaring download %s ... [img=24x24]%s[/img]" % [_download_zip_url, spinner_icon], Color.SNOW)

	updater_http_request.request_completed.connect(_on_http_request_request_completed)
	updater_http_request.request(_download_zip_url)

func _on_http_request_request_completed(result: int, response_code: int, headers: PackedStringArray, body: PackedByteArray) -> void:
	if result != HTTPRequest.RESULT_SUCCESS:
		message_h4("\n\n\nError downloading update!", Color.RED)
		return
	message_h4("\n\n\nSuccesfuly downloaded release. Saving file to user://temp.zip", Color.SNOW)

	# Save the downloaded zip
	var zip_file: FileAccess = FileAccess.open(TEMP_FILE_NAME, FileAccess.WRITE)
	zip_file.store_buffer(body)
	zip_file.close()

	OS.move_to_trash(ProjectSettings.globalize_path("res://addons/%s" % config.plugin_name))

	var zip_reader: ZIPReader = ZIPReader.new()
	zip_reader.open(TEMP_FILE_NAME)
	var files: PackedStringArray = zip_reader.get_files()

	var base_path = files[1]
	# Remove archive folder
	files.remove_at(0)
	# Remove assets folder
	files.remove_at(0)
	
	for path in files:
		var new_file_path: String = path.replace(base_path, "")
		if path.ends_with("/"):
			DirAccess.make_dir_recursive_absolute("res://addons/%s" % new_file_path)
		else:
			var file: FileAccess = FileAccess.open("res://addons/%s" % new_file_path, FileAccess.WRITE)
			file.store_buffer(zip_reader.read_file(path))

	zip_reader.close()
	DirAccess.remove_absolute(TEMP_FILE_NAME)

	updated.emit()
	print("Finished Updating.")
	hide()

func _on_close_pressed():
	hide()
	if $Panel/GridContainer/Panel/HBoxContainer/skip_update.button_pressed:
		_skip_update()

func _on_content_meta_clicked(meta :String):
	var properties = str_to_var(meta)
	if properties.has("url"):
		OS.shell_open(properties.get("url"))

func _on_content_meta_hover_started(meta :String):
	var properties = str_to_var(meta)
	if properties.has("tool_tip"):
		_content.set_tooltip_text(properties.get("tool_tip"))

func _on_content_meta_hover_ended(meta):
	_content.set_tooltip_text("")


func _on_close_requested() -> void:
	_on_close_pressed()
