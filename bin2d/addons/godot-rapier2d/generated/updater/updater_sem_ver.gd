extends RefCounted

## Credits: Copied from https://github.com/MikeSchulze/gdUnit4/blob/99b7c323f443e5fcc67f9a79b4df532727e8986f/addons/gdUnit4/src/core/GdUnit4Version.gd before edits.

const VERSION_PATTERN = "[center][color=#9887c4]gd[/color][color=#7a57d6]Unit[/color][color=#9887c4]4[/color] [color=#9887c4]${version}[/color][/center]"

var _major :int
var _minor :int
var _patch :int


func _init(major :int,minor :int,patch :int):
	_major = major
	_minor = minor
	_patch = patch


static func parse(value :String):
	var regex := RegEx.new()
	regex.compile("[a-zA-Z:,-]+")
	var cleaned := regex.sub(value, "", true)
	var parts := cleaned.split(".")
	var major := parts[0].to_int()
	var minor := parts[1].to_int()
	var patch := parts[2].to_int() if parts.size() > 2 else 0
	return new(major, minor, patch)


func equals(other :) -> bool:
	return _major == other._major and _minor == other._minor and _patch == other._patch


func is_greater(other :) -> bool:
	if _major > other._major:
		return true
	if _major == other._major and _minor > other._minor:
		return true
	return _major == other._major and _minor == other._minor and _patch > other._patch


func _to_string() -> String:
	return "v%d.%d.%d" % [_major, _minor, _patch]
