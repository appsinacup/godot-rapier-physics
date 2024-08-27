@tool
extends Node

## Credits: Mostly copied from https://github.com/MikeSchulze/gdUnit4/blob/99b7c323f443e5fcc67f9a79b4df532727e8986f/addons/gdUnit4/src/update/GdUnitUpdateClient.gd

signal request_completed(response)

@export var github_repo = ""

class HttpResponse:
	var _code :int
	var _body :PackedByteArray


	func _init(code_ :int, body_ :PackedByteArray):
		_code = code_
		_body = body_

	func code() -> int:
		return _code

	func response() -> Variant:
		var test_json_conv := JSON.new()
		test_json_conv.parse(_body.get_string_from_utf8())
		return test_json_conv.get_data()

	func body() -> PackedByteArray:
		return _body

var _http_request :HTTPRequest = HTTPRequest.new()

func _ready():
	add_child(_http_request)
	_http_request.connect("request_completed", Callable(self, "_on_request_completed"))


func _notification(what):
	if what == NOTIFICATION_PREDELETE:
		if is_instance_valid(_http_request):
			_http_request.queue_free()


func request_latest_version() -> HttpResponse:
	var error = _http_request.request("https://api.github.com/repos/%s/tags" % github_repo)
	if error != OK:
		var message = "request_latest_version failed: %d" % error
		return HttpResponse.new(error, message.to_utf8_buffer())
	return await self.request_completed


func request_releases() -> HttpResponse:
	var error = _http_request.request("https://api.github.com/repos/%s/releases" % github_repo)
	if error != OK:
		var message = "request_releases failed: %d" % error
		return HttpResponse.new(error, message.to_utf8_buffer())
	return await self.request_completed


func request_image(url :String) -> HttpResponse:
	var error = _http_request.request(url)
	if error != OK:
		var message = "request_image failed: %d" % error
		return HttpResponse.new(error, message.to_utf8_buffer())
	return await self.request_completed


func request_zip_package(url :String, file :String) -> HttpResponse:
	_http_request.set_download_file(file)
	var error = _http_request.request(url)
	if error != OK:
		var message = "request_zip_package failed: %d" % error
		return HttpResponse.new(error, message.to_utf8_buffer())
	return await self.request_completed


func _on_request_completed(_result :int, response_code :int, _headers :PackedStringArray, body :PackedByteArray):
	if _http_request.get_http_client_status() != HTTPClient.STATUS_DISCONNECTED:
		_http_request.set_download_file("")
	request_completed.emit(HttpResponse.new(response_code, body))
