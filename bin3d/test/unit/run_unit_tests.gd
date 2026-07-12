extends Node

# CI entry point for the res://test/unit unit tests.
#
# The unit-test scripts (test_body.gd, test_space.gd, the joint tests, ...) run their
# assertions synchronously inside their own _ready(). Node _ready() propagates bottom-up,
# so by the time this root node's _ready() runs, every unit-test script beneath it has
# already executed all of its assertions.
#
# A failed assert() either terminates the process (so the SUCCESS sentinel below is never
# printed) or logs an error to stderr. CI therefore fails the run if EITHER the sentinel is
# missing OR an assertion/script error appears in the log.
func _ready() -> void:
	print("UNIT TESTS STATUS: SUCCESS")
	get_tree().quit(0)
