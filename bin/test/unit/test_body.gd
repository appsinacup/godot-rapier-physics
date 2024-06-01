extends GutTest

func _ready():
	var body = PhysicsServer2D.body_create()
	get_viewport()
	PhysicsServer2D.body_attach_canvas_instance_id()
	var body_canvas_instance_id = PhysicsServer2D.body_get_canvas_instance_id(body)
	print("body canvas instance id is ", body_canvas_instance_id)
	PhysicsServer2D.body_set_space(body, body)
	print("body space rid is ",PhysicsServer2D.body_get_space(body))
	PhysicsServer2D.free_rid(body)
	#assert_eq(1, 2, "Should fail.  1 != 2")
