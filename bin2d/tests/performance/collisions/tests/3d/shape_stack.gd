extends PhysicsPerformanceTest3D

@export var shape: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var stack_height := 8
@export var body_spacing := 0.1
@export var simulation_duration := 8
		
func test_description() -> String:
	return """Checks the stability of the RigidBody Simulation. The stack should be stable and the RigidBody
	should sleep after the [simulation_duration]
	"""
	
func test_name() -> String:
	return "RigidBody3D | testing the %s stack stability" % [PhysicsTest3D.shape_name(shape)]


func test_start() -> void:
	$Camera.current = true
	
	var timer = Timer.new()
	timer.wait_time = simulation_duration
	timer.process_callback = Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(self.test_completed)
	add_child(timer)
	timer.start()
	
	# Ground
	var ground_body := StaticBody3D.new()
	var col_shape = CollisionShape3D.new()
	col_shape.shape = WorldBoundaryShape3D.new()
	ground_body.add_child(col_shape)
	add_child(ground_body)
	
	var stack = Node3D.new()
	stack.position = Vector3(0, -0.5, 0)
	
	var bodies_array : Array[RigidBody3D] = []
	for i in range(stack_height):
		var body := RigidBody3D.new()
		var body_col: Node3D = PhysicsTest3D.get_default_collision_shape(shape)
		body.add_child(body_col)
		
		# Spawn the body
		body.position = Vector3(0, (1 + body_spacing) * (i+1), 0)
		bodies_array.append(body)
		stack.add_child(body)

	add_child(stack)
	super() # launch the test
