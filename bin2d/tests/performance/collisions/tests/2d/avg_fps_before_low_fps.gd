extends PhysicsPerformanceTest2D

@export var shape1: PhysicsTest2D.TestCollisionShape = PhysicsTest2D.TestCollisionShape.RECTANGLE
@export var shape2: PhysicsTest2D.TestCollisionShape = PhysicsTest2D.TestCollisionShape.RECTANGLE
@export var MINIMUM_FPS := 30 # The minimum FPS to be reached to stop the test.
@export var NB_BODY_FOR_ONE_STEP := 500 # The number of bodies to perform an average.
@export var NB_ITERATION := 3 # The number of times the test should be run 
@export var NUMBER_BODIES_PER_SPAWN := 20
@export var SPAWN_DELAY := 0.25
var MAXIMUM_STEP := 25 # The size of the table that records the steps.

var timer : Timer
var bodies = []
var current_step := 0
var label_number : Label
var swap := false # change order bodies fall

var avg_fps: float = 0
var avg_max_bodies: float = 0
var nb_tick: float = 0
var iteration := 1
var array_index := 0
var avg_result_arr := [] 

#  Waiting before starting the test to stabilise the FPS
var NB_FRAME_WAITING = 20
var waiting = true
var cpt_waiting := 0

func test_name() -> String:
	return "Average FPS per %d bodies until FPS are less than %d (%s vs %s)" % [NB_BODY_FOR_ONE_STEP, MINIMUM_FPS, PhysicsTest2D.shape_name(shape1), PhysicsTest2D.shape_name(shape2)]

func test_start() -> void:
	avg_result_arr.resize(MAXIMUM_STEP)
	avg_result_arr.fill(0)
	label_number = Label.new()
	label_number.position = TOP_LEFT + Vector2(20,60)
	label_number.set("theme_override_font_sizes/font_size", 18)
	add_child(label_number)
	
	# ground
	add_child(PhysicsTest2D.get_static_body_with_collision_shape(Rect2(BOTTOM_LEFT - Vector2(5000, 100), Vector2(Global.WINDOW_SIZE.x + 5000, 100)), TestCollisionShape.RECTANGLE, true))
	timer = Timer.new()
	timer.wait_time = SPAWN_DELAY
	timer.process_callback =Timer.TIMER_PROCESS_PHYSICS
	timer.timeout.connect(spawn_body)
	add_child(timer)
	super() # launch the test

func _physics_process(_delta: float) -> void:
	label_number.text = "Bodies: %d" % bodies.size() 
	label_number.text += "\nIteration n° %d" % iteration 
	if waiting and cpt_waiting < NB_FRAME_WAITING:
		cpt_waiting += 1
		return
	elif waiting and cpt_waiting == NB_FRAME_WAITING: 
		timer.start()
		waiting = false
		cpt_waiting = 0
		return
	
	if bodies.size() != 0:
		avg_fps += get_fps()
		nb_tick += 1
	
	if get_smoothed_fps() <= MINIMUM_FPS and not waiting:
		timer.stop()
		avg_max_bodies += bodies.size()
		if iteration == NB_ITERATION:
			var final_result = avg_result_arr.map(func(number:float) : return (number/NB_ITERATION ))
			var formated_str = ""
			var cpt = 0
			
			var max_bodies = int(avg_max_bodies / NB_ITERATION)
			var max_cpt = floor(float(max_bodies) / float(NB_BODY_FOR_ONE_STEP))
			for v in final_result:
				if is_zero_approx(v) or cpt == max_cpt:
					break
				cpt += 1
				formated_str += "[%d] = %2.f, " % [cpt * NB_BODY_FOR_ONE_STEP, v]
			formated_str = formated_str.substr(0, formated_str.length() - 3)
			register_result("FPS AVG", formated_str)
			register_result("Maximum bodies", "%d" % max_bodies)
			test_completed()
		else:
			clean()
			nb_tick = 0.0
			avg_fps = 0.0
			array_index = 0
			iteration += 1
			cpt_waiting = 0
			waiting = true

func clean() -> void:
	for body in bodies:
		remove_child(body)
	bodies.clear()
		
func spawn_body() -> void:
	var offset = (Global.WINDOW_SIZE.x - 100) / 19
	swap = not swap
	for i in range(NUMBER_BODIES_PER_SPAWN):
		var shape := shape1 if swap else shape2
		var body = _get_rigid_body(TOP_LEFT + Vector2(50 + randf() * 25 + i * offset, 0), shape)
		bodies.append(body)
		add_child(body)
	
	if bodies.size() % NB_BODY_FOR_ONE_STEP == 0:
		assert(array_index >= 0 and array_index < avg_result_arr.size()) # If this assertion fails, increase MAXIMUM_STEP
		avg_result_arr[array_index] += avg_fps/nb_tick
		array_index += 1
		nb_tick = 0.0
		avg_fps = 0.0
	
func _get_rigid_body(p_position: Vector2, p_shape: PhysicsTest2D.TestCollisionShape) -> RigidBody2D:
	var body = RigidBody2D.new()
	var shape = PhysicsTest2D.get_default_collision_shape(p_shape)
	body.add_child(shape)
	body.position = p_position
	return body
