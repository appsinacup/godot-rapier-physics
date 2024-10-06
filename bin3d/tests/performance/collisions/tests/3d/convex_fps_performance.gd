extends PhysicsPerformanceTest3D

enum GroundType {
	BOX,
	CONCAVE,
	WORLD_BOUNDARY
}

var concave_polygon_ground = preload("res://base/mesh/concave_bac_3d.tres")

@export var shape1: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var shape2: PhysicsTest3D.TestCollisionShape = PhysicsTest3D.TestCollisionShape.CONVEX_POLYGON
@export var MINIMUM_FPS := 30 # The minimum FPS to be reached to stop the test.
@export var NB_BODY_FOR_ONE_STEP := 250 # The number of bodies to perform an average.
@export var NB_ITERATION := 3 # The number of times the test should be run 
@export var NUMBER_BODIES_PER_SPAWN := 5
@export var SPAWN_DELAY := 0.35
@export var GROUND_TYPE: GroundType = GroundType.WORLD_BOUNDARY
var MAXIMUM_STEP := 50 # The size of the table that records the steps.

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
	return "Average FPS per %d bodies until FPS are less than %d (%s vs %s)" % [NB_BODY_FOR_ONE_STEP, MINIMUM_FPS, PhysicsTest3D.shape_name(shape1), PhysicsTest3D.shape_name(shape2)]

func test_start() -> void:
	$Camera.current = true
	
	avg_result_arr.resize(MAXIMUM_STEP)
	avg_result_arr.fill(0)
	label_number = Label.new()
	label_number.position = Vector2(20,60)
	label_number.set("theme_override_font_sizes/font_size", 18)
	add_child(label_number)
	
	# Ground
	var ground_body := StaticBody3D.new()
	ground_body.position = Vector3()
	if GROUND_TYPE == GroundType.BOX:
		var ground_box_shape = get_collision_shape(Vector3(100, 1, 60))
		ground_body.add_child(ground_box_shape)
	elif GROUND_TYPE == GroundType.WORLD_BOUNDARY:
		var col_shape = CollisionShape3D.new()
		col_shape.shape = WorldBoundaryShape3D.new()
		ground_body.add_child(col_shape)
	else:
		var col_shape = CollisionShape3D.new()
		col_shape.shape = concave_polygon_ground
		ground_body.add_child(col_shape)
	add_child(ground_body)
	
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
	if _warming:
		return
	swap = not swap
	for i in range(NUMBER_BODIES_PER_SPAWN):
		var shape := shape1 if swap else shape2
		var index = i + 1
		var even = index % 2 == 0
		var offset = index if even else -index
		var body = RigidBody3D.new()
		body.add_child(PhysicsTest3D.get_default_collision_shape(shape))
		body.position = Vector3(offset, 20, 0)
		bodies.append(body)
		add_child(body)

	if bodies.size() % NB_BODY_FOR_ONE_STEP == 0:
		assert(array_index >= 0 and array_index < avg_result_arr.size()) # If this assertion fails, increase MAXIMUM_STEP
		avg_result_arr[array_index] += avg_fps/nb_tick
		array_index += 1
		nb_tick = 0.0
		avg_fps = 0.0
