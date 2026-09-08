extends PhysicsUnitTest2D

var simulation_duration := .5

@onready var bottom_box := %CollisionShape2D

func test_description() -> String:
  return &"Checks that [Faucet2D] fluids can collide with static bodies"

func test_name() -> String:
  return &"Faucet2D | testing collision"

func test_start() -> void:
  var faucet: Faucet2D = $Faucet2D
  var max_y : float = bottom_box.global_position.y - bottom_box.shape.extents.y / 2.0
  var check_particles = func(p_target: Faucet2D, _p_monitor: Monitor) -> bool:
    if p_target.points.is_empty():
      return false

    for point in p_target.points:
      if p_target.to_global(point).y > max_y:
        return false

    return true

  var monitor := create_generic_expiration_monitor(
    faucet, check_particles, null, simulation_duration
  )
  monitor.test_name = &"Fluid particles stay above the collision height"
  
