extends PhysicsUnitTest2D

func test_description() -> String:
  return &"Checks that [Faucet2D] fluids can iterate on their data"

func test_name() -> String:
  return &"Faucet2D | access metadata"

func test_start() -> void:

  var check:= func(fluid: Faucet2D, monitor: Monitor):
    var positions := fluid.points
    var accelerations := fluid.get_accelerations()
    var velocities := fluid.get_velocities()
    var remaining_lifetimes := fluid.get_remaining_times()
    
    if positions.size() != accelerations.size():
      monitor.failed("The number of positions (%d) is different from the number of accelerations (%d)" % [positions.size(), accelerations.size()])
      return
    
    if positions.size() != velocities.size():
      monitor.failed("The number of positions (%d) is different from the number of velocities (%d)" % [positions.size(), velocities.size()])
      return

    if positions.size() != remaining_lifetimes.size():
      monitor.failed("The number of positions (%d) is different from the number of remaining lifetimes (%d)" % [positions.size(), remaining_lifetimes.size()])
      return
    
  self.create_generic_manual_monitor($Faucet2D, check, 5, false)
