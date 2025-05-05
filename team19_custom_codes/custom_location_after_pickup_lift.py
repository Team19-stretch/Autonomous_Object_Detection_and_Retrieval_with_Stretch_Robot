import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
robot.lift.move_to(0.2)
robot.push_command()
