import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
robot.arm.move_to(0.0)
robot.push_command()
