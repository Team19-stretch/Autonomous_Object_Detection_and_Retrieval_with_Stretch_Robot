import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
robot.end_of_arm.move_to('wrist_pitch',0.2)
robot.push_command()
