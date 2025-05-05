import stretch_body.robot
robot = stretch_body.robot.Robot()
robot.startup()
robot.end_of_arm.move_to('wrist_yaw',3.9)
robot.push_command()
