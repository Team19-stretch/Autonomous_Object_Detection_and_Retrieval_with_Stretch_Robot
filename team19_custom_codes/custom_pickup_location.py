import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)

        # my_node's main logic goes here
        self.move_to_pose({'joint_arm': 0.0}, blocking=True)
        self.move_to_pose({'joint_lift': 1.0}, blocking=True)
        self.move_to_pose({'joint_wrist_yaw': 1.0}, blocking=True)
        self.move_to_pose({'joint_wrist_pitch': -0.5}, blocking=True)
        self.move_to_pose({'joint_head_tilt': 0.0}, blocking=True)
        self.move_to_pose({'joint_head_pan': 0.0}, blocking=True)
        self.move_to_pose({'joint_gripper_finger_left': 0.5}, blocking=True)
        self.move_to_pose({'joint_gripper_finger_right': 0.5}, blocking=True)

node = MyNode()
node.main()
