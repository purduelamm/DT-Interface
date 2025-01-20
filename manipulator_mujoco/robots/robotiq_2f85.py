import os
from manipulator_mujoco.robots.gripper import Gripper

_2f85_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/robotiq_2f85/2f85.xml',
)

_JOINT = 'right_coupler_joint'

_ACTUATOR = 'fingers_actuator'

class RQ_2f85(Gripper):
    def __init__(self, name: str = None):
        super().__init__(_2f85_XML, _JOINT, _ACTUATOR, name)