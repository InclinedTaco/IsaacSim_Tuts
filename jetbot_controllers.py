from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.controllers import BaseController
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot

import numpy as np
import carb

class CoolController(BaseController):
    def __init__(self):
        super().__init__(name="my_cool_controller")
        # An open loop controller that uses a unicycle model
        self._wheel_radius = 0.03
        self._wheel_base = 0.1125
        return

    def forward(self, command):
        # command will have two elements, first element is the forward velocity
        # second element is the angular velocity (yaw only).
        joint_velocities = [0.0, 0.0]
        joint_velocities[0] = ((2 * command[0]) - (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        joint_velocities[1] = ((2 * command[0]) + (command[1] * self._wheel_base)) / (2 * self._wheel_radius)
        # A controller has to return an ArticulationAction
        return ArticulationAction(joint_velocities=joint_velocities)

world=World()
world.scene.add_default_ground_plane()
assets_root_path= get_assets_root_path()

if assets_root_path is None:
    carb.log_error("Could not find Nucleus Server with /Isaac folder")

asset_path=assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

add_reference_to_stage(asset_path, "/World/cool_robot")
cool_robot=world.scene.add(WheeledRobot(prim_path="/World/cool_robot", name="cool_robot",wheel_dof_names=["left_wheel_joint","right_wheel_joint"],create_robot=True, usd_path=asset_path))
controller=WheelBasePoseController(name="cool_controller",open_loop_wheel_controller=CoolController())
print(f"Degrees of Freedom before reset: {cool_robot.num_dof}")

# fancy_cube=world.scene.add(DynamicCuboid(prim_path="/World/random_cube",
#                                  name="fancy_cube",
#                                  position=np.array([0,0,1.0]),
#                                  scale=np.array([0.1,0.1,0.1]),
#                                     color=np.array([1.0,1.0,0.0]),))

world.reset()

for i in range(500000):
    position,orientation=cool_robot.get_world_pose()
    goal_position=np.array([3.0,-3.0])
    cool_robot.apply_wheel_actions(controller.forward(start_position=position, start_orientation=orientation, goal_position=goal_position))
    current_position, current_orientation = cool_robot.get_world_pose()
    print(f"Error to goal: {np.linalg.norm(goal_position - current_position[:2])}")

    world.step(render=True)

simulation_app.close()
