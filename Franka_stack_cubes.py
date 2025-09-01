from isaacsim import SimulationApp
simulation_app=SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
import numpy as np

class FrankaPlaying(BaseTask):
    def __init__(self, name):
        super().__init__(name=name, offset=None)
        

        self._goal_positions = [
            np.array([0.5, -0.3, 0.0515 * 0.5]),
            np.array([0.5, -0.3, 0.0515 * 1.5]),
            np.array([0.5, -0.3, 0.0515 * 2.5]),
            np.array([0.5, -0.3, 0.0515 * 3.5]),
            np.array([0.5, -0.3, 0.0515 * 4.5]),
        ]
        

        self.cubes = []
        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        

        cube_props = [
            {"name": "fancy_cube1", "position": [0.3, 0.3, 0.3], "color": [0, 0, 1.0]},
            {"name": "fancy_cube2", "position": [0.3, -0.3, 0.3], "color": [1.0, 0, 0]},
            {"name": "fancy_cube3", "position": [-0.3, 0.3, 0.3], "color": [0, 1.0, 0]},
            {"name": "fancy_cube4", "position": [-0.3, 0.4, 0.3], "color": [1.0, 1.0, 0]},
            {"name": "fancy_cube5", "position": [-0.3, -0.4, 0.3], "color": [0.5, 0.5, 0.5]},
        ]
        

        for i, prop in enumerate(cube_props):
            cube = scene.add(DynamicCuboid(
                prim_path=f"/World/{prop['name']}",
                name=prop['name'],
                position=np.array(prop['position']),
                scale=np.array([0.0515, 0.0515, 0.0515]),
                color=np.array(prop['color'])
            ))
            self.cubes.append(cube)

        self._franka = scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        return

    def get_observations(self):
        observations = { self._franka.name: {"joint_positions": self._franka.get_joint_positions()} }
        for i, cube in enumerate(self.cubes):
            pos, _ = cube.get_world_pose()
            observations[cube.name] = {
                "position": pos,
                "goal_position": self._goal_positions[i]
            }
        return observations

    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        return



world = World()
task = FrankaPlaying(name="cool_task")
world.add_task(task)

world.reset()
franka = world.scene.get_object("fancy_franka")
controller = PickPlaceController(name="pick_and_place_controller", gripper=franka.gripper, robot_articulation=franka)

# This is now an index for our lists
current_cube_index = 0
num_cubes = len(task.cubes)

for i in range(50000):
    # If all cubes have been moved, do nothing
    if current_cube_index >= num_cubes:
        world.step(render=True)
        continue

    # Get the current cube and its name/goal from our lists
    current_cube = task.cubes[current_cube_index]
    current_cube_name = current_cube.name
    observations = world.get_observations()


    actions = controller.forward(
        picking_position=observations[current_cube_name]["position"],
        placing_position=observations[current_cube_name]["goal_position"],
        current_joint_positions=observations["fancy_franka"]["joint_positions"],
    )
    franka.apply_action(actions)

    if controller.is_done():
        print(f"Finished moving cube {current_cube_index + 1}. Starting next cube.")
        current_cube_index += 1  # Move to the next cube
        controller.reset()

    world.step(render=True)

simulation_app.close()
