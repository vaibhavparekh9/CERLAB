# main.py

# ===============================================
from isaacsim.simulation_app import SimulationApp

sim_app = SimulationApp({'sync_loads': True, 'headless': False, 'multi_gpu': False}) 
# ===============================================

from  isaacsim.core.api import World

from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.objects import VisualCone

import numpy as np
import random
import cv2

# ===============================================
world = World(stage_units_in_meters = 1) 
world.initialize_physics()
world.scene.add_default_ground_plane()
world.reset()
# ===============================================

cube1 = VisualCuboid(prim_path = '/world/visual_cube1', name = 'cube1', position = np.array([1, 0 ,0.05]),
                     size = 0.1, color = np.array([1,1,0]))
cube2 = VisualCuboid(prim_path = '/visual_cube2', name = 'cube2', position = np.array([-1, 0 ,0.05]),
                     size = 0.1, color = np.array([1,1,0]))
cube3 = VisualCuboid(prim_path = '/visual_cube3', name = 'cube3', position = np.array([-1, 0 ,2]),
                     size = 0.1, color = np.array([1,1,0]))
cube4 = VisualCuboid(prim_path = '/visual_cube4', name = 'cube4', position = np.array([1, 0 ,2]),
                     size = 0.1, color = np.array([1,1,0]))
cube5 = VisualCuboid(prim_path = '/visual_cube5', name = 'cube5', position = np.array([1, -2 ,0.05]),
                     size = 0.1, color = np.array([1,0,0]))
cube6 = VisualCuboid(prim_path = '/visual_cube6', name = 'cube6', position = np.array([-1, -2 ,0.05]),
                     size = 0.1, color = np.array([1,0,0]))
cube7 = VisualCuboid(prim_path = '/visual_cube7', name = 'cube7', position = np.array([1, -2 ,2]),
                     size = 0.1, color = np.array([1,0,0]))
cube8 = VisualCuboid(prim_path = '/visual_cube8', name = 'cube8', position = np.array([-1, -2 ,2]),
                     size = 0.1, color = np.array([1,0,0]))

# ===============================================
camera_frnt = Camera(prim_path = '/camera_front', position = np.array([0, 10, 1]), 
                resolution = (720, 720), orientation = rot_utils.euler_angles_to_quats(np.array([0, 0, -90]), degrees = True))
camera_frnt.initialize()

camera_top = Camera(prim_path = '/camera_top', position = np.array([0, -1, 12]), 
                resolution = (720, 720), orientation = rot_utils.euler_angles_to_quats(np.array([0, 90, -90]), degrees = True))
camera_top.initialize()
# ===============================================

for _ in range(5):
    sim_app.update()

for i in range(60):

    # ========= randomize shape =========
    cone_height = random.uniform(0.1, 0.5)
    cone_rad = cone_height/2

    cone_roll = random.randrange(-180, 180, 1)
    cone_pitch = random.randrange(-180, 180, 1)
    cone_yaw = random.randrange(-180, 180, 1)
    orient = np.array([cone_roll, cone_pitch, cone_yaw])

    cone_x = random.uniform(-1, 1)
    cone_y = random.uniform(0, -2)
    cone_z = random.uniform(cone_rad, 2 - cone_rad)
    posit = np.array([cone_x, cone_y, cone_z])

    cone = VisualCone(prim_path = '/visual_cone', name = 'cone', position = posit,
                  height = cone_height, radius = cone_rad, color = np.array([0,1,1]),
                  orientation = rot_utils.euler_angles_to_quats(orient, degrees = True)
                  )
    
    # ========= capture image =========
    camera_frame_frnt = camera_frnt.get_rgb()
    camera_frame_frnt = cv2.cvtColor(camera_frame_frnt, cv2.COLOR_RGB2BGR)
    camera_frame_frnt = cv2.putText(camera_frame_frnt, 'Front View', (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 255, 255), 1)
    cv2.imwrite(f"/sandbox/rand_poseGen/cam_front/frnt_frm {i}.png", camera_frame_frnt)

    camera_frame_top = camera_top.get_rgb()
    camera_frame_top = cv2.cvtColor(camera_frame_top, cv2.COLOR_RGB2BGR)
    camera_frame_top = cv2.putText(camera_frame_top, 'Top View', (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (255, 255, 255), 1)
    cv2.imwrite(f"/sandbox/rand_poseGen/cam_top/top_frm {i}.png", camera_frame_top)

    camera_frame_combined = cv2.hconcat([camera_frame_frnt, camera_frame_top])
    cv2.imwrite(f"/sandbox/rand_poseGen/cam_combined/comb_frm {i}.png", camera_frame_combined)


    sim_app.update()

sim_app.close() 