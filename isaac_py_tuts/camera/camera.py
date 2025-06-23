# camera.py

# ==================================================================
from isaacsim.simulation_app import SimulationApp

sim_app = SimulationApp({"sync_loads": True, "headless": False, "multi_gpu": False})
# ==================================================================
import numpy as np
import matplotlib.pyplot as plt

from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api.objects import VisualCuboid
# ==================================================================


world   = World(stage_units_in_meters = 1)
world.initialize_physics()
world.scene.add_default_ground_plane()
world.reset()

fancy_cube = world.scene.add(VisualCuboid
                             (prim_path="/visual_cube", 
                              name="fancy_cube", 
                              position=np.array([-0.5, 0, 0.5]), 
                              scale=np.array([1,1,1]), 
                              #size = 0.5,
                              color=np.array([1, 1, 0]), 
                              ))

cam = Camera(
    prim_path="/World/camera",
    position=np.array([-0.5,-20,0.5]),
    resolution=(640, 480),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 90]), degrees=True),
)
world.scene.add(cam)
cam.initialize()

# ==================================================================
from omni.kit.viewport.utility import get_active_viewport_window
import os                                  
import cv2

out_dir = "/home/vsparekh/isaac_py_tuts/src/camera/frames"
os.makedirs(out_dir, exist_ok=True)
frame_id = 0

# ==================================================================

# captured = False
# while sim_app.is_running() and not captured:
#     sim_app.update()

#     img = cam.get_rgb()    # H×W×3 uint8 RGB

#     # convert RGB→BGR for OpenCV
#     bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

#     # write to disk (overwrites each run)
#     cv2.imwrite("capture.png", bgr)
#     print("Wrote capture.png via OpenCV")
#     captured = True
    
# sim_app.close()


for _ in range(30):  
    sim_app.update()

# ─── Grab & save one frame ─────────────────────────────────
# Now we know cam.get_rgb() will return a valid array
img = cam.get_rgb()    # H×W×3 uint8 RGB

# Convert RGB→BGR for OpenCV
import cv2, os
bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

# Write to disk (overwrites each run)
cv2.imwrite("capture.png", bgr)
print("Wrote capture.png via OpenCV")

# ─── Enter main loop (if you still want it running) ──────
while sim_app.is_running():
    sim_app.update()

sim_app.close()