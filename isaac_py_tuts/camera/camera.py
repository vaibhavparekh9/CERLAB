# camera.py
from isaacsim.simulation_app import SimulationApp

import numpy as np
import matplotlib.pyplot as plt

from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera

CONFIG = {
    "sync_loads": True,
    "headless": False,
    "multi_gpu": False
}

sim_app = SimulationApp(launch_config=CONFIG)
# world   = World(stage_units_in_meters=1.0)

# cam = Camera(
#     prim_path="/World/camera",
#     resolution=(640, 480),
#     fov=60.0,
#     sensor_rate=30.0
# )
# world.scene.add(cam)

while sim_app.is_running():
    sim_app.update()
    # frame = cam.get_frame()
    
sim_app.close()