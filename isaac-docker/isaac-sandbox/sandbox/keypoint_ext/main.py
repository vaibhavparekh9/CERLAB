#===============================================================================

from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False}) 

#===============================================================================

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import omni.replicator.core as rep
import omni.usd

import os
import random
import numpy as np

#===============================================================================

# scene_path = "scene.usd"
# open_stage(usd_path=scene_path)
world = World()

#===============================================================================

rep.orchestrator.set_capture_on_play(False)
random.seed(42)
rep.set_global_seed(42)

stage = omni.usd.get_context().get_stage()

with rep.trigger.on_custom_event(event_name="case_x"):

    dome_light=rep.create.light(light_type="Dome", color=(1, 1, 1))
    
    vehicle = rep.create.from_usd(usd="toyota_fork/Toyota_forklift_8FD25.usdc", # position=(0,0,0), # rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), 
                                #   semantics=[("class", "forklift")]
                                )
    
    kp_front_left = rep.get.prims(path_pattern="/kp_front_left")
    with kp_front_left:
        rep.modify.semantics([('class', 'front_left')])

    kp_front_right = rep.get.prims(path_pattern="/kp_front_right")
    with kp_front_right:
        rep.modify.semantics([('class', 'front_right')])
    
    kp_tip_left = rep.get.prims(path_pattern="/kp_tip_left")
    with kp_tip_left:
        rep.modify.semantics([('class', 'tip_left')])
    
    kp_tip_right = rep.get.prims(path_pattern="/kp_tip_right")
    with kp_tip_right:
        rep.modify.semantics([('class', 'tip_right')])

    camera = rep.create.camera(position=rep.distribution.uniform((-10,-8,1), (3,8,4)), look_at=vehicle, focal_length=10)
    

rp = rep.create.render_product(camera, (1920, 1080))

annotator1 = rep.annotators.get("bounding_box_2d_tight", init_params={"semanticTypes": ["class"]}) 
annotator1.attach(rp)

writer = rep.writers.get("BasicWriter")
out_dir = os.getcwd() + "/output"
writer.initialize(output_dir = out_dir, 
                  rgb = True, bounding_box_2d_tight = True, semantic_segmentation = True)

writer.attach(rp)

#===============================================================================

for _ in range(60):
    simulation_app.update()

#===============================================================================    

for i in range(10):
    print(f"Step {i}")

    rep.utils.send_og_event(event_name="case_x")       

    rep.orchestrator.step(rt_subframes=20)

writer.detach()
annotator1.detach()
rp.destroy()
rep.orchestrator.wait_until_complete()

#===============================================================================

simulation_app.close()

#===============================================================================