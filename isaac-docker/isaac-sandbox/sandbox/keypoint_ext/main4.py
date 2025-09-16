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

scene_path = "parking_wo_cars.usdc"
open_stage(usd_path=scene_path)
world = World()

#===============================================================================

def byd_tail_lights(tail_light_ON):
    byd_left_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=5000,
        color = (2, 0, 0),
        position=(5.14744, -2.45622, 1.2107),
        count=tail_light_ON,
        )
        
    byd_right_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=5000,
        color = (2, 0, 0),
        position=(3.42388, -3.18664, 1.2107),
        count=tail_light_ON,
        )
    
    tail_lights = [byd_left_tail, byd_right_tail]
    return tail_lights

def raptor_tail_lights(tail_light_ON):
    raptor_left_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=6000,
        color = (2, 0, 0),
        position=(-7.00888, 1.32423, 1.38825),
        count=tail_light_ON,
        )
        
    raptor_right_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=6000,
        color = (2, 0, 0),
        position=(-4.38085, 2.32818, 1.38825),
        count=tail_light_ON,
        )
    
    tail_lights = [raptor_left_tail, raptor_right_tail]
    return tail_lights

def jeep_tail_lights(tail_light_ON):
    jeep_left_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=5000,
        color = (2, 0, 0),
        position=(12.92012, 2.99712, 1.41814),
        count=tail_light_ON,
        )
        
    jeep_right_tail = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=5000,
        color = (2, 0, 0),
        position=(14.85061, 3.70736, 1.41814),
        count=tail_light_ON,
        )
    
    tail_lights = [jeep_left_tail, jeep_right_tail]
    return tail_lights

#===============================================================================

def raptor_rev_lights(rev_light_ON):
    raptor_left_rev = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=20000,
        color = (2, 2, 2),
        position=(-6.76709, 1.8495, 1.23934),
        rotation=(90, -20, 0),
        scale = 0.075,
        count=rev_light_ON,
        )
        
    raptor_right_rev = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=20000,
        color = (2, 2, 2),
        position=(-4.68547, 2.61438, 1.23934),
        rotation=(90, 70, 0),
        scale = 0.075,
        count=rev_light_ON,
        )
    
    rev_lights = [raptor_left_rev, raptor_right_rev]
    return rev_lights

def jeep_rev_lights(rev_light_ON):
    jeep_left_rev = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=30000,
        color = (2, 2, 2),
        position=(13.05144, 3.45273, 1.25146),
        rotation=(90, -15, 0),
        scale = 0.075,
        count=rev_light_ON,
        )
        
    jeep_right_rev = rep.create.light(
        light_type="Sphere",
        temperature=6500,
        intensity=30000,
        color = (2, 2, 2),
        position=(14.54322, 3.977, 1.25146),
        rotation=(90, 40, 0),
        scale = 0.075,
        count=rev_light_ON,
        )
    
    rev_lights = [jeep_left_rev, jeep_right_rev]
    return rev_lights

#===============================================================================

# def byd_camera(tail_lights):
#     return rep.create.camera(position=rep.distribution.uniform((-4.00, -1.00, 0.40), (4.50, 1.50, 2.50)),
#                              look_at = tail_lights)

# def raptor_camera(tail_lights):
#     return rep.create.camera(position=rep.distribution.uniform((-6.20, -2.20, 0.80), (10.15, -0.80, 3.00)),
#                              look_at = tail_lights)

# def jeep_camera(tail_lights):
    return rep.create.camera(position=rep.distribution.uniform((-9.95, -2.20, 0.80), (26.00, -0.80, 3.00)),
                             look_at = tail_lights)

#===============================================================================

rep.orchestrator.set_capture_on_play(False)
random.seed(42)
rep.set_global_seed(42)

stage = omni.usd.get_context().get_stage()

car1 = rep.get.prims(path_pattern="/honda_civic")
with car1:
    rep.modify.semantics([('class', 'honda_civic')])
car2 = rep.get.prims(path_pattern="/jeep")
with car2:
    rep.modify.semantics([('class', 'jeep')])
car3 = rep.get.prims(path_pattern="/ford_raptor")
with car3:
    rep.modify.semantics([('class', 'ford_raptor')])

with rep.trigger.on_custom_event(event_name="case_x"):

    dome_light_textures = rep.create.light(light_type="Dome",
                                           texture=rep.distribution.choice([
                                            'background_textures/charolettenbrunn_park_4k.exr',
                                            'background_textures/docklands_01_4k.exr',
                                            ]))
    
    tail_light_ON = 1
    rev_light_ON = 1

    # tail_lights = byd_tail_lights(tail_light_ON)
    tail_lights = raptor_tail_lights(tail_light_ON)
    # tail_lights = jeep_tail_lights(tail_light_ON)

    rev_lights = raptor_rev_lights(rev_light_ON)
    rev_lights = jeep_rev_lights(rev_light_ON)
    
    # camera = byd_camera(tail_lights)
    # camera = raptor_camera(tail_lights)
    # camera = jeep_camera(tail_lights)

    # camera = rep.create.camera(position=rep.distribution.uniform((3.00, 0.00, 0.30), (4.50, 0.00, 1.50)), rotation=(0, 0, 90))
    # camera = rep.create.camera(position=rep.distribution.uniform((-6.00, -2.00, 0.50), (-4.00, -2.00, 1.80)), rotation=(0, 0, -90))
    camera = rep.create.camera(position=rep.distribution.uniform((-6.00, -2.00, 0.50), (-4.00, -2.00, 1.80)), look_at=car1, focal_length=10)
    # camera = rep.create.camera(position=rep.distribution.uniform((10.00, -2.00, 0.30), (13.00, -2.00, 1.50)), rotation=(0, 0, -90))

    # camera = rep.create.camera(position=rep.distribution.uniform((0, 0, 0.30), (0, 0, 1.80)), rotation=(0, 0, -90))

rp = rep.create.render_product(camera, (1920, 1080))

# Create a render product using the viewport perspective camera
# rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))

annotator1 = rep.annotators.get("semantic_segmentation", init_params={"semanticTypes": ["class"]})
annotator1.attach(rp)

writer = rep.writers.get("BasicWriter")
out_dir = os.getcwd() + "/output"
writer.initialize(output_dir = out_dir, 
                  rgb = True, bounding_box_2d_tight = True, semantic_segmentation=True
                  )

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