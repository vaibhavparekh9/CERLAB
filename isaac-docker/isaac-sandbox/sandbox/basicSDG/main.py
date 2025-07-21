'''
Issues to discuss:
- not all shapes fall within frame - even from sufficient distance
- colliders not working sometimes 
- annotations 
'''

import os
import random

from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False}) 

import omni.replicator.core as rep
import omni.usd


def randomizer():
    rep.orchestrator.set_capture_on_play(False)
    random.seed(42)
    rep.set_global_seed(42)

    stage = omni.usd.get_context().get_stage()

    with rep.trigger.on_custom_event(event_name="randomizer_event"):

        # rep.create.light(light_type="Dome", color=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
        # rep.create.light(light_type="Dome", color=(1, 1, 1))

        lights = rep.create.light(
            light_type="Dome",
            texture=rep.distribution.choice([
                'charolettenbrunn_park_4k.exr',
                'docklands_01_4k.exr',
                'zwartkops_curve_morning_4k.exr']
            ))

        cube = rep.create.cube(position=rep.distribution.uniform((-5, -5, -5), (5, 5, 5)), rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), scale=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)), 
                               semantics=[("class", "cube")])
        with cube:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
            rep.physics.collider()
            rep.physics.rigid_body()

        cone = rep.create.cone(position=rep.distribution.uniform((-5, -5, -5), (5, 5, 5)), rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), scale=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)),
                               semantics=[("class", "cone")])
        with cone:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
            rep.physics.collider()
            rep.physics.rigid_body()

        cylinder = rep.create.cylinder(position=rep.distribution.uniform((-5, -5, -5), (5, 5, 5)), rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), scale=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)),
                                       semantics=[("class", "cylinder")])
        with cylinder:
            rep.randomizer.color(colors=rep.distribution.uniform((0, 0, 0), (1, 1, 1)))
            rep.physics.collider()
            rep.physics.rigid_body()

        shapes = [cube, cone, cylinder]

        # distractors
        distractor_toruses = rep.create.torus(position=rep.distribution.uniform((-5, -5, -5), (5, 5, 5)), rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), scale=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)),
                                                 count = 12)
        distractor_spheres = rep.create.sphere(position=rep.distribution.uniform((-5, -5, -5), (5, 5, 5)), rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)), scale=rep.distribution.uniform((0.1, 0.1, 0.1), (1, 1, 1)),
                                                 count = 12)


        camera = rep.create.camera(
            position=rep.distribution.uniform((-20, -20, -20), (20, 20, 20)),
            look_at = shapes
            )
    
    rp = rep.create.render_product(camera, (1280, 720))

    annotator1 = rep.annotators.get("bounding_box_2d_tight", init_params={"semanticTypes": ["class"]})
    annotator1.attach(rp)


    # Create a render product using the viewport perspective camera
    # rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))


    writer = rep.writers.get("BasicWriter")
    out_dir = os.getcwd() + "/output"
    writer.initialize(output_dir = out_dir, 
                      rgb = True, 
                      bounding_box_2d_tight = True, 
                      semantic_segmentation=True
                      )
    writer.attach(rp)


    for i in range(60):
        print(f"Step {i}")

        rep.utils.send_og_event(event_name="randomizer_event")

        rep.orchestrator.step()

        
    writer.detach()
    annotator1.detach()
    rp.destroy()
    rep.orchestrator.wait_until_complete()


randomizer()

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
