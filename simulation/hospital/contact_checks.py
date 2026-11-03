"""Adversarial physical contact checks; executed only in the isolated test fixture."""

import json
import math
import time
from collections import Counter

import numpy as np


async def run(runtime):
    import omni.kit.app
    import omni.physx
    from pxr import PhysicsSchemaTools
    from isaacsim.core.simulation_manager import SimulationManager

    contacts = Counter()
    worst_penetration = {"wall": 0.0, "robots": 0.0}
    maximum_pose_error = 0.0
    minimum_robot_distance = math.inf
    wall_limit = 11.1600008
    max_robot3_x = -math.inf
    observed_pairs = set()

    def contact(headers, data):
        for header in headers:
            a,b = str(PhysicsSchemaTools.intToSdfPath(header.actor0)), str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            c,d = str(PhysicsSchemaTools.intToSdfPath(header.collider0)), str(PhysicsSchemaTools.intToSdfPath(header.collider1))
            pair = a+" "+b+" "+c+" "+d
            if "GroundPlane" not in pair and len(observed_pairs) < 32:
                observed_pairs.add(pair)
            kind = "wall" if "Nova_Carter_ROS_3" in pair and ("Geo_M3_BackWall2" in pair or "Geo_M3_TrimStrateSide4" in pair) else (
                "robots" if "Nova_Carter_ROS_1" in pair and "Nova_Carter_ROS_2" in pair else None)
            if kind:
                contacts[kind] += header.num_contact_data
                for index in range(header.contact_data_offset, header.contact_data_offset+header.num_contact_data):
                    worst_penetration[kind] = min(worst_penetration[kind], float(data[index].separation))

    subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact)
    report = {"passed": False}
    body_probe = runtime.query.raycast_closest((-.25,-1,.4),(0,1,0),2)
    report["chassis_collision_probe"] = {"hit":body_probe["hit"],"collider":body_probe.get("collision","")}

    async def drive(name, duration):
        nonlocal maximum_pose_error,minimum_robot_distance,max_robot3_x
        started = SimulationManager.get_simulation_time()
        deadline = time.monotonic()+60
        while SimulationManager.get_simulation_time()-started < duration:
            if time.monotonic() > deadline:
                raise RuntimeError("Contact test simulation stopped progressing")
            runtime.commands[name] = (.3,0,time.monotonic())
            await omni.kit.app.get_app().next_update_async()
            positions = runtime.articulations.get_world_poses()[0].numpy()
            for index,origin in enumerate(runtime.initial):
                # Fixture headings are zero, so local/global translation differs only by spawn.
                displayed = runtime.odom[index,:2] + np.asarray(origin[:2])
                maximum_pose_error = max(maximum_pose_error,float(np.linalg.norm(displayed-positions[index,:2])))
            minimum_robot_distance = min(minimum_robot_distance,float(np.linalg.norm(positions[0,:2]-positions[1,:2])))
            max_robot3_x = max(max_robot3_x,float(positions[2,0]))
        runtime.commands[name] = (0,0,time.monotonic())
        for _ in range(30):
            await omni.kit.app.get_app().next_update_async()

    try:
        assert body_probe["hit"] and "Nova_Carter_ROS_1/chassis_link" in body_probe.get("collision",""), "Chassis collider is missing"
        await drive("carter3",12)
        positions = runtime.articulations.get_world_poses()[0].numpy()
        assert contacts["wall"] > 0, "No actual hospital-wall contact was reported"
        assert max_robot3_x < wall_limit-.15, ("Chassis crossed the wall",max_robot3_x)
        encoder_x = runtime.initial[2][0]+runtime.encoder_odom[2,0]
        report["blocked_wheel_error_m"] = abs(encoder_x-float(positions[2,0]))
        await drive("carter1",12)
        assert contacts["robots"] > 0, "No robot-to-robot contact was reported"
        assert minimum_robot_distance > .6, ("Robot bodies interpenetrated",minimum_robot_distance)
        assert maximum_pose_error < .04, ("Displayed odometry diverged during contact",maximum_pose_error)
        assert min(worst_penetration.values()) > -.04, worst_penetration
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        for name in runtime.commands:
            runtime.commands[name] = (0,0,time.monotonic())
        subscription = None
        report.update(contacts=dict(contacts),worst_contact_separation_m=worst_penetration,
            maximum_pose_error_m=maximum_pose_error,minimum_robot_distance_m=minimum_robot_distance,
            max_robot3_x=max_robot3_x,wall_x=wall_limit,observed_pairs=sorted(observed_pairs))
        (runtime.directory/"contact-results.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report),flush=True)
