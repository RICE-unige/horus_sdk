"""Isaac Sim --exec entry point; imports happen after Kit startup."""

import asyncio
import os
from pathlib import Path
import sys

import omni.kit.app

root = Path(os.environ["HORUS_HOSPITAL_SOURCE"])
sys.path.insert(0, str(root))
from kit_runtime import HospitalRuntime


async def start_hospital():
    global hospital_runtime
    try:
        hospital_runtime = HospitalRuntime(os.environ["HORUS_HOSPITAL_RUN"],
                                           int(os.environ.get("ROS_DOMAIN_ID", "87")))
        await hospital_runtime.start(os.environ["HORUS_HOSPITAL_SCENE"])
        if os.environ.get("HORUS_HOSPITAL_CONTACT_TEST") == "1":
            from contact_checks import run
            await run(hospital_runtime)
            hospital_runtime.stop()
            omni.kit.app.get_app().post_quit(0)
    except Exception:
        import traceback
        failure = traceback.format_exc()
        print(failure)
        (Path(os.environ["HORUS_HOSPITAL_RUN"]) / "simulation-failed.txt").write_text(failure)
        omni.kit.app.get_app().post_quit(1)


# Retain the startup task for the application lifetime.
hospital_start_task = asyncio.ensure_future(start_hospital())
