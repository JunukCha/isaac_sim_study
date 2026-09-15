import json
import sys

import carb
import omni.usd

from pxr import UsdGeom
from omni.kit.viewport.utility import get_active_viewport


def save_current_camera(output_path):
    viewport = get_active_viewport()

    if viewport is None:
        carb.log_error("[Camera] No active viewport found.")
        return

    stage = omni.usd.get_context().get_stage()

    camera_path = viewport.camera_path
    camera_prim = stage.GetPrimAtPath(camera_path)

    if not camera_prim.IsValid():
        carb.log_error(f"[Camera] Invalid camera prim: {camera_path}")
        return

    # Read current camera transform
    xform = UsdGeom.Xformable(camera_prim)
    world_transform = xform.ComputeLocalToWorldTransform(0.0)

    position = world_transform.ExtractTranslation()
    orientation = world_transform.ExtractRotationQuat()

    camera_data = {
        "camera_path": str(camera_path),
        "position": [
            float(position[0]),
            float(position[1]),
            float(position[2]),
        ],
        "orientation": [
            float(orientation.GetReal()),
            float(orientation.GetImaginary()[0]),
            float(orientation.GetImaginary()[1]),
            float(orientation.GetImaginary()[2]),
        ],
    }

    with open(output_path, "w") as f:
        json.dump(camera_data, f, indent=4)

    carb.log_warn(f"[Camera] Saved camera pose to: {output_path}")
    carb.log_warn(f"[Camera] Position: {camera_data['position']}")
    carb.log_warn(f"[Camera] Orientation: {camera_data['orientation']}")


# Default path
OUTPUT_PATH = "/workspace/isaacsim/isaac_sim_study/python_scripts/projects/3_3d_manipulation/3_camera_pose.json"

save_current_camera(OUTPUT_PATH)