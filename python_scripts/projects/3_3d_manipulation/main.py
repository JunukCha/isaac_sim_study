import sys
from pathlib import Path

# Add python_scripts directory to Python module search path
PYTHON_SCRIPTS_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_SCRIPTS_DIR))

from utils.live_stream import simulation_app

import time

# Isaac Sim modules must be imported after SimulationApp
from isaacsim.storage.native import get_assets_root_path
import omni.usd
import carb

from pxr import UsdGeom, UsdLux, Gf

# ---------------------------------------------------------
# Get stage
# ---------------------------------------------------------
stage = omni.usd.get_context().get_stage()


# ---------------------------------------------------------
# Helper functions
# ---------------------------------------------------------
def create_cube(path, position, scale, color):
    cube = UsdGeom.Cube.Define(stage, path)

    cube.CreateSizeAttr(1.0)

    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(*scale))

    cube.CreateDisplayColorAttr(
        [Gf.Vec3f(*color)]
    )

    return cube


def create_cylinder(path, position, radius, height, color):
    cylinder = UsdGeom.Cylinder.Define(stage, path)

    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")

    xform = UsdGeom.Xformable(cylinder)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    cylinder.CreateDisplayColorAttr(
        [Gf.Vec3f(*color)]
    )

    return cylinder


# ---------------------------------------------------------
# Scene root
# ---------------------------------------------------------
UsdGeom.Xform.Define(stage, "/World")
UsdGeom.Xform.Define(stage, "/World/Scene")


# ---------------------------------------------------------
# Ground
# ---------------------------------------------------------
create_cube(
    "/World/Scene/Ground",
    position=(0.0, 0.0, -0.05),
    scale=(6.0, 6.0, 0.1),
    color=(0.3, 0.3, 0.3),
)


# ---------------------------------------------------------
# Desk
# ---------------------------------------------------------
desk_height = 0.75
desk_thickness = 0.08

create_cube(
    "/World/Scene/Desk",
    position=(0.6, 0.0, desk_height),
    scale=(1.4, 1.0, desk_thickness),
    color=(0.45, 0.25, 0.12),
)


# ---------------------------------------------------------
# Desk legs
# ---------------------------------------------------------
leg_positions = [
    (0.05, -0.4, desk_height / 2),
    (0.05,  0.4, desk_height / 2),
    (1.15, -0.4, desk_height / 2),
    (1.15,  0.4, desk_height / 2),
]

for i, position in enumerate(leg_positions):
    create_cube(
        f"/World/Scene/DeskLeg_{i}",
        position=position,
        scale=(0.08, 0.08, desk_height),
        color=(0.35, 0.18, 0.08),
    )


# ---------------------------------------------------------
# Tabletop height
# ---------------------------------------------------------
table_top_z = desk_height + desk_thickness / 2


# ---------------------------------------------------------
# Cup
# ---------------------------------------------------------
cup_height = 0.18

create_cylinder(
    "/World/Scene/Cup",
    position=(
        0.45,
        0.20,
        table_top_z + cup_height / 2,
    ),
    radius=0.06,
    height=cup_height,
    color=(0.15, 0.35, 0.9),
)


# ---------------------------------------------------------
# Obstacle box
# ---------------------------------------------------------
box_size = (0.16, 0.16, 0.18)

create_cube(
    "/World/Scene/ObstacleBox",
    position=(
        0.58,
        -0.15,
        table_top_z + box_size[2] / 2,
    ),
    scale=box_size,
    color=(0.9, 0.55, 0.1),
)


# ---------------------------------------------------------
# Manipulation target
# ---------------------------------------------------------
target_size = (0.10, 0.10, 0.10)

create_cube(
    "/World/Scene/Target",
    position=(
        0.80,
        0.05,
        table_top_z + target_size[2] / 2,
    ),
    scale=target_size,
    color=(0.1, 0.85, 0.2),
)

# ---------------------------------------------------------
# Add Franka
# ---------------------------------------------------------
assets_root = get_assets_root_path()

franka_usd = (
    assets_root
    + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
)

franka_prim = stage.DefinePrim("/World/Franka", "Xform")
franka_prim.GetReferences().AddReference(franka_usd)

# Place Franka in front of the table
franka_xform = UsdGeom.Xformable(franka_prim)
trans_attr = franka_prim.GetAttribute("xformOp:translate")
trans_attr.Set(Gf.Vec3d(-0.35, 0.0, 0.0))

# franka_xform.AddTranslateOp().Set(
#     Gf.Vec3d(-0.35, 0.0, 0.0)
# )

carb.log_warn("Franka added.")

# ---------------------------------------------------------
# Lighting
# ---------------------------------------------------------
light = UsdLux.DistantLight.Define(
    stage,
    "/World/Scene/Light",
)

light.CreateIntensityAttr(3000.0)

light_xform = UsdGeom.Xformable(light)

light_xform.AddRotateXYZOp().Set(
    Gf.Vec3f(-45.0, 30.0, 0.0)
)


carb.log_warn(
    "3D Scene-Guided Manipulation scene created."
)


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------
start_time = time.time()
duration = 60.0  # seconds

while simulation_app.is_running():
    simulation_app.update()

    if time.time() - start_time >= duration:
        break


# ---------------------------------------------------------
# Shutdown
# ---------------------------------------------------------
simulation_app.close()