import omni.usd
from pxr import UsdGeom, UsdLux, Gf

stage = omni.usd.get_context().get_stage()

# Reset /World completely
if stage.GetPrimAtPath("/World").IsValid():
    stage.RemovePrim("/World")

UsdGeom.Xform.Define(stage, "/World")


def create_cube(path, position, scale):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)

    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3d(*scale))

    return cube


# Ground
create_cube(
    "/World/Ground",
    position=(0.0, 0.0, -0.05),
    scale=(10.0, 10.0, 0.1),
)

# Boxes
create_cube(
    "/World/Box1",
    position=(0.0, 0.0, 0.5),
    scale=(1.0, 1.0, 1.0),
)

create_cube(
    "/World/Box2",
    position=(2.0, 0.0, 0.5),
    scale=(0.7, 0.7, 1.0),
)

create_cube(
    "/World/Box3",
    position=(-2.0, 1.0, 0.75),
    scale=(1.0, 1.0, 1.5),
)

# Sphere
sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")
sphere.GetRadiusAttr().Set(0.5)

sphere_xform = UsdGeom.Xformable(sphere)
sphere_xform.AddTranslateOp().Set(
    Gf.Vec3d(0.0, 2.0, 0.5)
)

# Dome light
dome_light = UsdLux.DomeLight.Define(
    stage,
    "/World/DomeLight"
)
dome_light.CreateIntensityAttr(500.0)

# Key light
key_light = UsdLux.RectLight.Define(
    stage,
    "/World/KeyLight"
)

key_light.CreateIntensityAttr(3000.0)
key_light.CreateWidthAttr(5.0)
key_light.CreateHeightAttr(5.0)

light_xform = UsdGeom.Xformable(key_light)

light_xform.AddTranslateOp().Set(
    Gf.Vec3d(2.0, -3.0, 5.0)
)

light_xform.AddRotateXYZOp().Set(
    Gf.Vec3f(35.0, 0.0, 25.0)
)

print("Scene recreated successfully!")