from isaacsim.storage.native import get_assets_root_path
import omni.usd
import carb

stage = omni.usd.get_context().get_stage()
root = get_assets_root_path()

robot_path = (
    root
    + "/Isaac/Robots_Multiphysics/FrankaRobotics/"
      "FrankaPanda/franka/franka.usda"
)

prim = stage.DefinePrim("/World/Robot", "Xform")
prim.GetReferences().AddReference(robot_path)

carb.log_info(f"Robot added: {robot_path}")