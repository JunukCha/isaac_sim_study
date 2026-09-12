from isaacsim.core.experimental.prims import Articulation
import carb

robot = Articulation("/World/Robot")

# Franka:
# 0~6: arm joints
# 7~8: gripper fingers
target = [[
    0.0,    # joint1
    -0.7,   # joint2
    0.0,    # joint3
    -2.0,   # joint4
    0.0,    # joint5
    1.5,    # joint6
    0.7,    # joint7
    0.04,   # finger1
    0.04,   # finger2
]]

robot.set_dof_position_targets(target)

carb.log_info("Franka joint target applied")