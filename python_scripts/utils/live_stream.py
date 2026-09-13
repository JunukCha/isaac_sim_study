from isaacsim import SimulationApp

CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "hide_ui": False,
    "renderer": "RealTimePathTracing",
    "display_options": 3286,
}

simulation_app = SimulationApp(CONFIG)

import omni.kit.app
from isaacsim.core.experimental.utils.app import enable_extension

# Show mouse cursor in livestream
simulation_app.set_setting("/app/window/drawMouse", True)

# Enable Livestream
enable_extension("omni.kit.livestream.app")