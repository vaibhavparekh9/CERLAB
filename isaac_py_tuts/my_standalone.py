from isaacsim import SimulationApp
"""
NOTE: Any Omniverse level imports must occur after the class is instantiated.
Because APIs are provided by the extension/runtime plugin system, it must be loaded before they will be available to import.
"""

# Start the application
simulation_app = SimulationApp({"headless": False})

# Get the utility to enable extensions
from isaacsim.core.utils.extensions import enable_extension

# Enable the layers and stage windows in the UI
enable_extension("omni.kit.widget.stage")
enable_extension("omni.kit.widget.layers")

# Keep updating frame until app is running
while simulation_app.is_running():
    simulation_app.update()

# Close when update is stopped (manually closed)
simulation_app.close()


# this code can be found at: https://docs.isaacsim.omniverse.nvidia.com/4.5.0/python_scripting/manual_standalone_python.html

