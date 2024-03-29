import maya.cmds as cmds

# Specify the test scene path
test_scene_path = ".../MayaPhysicsMoveTool/test/simple_test_scene.mb"

# Open the test scene
cmds.file(test_scene_path, open=True, force=True)

# Specify the plugin path
plugin_path = ".../MayaPhysicsMoveTool/build/lib/Release/MayaViewportCollisionsPlugin.mll"

# Unload the plugin if it's loaded
if cmds.pluginInfo(plugin_path, query=True, loaded=True):
    cmds.unloadPlugin(plugin_path)

# Load the plugin
cmds.loadPlugin(plugin_path)

# Use your custom move manipulator context
cmds.physicsManipContext('physicsManipContext1')
cmds.refresh()
cmds.select("pCube5")
cmds.setToolTo('physicsManipContext1')

