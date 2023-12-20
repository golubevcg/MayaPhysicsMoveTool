import maya.cmds as cmds

# Specify the test scene path
#test_scene_path = "C:/Users/golub/Documents/maya_viewport_collision_plugin/test/collisions_test_scene.mb"
#test_scene_path = "C:/Users/golub/Documents/maya_viewport_collision_plugin/test/collisions_test_scene_no_plane.mb"
test_scene_path = "C:/Users/golub/Documents/maya_viewport_collision_plugin/test/collisions_test_scene_no_plane_sphere.mb"

# Open the test scene
cmds.file(test_scene_path, open=True, force=True)

# Specify the plugin path
plugin_path = "C:/Users/golub/Documents/maya_viewport_collision_plugin/build/lib/Release/MayaViewportCollisionsPlugin.mll"

# Unload the plugin if it's loaded
if cmds.pluginInfo(plugin_path, query=True, loaded=True):
    cmds.unloadPlugin(plugin_path)

# Load the plugin
cmds.loadPlugin(plugin_path)

# Use your custom move manipulator context
cmds.customMoveManipContext('customMoveManipContext1')
cmds.refresh()
cmds.select("pCube5")
cmds.setToolTo('customMoveManipContext1')
