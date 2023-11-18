import pybullet as p

# Start PyBullet in GUI mode
p.connect(p.GUI)

# Load the .bullet file
# Replace 'your_file.bullet' with the path to your .bullet file
bullet_file = 'bulletWorld.bullet'
p.restoreState(fileName=bullet_file)

# Keep the GUI window open until manually closed
while p.isConnected():
    p.getCameraImage(320, 200)  # You can adjust the resolution as needed

# Disconnect PyBullet after the window is closed
p.disconnect()
