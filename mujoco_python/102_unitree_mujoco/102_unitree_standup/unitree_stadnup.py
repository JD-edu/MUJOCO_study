import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "Unitree Stand", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Desired standing joint positions
# desired_positions = np.zeros(model.nu)  # ← try setting to 0.0 first
# For some robots, you need slight joint bends like
# SET GOOD STANDING POSE
'''
desired_positions = np.array([
    0.0,  0.1,  -1.4,   # Front Left  (Hip Roll, Hip Pitch, Knee)
    0.0,  0.1,  -1.4,   # Front Right
    0.0,  0.1,  -1.4,   # Rear Left
    0.0,  0.1,  -1.4    # Rear Right
])
'''
desired_positions = np.array([
    0.0,  0.9,  -1.8,  # Front Left
    0.0,  0.9,  -1.8,  # Front Right
    0.0,  0.9,  -1.8,  # Rear Left
    0.0,  0.9,  -1.8   # Rear Right
])

# You may have 12 actuators, so shape must match model.nu
# Check: len(desired_positions) == model.nu


# PD gains
kp = 80.0  # Position gain
kd = 2.0   # Damping gain

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    # --- PD CONTROL: compute torques ---
    for i in range(model.nu):
        qpos = data.sensordata[i]        # Sensor data: joint angle
        qvel = data.sensordata[i + model.nu]  # Sensor data: joint velocity
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque

    # --- Simulate ---
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
