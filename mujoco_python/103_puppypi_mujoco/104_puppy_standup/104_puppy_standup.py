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
cam = mj.MjvCamera()
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# 카메라 위치 설정 (optional)
cam.lookat[:] = [0, 0, 0]
cam.distance = 0.5
cam.elevation = -20
cam.azimuth = 90

# 시행착오법으로  최적의 값을 찾았음, 역시 desired_position의 각 값이 각 legs에 어떤 것에 
# 해당하는 것인지도 시행착오법으로 찾았음 
desired_positions = np.array([
    0.5,  0.2,     # back Right  hip, knee
    0.5,  0.2,     # back Left  hip, knee 
    0.5,  0.1,     # Front Right  hip, knee
    0.5,  0.1,     # Front Left  hip, knee 
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
        qpos = data.sensordata[i*3]        # Sensor data: joint angle
        qvel = data.sensordata[i*3+1]  # Sensor data: joint velocity
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
