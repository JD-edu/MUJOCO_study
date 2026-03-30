import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "Quadruped Trot Gait", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# PD gains
kp = 80.0
kd = 2.0

# Trot gait parameters
freq = 1.0            # Hz
amp = 0.4            # Radians (hip pitch swing amplitude)
knee_angle = -1.8     # Fixed bent knee
hip_roll_angle = 0.0  # No roll for now
offset = 0.9

start_time = time.time()

# Main loop
while not glfw.window_should_close(window):
    t = time.time() - start_time
    time_prev = data.time

    # Gait logic
    # Each leg has: [hip_roll, hip_pitch, knee]

    desired_positions = np.array([
        hip_roll_angle,  offset + amp * np.sin(2 * np.pi * freq * t + np.pi), knee_angle,  # Front Left
        hip_roll_angle,  offset + amp * np.sin(2 * np.pi * freq * t + 0), knee_angle,  # Front Right
        hip_roll_angle,  offset + amp * np.sin(2 * np.pi * freq * t + 0), knee_angle,  # Rear Left
        hip_roll_angle,  offset + amp * np.sin(2 * np.pi * freq * t + np.pi), knee_angle   # Rear Right
    ])

    '''desired_positions = np.array([
        hip_roll_angle,  amp * np.sin(2 * np.pi * freq * t + np.pi) - 0.7, knee_angle,  # FL
        hip_roll_angle,  amp * np.sin(2 * np.pi * freq * t + 0) - 0.7,     knee_angle,  # FR
        hip_roll_angle,  amp * np.sin(2 * np.pi * freq * t + 0) - 0.7,     knee_angle,  # RL
        hip_roll_angle,  amp * np.sin(2 * np.pi * freq * t + np.pi) - 0.7, knee_angle   # RR
    ])'''

  

    print("FL pitch:", amp * np.sin(2 * np.pi * freq * t + np.pi) - 0.7)


    # --- PD CONTROL ---
    for i in range(model.nu):
        # NOTE: If no sensors, you can use `data.qpos` and `data.qvel` instead
        #qpos = data.qpos[i + model.jnt_dofadr[0]]  # assumes 1-DOF joints
        #qvel = data.qvel[i]
        #torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        #data.ctrl[i] = torque
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]

        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]

        #if i == 0:  # Only control actuator 0
        pos_error = (desired_positions[i] - qpos + np.pi) % (2 * np.pi) - np.pi
        torque = kp * pos_error - kd * qvel
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
