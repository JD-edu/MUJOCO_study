import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from mujoco_model_util import check_model_data
import math 
import matplotlib.pyplot as plt


def keyboard(window, key, scancode, act, mods):
    global knee, hip
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    elif (act == glfw.PRESS and key == glfw.KEY_W):
        knee += 0.1
        if knee > 1.5:
            knee= 1.7
    elif (act == glfw.PRESS and key == glfw.KEY_S):
        knee -= 0.1
        if knee < -0.2:
            knee= 0

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)
    
def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)    

log_time = []
log_rf_hip = []
log_lf_hip = []
log_rb_hip = []
log_lb_hip = []

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

check_model_data(model, data)

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


desired_positions = np.array([
    0.0,  0.0,    # back Right  hip, knee
    0.0,  0.0,    # back Left  hip, knee 
    0.0,  0.0,    # Front Right  hip, knee
    0.0,  0.0     # Front Left  hip, knee 
])

# 시행착오법으로  최적의 값을 찾았음, 역시 desired_position의 각 값이 각 legs에 어떤 것에 
# 해당하는 것인지도 시행착오법으로 찾았음 
base_pose = np.array([
    0.5,  0.2,     # back Right  hip, knee
    0.5,  0.2,     # back Left  hip, knee 
    0.5,  0.1,     # Front Right  hip, knee
    0.5,  0.1,     # Front Left  hip, knee 
])


# Gait amplitude (adjust per joint)
gait_amp = np.array([
    0.3,  0.1,   # back Right  hip, knee
    0.3,  0.1,   # back Left  hip, knee 
    0.2,  0.1,   # Front Right  hip, knee
    0.2,  0.1    # lFront Left  hip, knee 
])


# Gait phase (0: swing, π: stance)
# Diagonal trot: (LF+RB) vs (RF+LB)
gait_phase = np.array([
    math.pi,  math.pi,      # lb (stance)
    0.0,      0.0,          # rb (swing)
    math.pi,  math.pi,      # rf (stance)
    0.0,      0.0           # lf (swing)
])

# You may have 12 actuators, so shape must match model.nu
# Check: len(desired_positions) == model.nu
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)


# PD gains
kp = 40.0  # Position gain
kd = 2.0   # Damping gain

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

gait_freq = 5.0

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    # --- PD CONTROL: compute torques ---
    for i in range(model.nu):   
        phase = 2 * math.pi * gait_freq * data.time + gait_phase[i]
        offset = gait_amp[i] * math.sin(phase)
        desired_positions[i] = base_pose[i] + offset
        qpos = data.sensordata[i*3]        # Sensor data: joint angle
        qvel = data.sensordata[i*3+1]  # Sensor data: joint velocity
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque

        rf_hip = data.qpos[7 + 4]
        lf_hip = data.qpos[7 + 6]
        rb_hip = data.qpos[7 + 2]
        lb_hip = data.qpos[7 + 0]

        log_time.append(data.time)
        log_rf_hip.append(rf_hip)
        log_lf_hip.append(lf_hip)
        log_rb_hip.append(rb_hip)
        log_lb_hip.append(lb_hip)

    # back two legs 
    #print(f"time={data.time:.2f}  lb_hip={data.qpos[7+0]:.2f}  rb_hip={data.qpos[7+2]:.2f}")
    print(f"time={data.time:.2f}  lf_hip={data.qpos[7 + 6]:.2f}  rf_hip={data.qpos[7 + 4]:.2f}   lb_hip={data.qpos[7+0]:.2f}  rb_hip={data.qpos[7+2]:.2f}")
   
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

# Plot joint angles over time
plt.figure(figsize=(10, 6))
plt.title("Front Leg Joint Positions Over Time")

plt.plot(log_time, log_rf_hip, label="RF Hip", linestyle='-', color='blue')
plt.plot(log_time, log_lf_hip, label="LF Hip", linestyle='--', color='blue')
plt.plot(log_time, log_rb_hip, label="RB Hip", linestyle='-', color='green')
plt.plot(log_time, log_lb_hip, label="LB Hip", linestyle='--', color='green')

plt.xlabel("Time [s]")
plt.ylabel("Joint Position [rad]")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


