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

# Viewer setup
glfw.init()
window = glfw.create_window(1200, 900, "jdCobot100", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

cam.lookat[:] = [0, 0, 0]
cam.distance = 1.0
cam.elevation = -20
cam.azimuth = 90

# Mouse state
button_left = button_middle = button_right = False
lastx = lasty = 0

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    button_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
    button_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos
    if not (button_left or button_middle or button_right):
        return
    width, height = glfw.get_window_size(window)
    shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05*yoffset, scene, cam)

# Callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# PD Control gains
kp, kd = 100.0, 5.0

# Define pick-and-place joint sequences
waypoints = [
    np.array([0, 1.5708, 1.5708, 1.5708, 1.5708, 0]),  # Home pose
    np.array([1.5708, 1.0, 1.0, 1.0, 1.0, 1.0]),  # Move to pick
    #np.array([0, 1.5708, 1.5708, 1.5708, 1.5708, 0]),  # Grasp (simulate with wrist pose)
    #np.array([0, 0.8, 1.0, 1.2, 1.2, 1.2]),  # Lift
    #np.array([0, 1.2, 1.3, 1.6, 1.2, 1.2]),  # Move to place
    #np.array([0, 1.2, 1.3, 1.6, 1.0, 1.0]),  # Release
    #np.array([0, 0.8, 1.0, 1.2, 1.0, 1.0]),  # Return to home
]
waypoint_idx = 0
target_positions = waypoints[waypoint_idx]
num_actuators = model.nu


# Initialize positions
for i in range(num_actuators):
    joint_id = model.actuator_trnid[i][0]
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = waypoints[1][i]

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    # PD control loop
    errors = []

    for i in range(num_actuators):
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]
        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]
        pos_error = target_positions[i] - qpos
        errors.append(pos_error)
        vel_error = -qvel
        torque = kp * pos_error + kd * vel_error
        data.ctrl[i] = torque
    print(errors)

    if abs(errors[0]) < 0.00001 and abs(errors[1]) < 0.00001 and abs(errors[2]) < 0.00001 and abs(errors[3]) < 0.00001:
        target_position = waypoints[waypoint_idx]
        waypoint_idx += 1
        if waypoint_idx > 1:
            waypoint_idx = 0
        

    
    
    #angles = []
    #for i in range(num_actuators):
    #    joint_id = model.actuator_trnid[i][0]
    #    qpos_adr = model.jnt_qposadr[joint_id]
    #    angles.append(data.qpos[qpos_adr])
    #print(angles)

    while (data.time - time_prev) < (1.0 / 500.0):
        mj.mj_step(model, data)

    
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
