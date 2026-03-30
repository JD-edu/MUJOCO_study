import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "jdCobot100", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Camera initial pose
cam.lookat[:] = [0, 0, 0]
cam.distance = 1.0
cam.elevation = -20
cam.azimuth = 90

# Mouse state
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def keyboard(window, key, scancode, act, mods):
    global knee, hip
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


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


# Register mouse callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# PD Control setup
reset_positions = np.array([0, 1.5708, 1.5708, 1.5708, 1.5708, 1.5708])
target_positions = np.array([1.5708, 1, 1, 1, 1, 1])
kp = 100.0
kd = 5.0

for i in range(model.nu):
    joint_id = model.actuator_trnid[i][0]
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = reset_positions[i]

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    for i in range(model.nu):
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]
        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]
        pos_error = target_positions[i] - qpos
        vel_error = -qvel
        torque = kp * pos_error + kd * vel_error
        data.ctrl[i] = torque

    while (data.time - time_prev) < (1.0 / 600.0):
        mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
