import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 
from scipy.spatial.transform import Rotation as R

#xml_path = "puppy.xml"
xml_path = "scene.xml"

simend = 10

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

#def quat2euler(quat_mujoco):
    #mujocoy quat is constant,x,y,z,
    #scipy quaut is x,y,z,constant
#    quat_scipy = np.array([quat_mujoco[3],quat_mujoco[0],quat_mujoco[1],quat_mujoco[2]])
#    r = R.from_quat(quat_scipy)
#    euler = r.as_euler('xyz', degrees=True)
#    return euler

def init_controller(model, datra):
    pass 

def controller(model, data):
    #data.ctrl[0] = 10
    #data.ctrl[1] = 10 
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def check_model_data():
    print("=== mj_data Overview ===")
    print(f"Number of qpos     : {model.nq}")
    print(f"Number of qvel     : {model.nv}")
    print(f"Number of actuators: {model.nu}")
    print(f"Number of sensors  : {model.nsensor}")
    print()

    # Generalized position and velocity
    print("--- Generalized Coordinates ---")
    print(f"qpos shape: {data.qpos.shape} -> {data.qpos}")
    print(f"qvel shape: {data.qvel.shape} -> {data.qvel}")
    print()

    # Actuator control input
    print("--- Actuator Inputs (data.ctrl) ---")
    print(f"ctrl shape: {data.ctrl.shape} -> {data.ctrl}")
    print()

    # Sensor data
    print("--- Sensor Data (data.sensordata) ---")
    print(f"sensordata shape: {data.sensordata.shape}")
    print(f"sensordata: {data.sensordata}")
    print()

    # Force applied to bodies (external forces, e.g. ElasticBand)
    print("--- External Forces (data.xfrc_applied) ---")
    print(f"xfrc_applied shape: {data.xfrc_applied.shape}")
    print()

    # Joint forces (for diagnostics)
    print("--- Applied Joint Forces (data.qfrc_applied) ---")
    print(f"qfrc_applied shape: {data.qfrc_applied.shape}")
    print()

    # Center of mass velocity
    print("--- Subtree COM Linear Velocity (data.subtree_linvel) ---")
    print(f"subtree_linvel shape: {data.subtree_linvel.shape}")
    print()

    # Optional: contact forces if using collision
    print("--- Contacts (data.ncon) ---")
    print(f"Number of active contacts: {data.ncon}")



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

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath
print(xml_path)

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

glfw.init()
window = glfw.create_window(1200, 900, "demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

cam.azimuth = 90 ; cam.elevation = -45 ; cam.distance =  13
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])


#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)
check_model_data()

while not glfw.window_should_close(window):
    time_prev = data.time
    #for i in range(model.nu):
    #        data.ctrl[i] += 0.1
    #        if data.ctrl[i] > 1.57:
    #            data.ctrl[i] = -1.57
    #print(data.ctrl[0])
    target_angle = 0.5 * np.sin(2 * np.pi * 0.5 * data.time)

    for i in range(model.nu):
        #data.ctrl[i] = target_angle
        pass

    while (data.time - time_prev < 1.0/60.0):
    
        mj.mj_step(model, data)

    #quat = np.array([data.qpos[3],data.qpos[4],data.qpos[5],data.qpos[6]])
    #euler = quat2euler(quat)

    #if (data.time>=simend):
    #    break

    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)

    glfw.poll_events()

glfw.terminate()
