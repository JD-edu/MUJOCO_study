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

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

_overlay = {}
def add_overlay(gridpos, text1, text2):

    if gridpos not in _overlay:
        _overlay[gridpos] = ["", ""]
    _overlay[gridpos][0] += text1 + "\n"
    _overlay[gridpos][1] += text2 + "\n"

def create_overlay(model,data):
    topleft = mj.mjtGridPos.mjGRID_TOPLEFT
    add_overlay(
        topleft,
        "pos 1 up",'q' ,
    )
    add_overlay(
        topleft,
        "pos 1 down",'a' ,
    )
    add_overlay(
        topleft,
        "pos 1 value", '%.2f' % data.ctrl[0],
    )

#HINT2: add the logics for key press here
def keyboard(window, key, scancode, act, mods):
    global data

    if (act == glfw.PRESS and key == glfw.KEY_R):
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

    if act == glfw.PRESS and key == glfw.KEY_Q:
        desired_positions[0] += 0.05 

    if act == glfw.PRESS and key == glfw.KEY_A:
        desired_positions[0] -= 0.05 

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    pass

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
    0.3,  0.1,  -1.2,  # Front Left
    0.3,  0.1,  -1.2,  # Front Right
    0.3,  0.1,  -1.2,  # Rear Left
    0.3,  0.1,  -1.2   # Rear Right
])

# You may have 12 actuators, so shape must match model.nu
# Check: len(desired_positions) == model.nu


# PD gains
kp = 80.0  # Position gain
kd = 2.0   # Damping gain

glfw.set_key_callback(window, keyboard)

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    # --- PD CONTROL: compute torques ---
    '''for i in range(model.nu):
        joint_id = model.actuator_trnid[i][0]  # Get joint that actuator[i] controls
        qpos_adr = model.jnt_qposadr[joint_id] # Get qpos address of that joint
        qvel_adr = model.jnt_dofadr[joint_id]  # Get qvel address of that joint

        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]

        pos_error = (desired_positions[i] - qpos + np.pi) % (2 * np.pi) - np.pi

        torque = kp * pos_error - kd * qvel
        data.ctrl[i] = torque
    '''
    # mov specifed joint 
    joint_id = model.actuator_trnid[0][0]  # Get joint that actuator[i] controls
    qpos_adr = model.jnt_qposadr[joint_id] # Get qpos address of that joint
    qvel_adr = model.jnt_dofadr[joint_id]  # Get qvel address of that joint

    qpos = data.qpos[qpos_adr]
    qvel = data.qvel[qvel_adr]

    pos_error = (desired_positions[0] - qpos + np.pi) % (2 * np.pi) - np.pi

    torque = kp * pos_error - kd * qvel
    data.ctrl[0] = torque 
    
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)
        
    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #create overlay
    create_overlay(model,data)

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    for gridpos, [t1, t2] in _overlay.items():

        mj.mjr_overlay(
            mj.mjtFontScale.mjFONTSCALE_150,
            gridpos,
            viewport,
            t1,
            t2,
            context)

    # clear overlay
    _overlay.clear()

    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
