import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from mujoco_model_util import check_model_data, Slider

s1 = Slider(10, 50, 0.55)
s2 = Slider(10, 90, 0.675)
s3 = Slider(10, 130, 0.55)
s4 = Slider(10, 170, 0.675)
s5 = Slider(10, 210, 0.55)
s6 = Slider(10, 250, 0.625)
s7 = Slider(10, 290, 0.55)
s8 = Slider(10, 330, 0.625)

sliders = [s1, s2, s3, s4, s5, s6, s7, s8]

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

check_model_data(model, data)

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "PuppyPi Slider Trim", None, None)
glfw.make_context_current(window)
cam = mj.MjvCamera()
opt = mj.MjvOption()
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Desired standing joint positions
# desired_positions = np.zeros(model.nu)  # ← try setting to 0.0 first
# For some robots, you need slight joint bends like
# SET GOOD STANDING POSE

joint_range = 2.0
desired_positions = np.array([
    0.2,  0.7,     # back Left      lb_joint1   lb_joint2 
    0.2,  0.7,     # back Right     rb_joint1   rb_joint2
    0.2,  0.5,     # Front Left       rf_joint1   rf_joint2
    0.2,  0.5,     # Front Right      lf_joint1   lf_joint2
])


slider_values = np.array([
    0.55, 0.675,
    0.55, 0.675,
    0.55, 0.625,
    0.55, 0.625
])

# You may have 12 actuators, so shape must match model.nu
# Check: len(desired_positions) == model.nu


# PD gains
kp = 80.0  # Position gain
kd = 2.0   # Damping gain

# Overlay and Slider Globals
_overlay = {}
slider_value = 0.5

# --- Helper functions ---
def add_overlay(gridpos, text1, text2):
    if gridpos not in _overlay:
        _overlay[gridpos] = ["", ""]
    _overlay[gridpos][0] += text1 + "\n"
    _overlay[gridpos][1] += text2 + "\n"

def create_overlay(model, data):
    topleft = mj.mjtGridPos.mjGRID_TOPLEFT
    for idx in range(8):
        #add_overlay(topleft,f"slider {12 - idx}",'%.2f' % sliders[11 - idx].slider_value  # dispaly slider value
        add_overlay(topleft,f"slider {idx}",'%.2f' % desired_positions[idx]  # dispaly slider value
    )
    
def keyboard(window, key, scancode, act, mods):
    global desired_positions, knee, hip
    if (act == glfw.PRESS and key == glfw.KEY_R):
        for i in range(8):
            sliders[i].slider_value = slider_values[i]
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    
def mouse_button_callback(window, button, action, mods):
    global dragging_slider
    if button == glfw.MOUSE_BUTTON_LEFT:
        xpos, ypos = glfw.get_cursor_pos(window)
        width, height = glfw.get_window_size(window)
        ypos = height - ypos
        if action == glfw.PRESS:
            for i in range(8):
                sliders[i].set_dragging_true(xpos, ypos)
            
        elif action == glfw.RELEASE:
            for i in range(8):
                sliders[i].set_dragging_false()   

def mouse_move_callback(window, xpos, ypos):
    global slider_value
    for i in range(8):
        if sliders[i].dragging_slider:
            width, height = glfw.get_window_size(window)
            ypos = height - ypos
            sliders[i].set_slide_value(xpos)

# Register Callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_mouse_button_callback(window, mouse_button_callback)
glfw.set_cursor_pos_callback(window, mouse_move_callback)

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    for i in range(model.nu):
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]

        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]

        #if i == 0:  # Only control actuator 0
        pos_error = (desired_positions[i] - qpos + np.pi) % (2 * np.pi) - np.pi
        torque = kp * pos_error - kd * qvel
        data.ctrl[i] = torque

#mj.set_mjcb_control(controller)

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time

    # Update desired position from slider
    for i in range(model.nu):
        desired_positions[i] = (sliders[i].slider_value - 0.5) * 2 * joint_range
        #desired_positions[i] = (sliders[i].slider_value - 0.5) * 2 * amplitude[i]
        #desired_positions[i] = (sliders[i].slider_value - 0.5) * 1.5  # -1.0 to +1.0 radians
    # Assuming joint order: abduction, knee, abduction, knee, ...
    #for i in range(model.nu):
    #    if i % 2 == 0:  # abduction joint (e.g., hip sideways)
    #        desired_positions[i] = (sliders[i].slider_value - 0.5) * 1.5  # e.g., [-0.75, 0.75]
    #    else:  # knee joint
    #        desired_positions[i] = -1.0 - (sliders[i].slider_value) * 1.0  # from -1 to -2
    print(desired_positions)

   
    # --- PD CONTROL ---
    for i in range(model.nu):
        #joint_id = model.actuator_trnid[i][0]
        #qpos_adr = model.jnt_qposadr[joint_id]
        #qvel_adr = model.jnt_dofadr[joint_id]

        #qpos = data.qpos[qpos_adr]
        #qvel = data.qvel[qvel_adr]

        #if i == 0:  # Only control actuator 0
        '''
        Below code works for unitree
        '''
        #pos_error = (desired_positions[i] - qpos + np.pi) % (2 * np.pi) - np.pi
        #torque = kp * pos_error - kd * qvel
        #data.ctrl[i] = torque
        #else:
        #    data.ctrl[i] = 0
        qpos = data.sensordata[i*3]        # Sensor data: joint angle
        qvel = data.sensordata[i*3+1]  # Sensor data: joint velocity
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque
        #print(desired_positions)


    # --- Simulate Physics ---
    while (data.time - time_prev) < (1.0 / 60.0):
        mj.mj_step(model, data)

    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    create_overlay(model, data)

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

    _overlay.clear()

    for i in range(8):
        sliders[i].draw_slide()
   
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
