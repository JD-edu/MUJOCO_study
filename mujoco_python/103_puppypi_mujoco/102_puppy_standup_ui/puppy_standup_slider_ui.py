import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

class Slider:
    def __init__(self, x, y, value):
        self.slider_start_x = x
        self.slider_start_y = y
        self.dragging_slider = False
        self.slider_value = value
        self.slider_width = 200
        self.slider_height = 20

    def set_dragging_true(self, click_x, click_y):
        if (self.slider_start_x <= click_x <= self.slider_start_x + self.slider_width) and (self.slider_start_y <= click_y <= self.slider_start_y + self.slider_height):
                self.dragging_slider = True
    
    def set_dragging_false(self):
        self.dragging_slider = False
    
    def set_slide_value(self, click_x):
        new_value = (click_x - self.slider_start_x) / self.slider_width
        self.slider_value = np.clip(new_value, 0.0, 1.0)

    def draw_slide(self):
        mj.mjr_rectangle(
            mj.MjrRect(self.slider_start_x, self.slider_start_y, self.slider_width, self.slider_height),
            0.4, 0.4, 0.4, 1.0
        )
        knob_x = int(self.slider_start_x + self.slider_value * self.slider_width) - 5
        mj.mjr_rectangle(
            mj.MjrRect(knob_x, self.slider_start_y - 5, 10, self.slider_height + 10),
            0.8, 0.2, 0.2, 1.0
        )

    
s1 = Slider(10, 50, 0.5)
s2 = Slider(10, 90, 0.73)
s3 = Slider(10, 130, 0.05)
s4 = Slider(10, 170, 0.5)
s5 = Slider(10, 210, 0.73)
s6 = Slider(10, 250, 0.05)
s7 = Slider(10, 290, 0.5)
s8 = Slider(10, 330, 0.73)
s9 = Slider(10, 370, 0.05)
s10 = Slider(10, 410, 0.5)
s11 = Slider(10, 450, 0.73)
s12 = Slider(10, 490, 0.05)

sliders = [s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12]

# Initialize MuJoCo Model
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Viewer setup
glfw.init()
window = glfw.create_window(1000, 800, "Unitree Stand", None, None)
glfw.make_context_current(window)

cam = mj.MjvCamera()
opt = mj.MjvOption()
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_50)

# Globals
desired_positions = np.array([
    0.0, 0.9, -1.8, # FL
    0.0, 0.9, -1.8, # FR
    0.0, 0.9, -1.8, # RL
    0.0, 0.9, -1.8  # RR
])


desired_slider = np.array([
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05
])

desired_slider_reset = np.array([
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05,
    0.5, 0.73, 0.05
])
kp = 80.0
kd = 2.0

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
    for idx in range(12):
        #add_overlay(topleft,f"slider {12 - idx}",'%.2f' % sliders[11 - idx].slider_value  # dispaly slider value
        add_overlay(topleft,f"slider {12 - idx}",'%.2f' % desired_positions[idx]  # dispaly slider value
    )
    
def keyboard(window, key, scancode, act, mods):
    global desired_positions
    if (act == glfw.PRESS and key == glfw.KEY_R):
        for i in range(12):
            sliders[i].slider_value = desired_slider_reset[i]
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    if act == glfw.PRESS and key == glfw.KEY_Q:
        for i in range(model.nu):
            desired_positions[i] += 0.05
    if act == glfw.PRESS and key == glfw.KEY_A:
        for i in range(model.nu):
            desired_positions[0] -= 0.05

def mouse_button_callback(window, button, action, mods):
    global dragging_slider
    if button == glfw.MOUSE_BUTTON_LEFT:
        xpos, ypos = glfw.get_cursor_pos(window)
        width, height = glfw.get_window_size(window)
        ypos = height - ypos
        if action == glfw.PRESS:
            for i in range(12):
                sliders[i].set_dragging_true(xpos, ypos)
            
        elif action == glfw.RELEASE:
            for i in range(12):
                sliders[i].set_dragging_false()   

def mouse_move_callback(window, xpos, ypos):
    global slider_value
    for i in range(12):
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
   
mj.set_mjcb_control(controller)

# --- Main Simulation Loop ---
while not glfw.window_should_close(window):
    time_prev = data.time

    # Update desired position from slider
    for i in range(model.nu):
        desired_positions[i] = (sliders[i].slider_value - 0.5) * 4.0  # -1.0 to +1.0 radians
    print(desired_positions)
    # --- PD CONTROL ---
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
        #else:
        #    data.ctrl[i] = 0

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

    for i in range(12):
        sliders[i].draw_slide()
   
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
