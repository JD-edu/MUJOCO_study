import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 
from scipy.spatial.transform import Rotation as R
import time 

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
    
desired_positions = np.array([
    0,  -0.8,   # Front Left
    0,  -0.8,   # Front Right
    0,  -0.8,   # Rear Left
    0,  -0.8,   # Rear Right
])


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

kp = 100.0  # Position gain
kd = 2.0 # 2.0   # Damping gain

key_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "stand")
mj.mj_resetDataKeyframe(model, data, key_id)

print("Initial joint angles (qpos):", data.qpos[7:])
qq = -2.7
while not glfw.window_should_close(window):
    time_prev = data.time
    '''
    <sensor>
        <actuatorpos name="lb_joint1_p" actuator="lb_joint1" />
        <actuatorvel name="lb_joint1_v" actuator="lb_joint1" />
        <actuatorfrc name="lb_joint1_f" actuator="lb_joint1" noise="0.001" />
        <actuatorpos name="lb_joint2_p" actuator="lb_joint2" />
        <actuatorvel name="lb_joint2_v" actuator="lb_joint2" />
        <actuatorfrc name="lb_joint2_f" actuator="lb_joint2" noise="0.001" />
        <actuatorpos name="rb_joint1_p" actuator="rb_joint1" />
        <actuatorvel name="rb_joint1_v" actuator="rb_joint1" />
        <actuatorfrc name="rb_joint1_f" actuator="rb_joint1" noise="0.001" />
        <actuatorpos name="rb_joint2_p" actuator="rb_joint2" />
        <actuatorvel name="rb_joint2_v" actuator="rb_joint2" />
        <actuatorfrc name="rb_joint2_f" actuator="rb_joint2" noise="0.001" />
        <actuatorpos name="rf_joint1_p" actuator="rf_joint1" />
        <actuatorvel name="rf_joint1_v" actuator="rf_joint1" />
        <actuatorfrc name="rf_joint1_f" actuator="rf_joint1" noise="0.001" />
        <actuatorpos name="rf_joint2_p" actuator="rf_joint2" />
        <actuatorvel name="rf_joint2_v" actuator="rf_joint2" />
        <actuatorfrc name="rf_joint2_f" actuator="rf_joint2" noise="0.001" />
        <actuatorpos name="lf_joint1_p" actuator="lf_joint1" />
        <actuatorvel name="lf_joint1_v" actuator="lf_joint1" />
        <actuatorfrc name="lf_joint1_f" actuator="lf_joint1" noise="0.001" />
        <actuatorpos name="lf_joint2_p" actuator="lf_joint2" />
        <actuatorvel name="lf_joint2_v" actuator="lf_joint2" />
        <actuatorfrc name="lf_joint2_f" actuator="lf_joint2" noise="0.001" />
        <framequat name="orientation" objtype="site" noise="0.001" objname="imu" />
        <gyro name="angular-velocity" site="imu" noise="0.005" cutoff="34.9" />
    </sensor>

    Above sensor tag, parameter arrignment 
    
    actuatorpos
    actuatorvel
    actuatorfrc... 
    So below code is different from Unitree Go2 
    '''
    print("=======")
    #qq += 0.1
    #if qq > 2:
    #    qq = -2 
    qq = 0.2
    desired_positions = np.array([
        qq,  0.5,   # Front Left  first value:hip second value:knee 
        qq,  0.5,   # Front Right
        qq,  0.5,   # Rear Left
        qq,  0.5,   # Rear Right
    ])
    print(qq)
    for i in range(model.nu):
        #qpos = data.sensordata[i*3]        # Sensor data: joint angle
        #qvel = data.sensordata[i*3+1]  # Sensor data: joint velocity
        #torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        #data.ctrl[i] = torque
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]
        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque
        #print(data.qpos[qpos_adr])
        #print(data.ctrl[i])
    #print("current joint angle (qpos):", data.qpos[7:])
    #print("ctrl :", data.ctrl)
    #for i in range(model.nu):
    #    joint_id = model.actuator_trnid[i][0]
    #    axis = model.jnt_axis[joint_id]
    #    print(f"Joint {i} axis: {axis}")


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
