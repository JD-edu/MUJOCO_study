import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 
from scipy.spatial.transform import Rotation as R

xml_path = "jdcobot_100_description.urdf"

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
window = glfw.create_window(1200, 1000, "demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
glfw.set_key_callback(window, keyboard)

cam.azimuth = 90 ; cam.elevation = -45 ; cam.distance =  13
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])


#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    #quat = np.array([data.qpos[3],data.qpos[4],data.qpos[5],data.qpos[6]])
    #euler = quat2euler(quat)

    if (data.time>=simend):
        break

    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)

    glfw.poll_events()

glfw.terminate()