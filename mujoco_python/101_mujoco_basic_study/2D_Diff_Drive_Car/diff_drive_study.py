import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 
from scipy.spatial.transform import Rotation as R

xml_path = "differentialdrive.xml"

simend = 10

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def quat2euler(quat_mujoco):
    #mujocoy quat is constant,x,y,z,
    #scipy quaut is x,y,z,constant
    quat_scipy = np.array([quat_mujoco[3],quat_mujoco[0],quat_mujoco[1],quat_mujoco[2]])

    r = R.from_quat(quat_scipy)
    euler = r.as_euler('xyz', degrees=True)

    return euler
def init_controller(model, datra):
    pass 

def controller(model, data):
    data.ctrl[0] = 10
    data.ctrl[1] = 10 

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
window = glfw.create_window(640, 480, "demo", None, None)
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

    quat = np.array([data.qpos[3],data.qpos[4],data.qpos[5],data.qpos[6]])
    euler = quat2euler(quat)

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

'''
🔍 What is data = mj.MjData(model)?
In MuJoCo, the MjData object is the main simulation state container. It stores all the dynamic quantities that change over time as the simulation runs.
It is always created from a model, like so:

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
📦 What's inside data?
Here are some key fields:

Field	Description
data.qpos	Generalized positions (e.g. joint angles, base position, quaternion)
data.qvel	Generalized velocities (joint speeds, base linear/angular velocity)
data.ctrl	Control inputs (what actuators use)
data.time	Current simulation time
data.xpos, data.xquat, data.site_xpos	World-frame positions of bodies, joints, sites, etc.
data.qacc	Generalized accelerations
data.sensor	Sensor values
data.actuator_force	Forces produced by actuators
❓ Why is the robot moving if you don’t change data.qpos or data.qvel?
Because you are controlling it through data.ctrl[] inside a controller callback that is registered like this:

mj.set_mjcb_control(controller)
And your controller() function is:

def controller(model, data):
    data.ctrl[0] = 10
    data.ctrl[1] = 10 
🔁 What happens here?
On every physics step (mj_step()), MuJoCo:
Calls your controller → sets data.ctrl[:]
Uses actuator model (in this case, <velocity> actuators) to compute torques
Applies those torques to the robot's joints
Integrates physics (updates qpos, qvel, time, etc.)
So, even though you're not directly modifying data.qpos, MuJoCo updates it automatically based on control and physics.

🚘 In your case: Differential Drive
Your XML defines actuators like this:

<actuator>
    <velocity joint="left-wheel" kv="100"/>
    <velocity joint="right-wheel" kv="100"/>
</actuator>
This means:

data.ctrl[0] → desired velocity for left wheel
data.ctrl[1] → desired velocity for right wheel

So this line:

data.ctrl[0] = 10
data.ctrl[1] = 10
… tells MuJoCo:

"Drive both wheels at 10 rad/s → move forward"
As simulation steps run via:

mj.mj_step(model, data)
MuJoCo:
Applies torque to make the wheels spin at 10 rad/s
Integrates motion (including base movement if the robot has a free joint)
Updates data.qpos[] to reflect new position and orientation

📐 Bonus: Why data.qpos[3:7]?
This accesses the base orientation quaternion:
qpos[0:3]: base position (x, y, z)
qpos[3:7]: base orientation as quaternion [w, x, y, z]
That’s why you convert it to Euler angles using:

quat = np.array([data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]])
euler = quat2euler(quat)
✅ Summary

Concept	Explanation
data	Holds all dynamic state of the simulation (positions, velocities, controls, etc.)
Why robot moves	controller() sets data.ctrl[], which MuJoCo uses to drive actuators
mj_step()	Advances physics → updates data.qpos, data.qvel
data.qpos[3:7]	Base orientation quaternion (converted to Euler for logging)
'''

'''
✅ 1. If I have 3 actuators, do I have 3 values in data.ctrl?
✔️ Yes — exactly one data.ctrl[i] per actuator.
MuJoCo allocates the data.ctrl array based on how many <actuator> entries you define in your MJCF XML.

For example:

xml
<actuator>
    <velocity joint="wheel1" kv="100"/>
    <velocity joint="wheel2" kv="100"/>
    <velocity joint="arm_joint" kv="100"/>
</actuator>
Then in Python:

print(len(data.ctrl))  # → 3
And you can access each actuator's control input like:

data.ctrl[0] = ...
data.ctrl[1] = ...
data.ctrl[2] = ...
✅ 2. Does data.qpos have 8 values?
✔️ It depends on your model — but for a free-floating base + 2 hinge joints, the answer is yes: 8 values.
Here’s how it’s typically structured:

Index Range	Meaning
qpos[0:3]	Base position (x, y, z)
qpos[3:7]	Base orientation (quaternion: w, x, y, z)
qpos[7]	Joint 1 (e.g. left-wheel)
qpos[8]	Joint 2 (e.g. right-wheel)
...	More joints if you have them
So if your robot has:
A free joint (for the chassis body) → takes 7 slots in qpos
2 hinge joints (left + right wheels)
Then:

len(data.qpos) == 9
⛳ How to check?
Run:

print(len(data.qpos))
print(model.joint_names)  # See which joints you defined
🧠 Rule of Thumb
data.ctrl.size == model.nu → number of actuators
data.qpos.size == model.nq → number of generalized coordinates
✅ Summary Table
What	What It Represents
data.ctrl[i]	The i-th actuator input (e.g. target velocity or torque)
data.ctrl.size	Number of actuators
data.qpos	All positions in the model: base + joints
data.qpos.size	Depends on number of joints + whether base is fixed or floating
'''





