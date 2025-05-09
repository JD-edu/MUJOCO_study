import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 

xml_path = 'ball.xml'
simend = 10 

def init_controller(model, data):
    pass

def controller(model, data):
    vx = data.qvel[0]
    vy = data.qvel[1]
    vz = data.qvel[2]
    v = np.sqrt(vx**2+vy**2+vz**2)
    c = 0.5
    '''
    💡 What is xfrc_applied?
    A MuJoCo array: data.xfrc_applied[body_id][0:6]
    [0:3] = force (x, y, z)
    [3:6] = torque (x, y, z)
    🧠 It applies an external force/torque directly to a body in the world coordinate frame.
    '''
    data.xfrc_applied[1][0] = -c*vx*v
    data.xfrc_applied[1][1] = -c*vx*v
    data.xfrc_applied[1][2] = -c*vx*v

def print_data_summary(data):
    print("\n=== Key simulation state ===")
    
    print("time:", data.time)

    print("qpos (positions):", data.qpos)
    print("qvel (velocities):", data.qvel)
    print("qacc (accelerations):", data.qacc)

    print("ctrl (control inputs):", data.ctrl)
    print("actuator_force:", data.actuator_force)

    print("xpos (body positions):", data.xpos)
    print("xquat (body orientations):", data.xquat)

    print("xfrc_applied (external force):", data.xfrc_applied)

    print("sensor data (if any):", data.sensordata)

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        
        data.qpos[0] = 0
        data.qpos[2] = 0.1
        data.qvel[0] = 2
        data.qvel[2] = 5

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname+"/"+xml_path)
xml_path = abspath

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()


for attr in dir(opt):
    if not attr.startswith("_"):
        print(attr)


'''
🧠 Here's the short answer:
You are using a <body> with a <joint type="free"/>, meaning the object is free-floating in 3D space. This automatically adds:

7 generalized position variables (qpos)

6 generalized velocity variables (qvel)

🧩 Details of the State Variables
✅ qpos: size = 7 for a free-floating body

Index	Meaning	Example Value
0	x-position	0.0
1	y-position	0.0
2	z-position	1.0
3	orientation w	1.0
4	orientation x	0.0
5	orientation y	0.0
6	orientation z	0.0
→ So the first 3 values are position, and the next 4 are a quaternion representing orientation.

✅ qvel: size = 6

Index	Meaning	
0–2	Linear velocity (vx, vy, vz)	
3–5	Angular velocity (wx, wy, wz)	
📐 Why this happens
In your XML (ball.xml), you likely have something like:

xml
<body name="ball" pos="0 0 1">
    <joint type="free"/>
    <geom type="sphere" size="0.1"/>
</body>
The moment you use <joint type="free"/>, MuJoCo treats this as a floating base, and allocates 7 qpos and 6 qvel values for it — even if there are no additional joints.

This is how MuJoCo simulates 3D rigid body motion (position + orientation + linear/angular velocity).

'''
#print_data_summary(data)
'''
🧠 qpos vs xpos — Key Concept

Quantity	Meaning	Frame	Data Type
qpos	Generalized position — joint states, base pose (including orientation as quaternion)	Configuration space	Low-dimensional
xpos	World position of a body — Cartesian (x, y, z) position	World space	3D Cartesian
🔹 qpos: Generalized Coordinates
data.qpos stores the minimal coordinates needed to describe the full system state.

Includes:

Base position and orientation (if free body)

Joint angles or positions

📦 Examples

System	qpos contents
Free-floating ball	7 values (x, y, z, qw, qx, qy, qz)
2-joint arm	2 values (joint angles)
Mobile robot + arm	7 (base) + n (joints)
→ These are the internal DOF-based coordinates MuJoCo simulates.

🔹 xpos: Cartesian Position of Bodies
data.xpos is a matrix of shape (nbody, 3) (for nbody total bodies in your model).

It holds the world-space position of each body’s origin, computed from qpos via forward kinematics.

📌 Think of it like:
“Where is this body in the 3D world right now?”

Example:

data.xpos[1]  # position of body 1 (e.g., the floating ball)
This is a 3D point [x, y, z].

📷 Analogy: qpos is like instructions, xpos is like final camera view
qpos = joint angles + root pose

xpos = where things ended up in 3D space after computing forward kinematics

✅ Summary Table

Field	Description	Example Use
data.qpos	Joint positions + base pose	Set robot configuration or control
data.xpos[i]	World position of body i	Visualize, measure, compute distances
✅ When to use which?

Goal	Use
Move robot	Modify qpos (or ctrl)
Track robot position in 3D	Read xpos
Compute distance to a target	Use xpos
Apply controller based on body position	Use xpos or site_xpos
'''

glfw.init()
window = glfw.create_window(640, 480, "demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

glfw.set_key_callback(window, keyboard)

cam.azimuth = 90.0
cam.elevation = -45
cam.distance = 2
cam.lookat = np.array([0., 0.0, 0.0])

data.qpos[0] = 0
data.qpos[2] = 0.1
data.qvel[0] = 2
data.qvel[2] = 5

mj.set_mjcb_control(controller)

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data) 
    
    if(data.time>=simend):
        break

      # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()