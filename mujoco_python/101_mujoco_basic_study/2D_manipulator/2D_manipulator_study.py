import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os 

xml_path = 'manipulator.xml'

'''
They are user-defined callback functions intended for:
initializing and running robot controller.
allowing MuJoCo to call your logic at the right times
🔹 init_controller(model, data)
✅ When it runs:
It is called once at the start of the program
🧠 Purpose:
Initialize control variables, loggers, parameters
Reset positions, set gains, initialize trajectory planners

🔹 controller(model, data)
✅ When it runs:
Called at every simulation step
This sets controller() as MuJoCo’s callback for control:
Called before each physics step to compute actuator forces or control signals.
🧠 Purpose:
Read current state (e.g. data.qpos, data.qvel)
Compute control input (e.g. PD control, torque)
Write to data.ctrl[:]
'''
def init_controller(model, data):
    print("init_controller")

def controller(model, data):
    #print("controller")
    pass

def keyboard(widow, key, scancode, act , mods):
    if act == glfw.PREWSS and key == glfw.KEY_SPACE:
        print("space")
        mj.mj_resetData(model, data)

dirname = os.path.dirname(__file__)
#print(dirname)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath
#print(xml_path)

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

glfw.init()
window = glfw.create_window(640, 480, "demo", None, None)
'''
🔹 glfw.make_context_current(window)
✅ What it does:
This sets the OpenGL context associated with your window as the current context for rendering and drawing.
📌 Why it's important:
OpenGL needs to know which window you're about to draw into. Without this line, OpenGL (and MuJoCo's renderer) won't know where to send the rendered image.
🧠 In short:
“Make this GLFW window the active drawing target.”
'''
glfw.make_context_current(window)
'''
🔹 glfw.swap_interval(1)
✅ What it does:
This sets the vertical synchronization (V-sync) interval.
1 means enable V-sync:
OpenGL waits for the monitor’s vertical refresh (usually 60Hz) before swapping buffers.
This limits the frame rate to match the monitor and reduces tearing.
0 would mean no sync, maximum frame rate.
🧠 In short:
“Wait for the display refresh before showing each new frame” — smoother, tear-free visuals.
'''
glfw.swap_interval(1)

mj.mjv_defaultCamera(cam)
'''
✅ What it does:
Initializes a MjvCamera structure (cam) with default settings.
📌 Why it's needed:
Before rendering anything, the camera object must be initialized.
You can later change:
cam.azimuth: horizontal rotation
cam.elevation: vertical rotation
cam.distance: distance to look-at point
cam.lookat: point to center the camera view on
'''
mj.mjv_defaultOption(opt)
'''
⚙️ 2. mj.mjv_defaultOption(opt)
✅ What it does:
Initializes a MjvOption structure (opt) with default visualization options.
🧠 What it controls:
Whether to show:
Contact forces
Joint frames
Transparency
Labels, etc.
These options affect how objects are rendered in the scene.
'''
scene = mj.MjvScene(model, maxgeom=10000)
'''
✅ What it does:
Creates a renderable scene object, which holds all the geometries (geom) to be drawn.
📌 maxgeom=10000:
This sets how many visual elements (boxes, spheres, lines, contacts, etc.) the scene can hold.
It must be large enough to fit all rendered geometry.
🧠 Role:
MjvScene is the intermediate structure between physics (model, data) and rendering (mjr_render).
'''
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
'''
🎨 4. context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
✅ What it does:
Creates a rendering context used to draw the scene on screen.
🧠 What it stores:
Font rendering information
OpenGL shader state
GPU buffer settings
📌 mjFONTSCALE_150:
Sets font size scale (150%) for on-screen text (e.g. joint names, status).
'''
# install GLFW keyboard callbacks
glfw.set_key_callback(window, keyboard)

# Example on how to set camera configuration
#initialize the controller here. This function is called once, in the beginning
cam.azimuth = 89.83044433593757 ; cam.elevation = -89.0 ; cam.distance =  5.04038754800176
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

N = 500
q0_start = 0
q0_end = 1.57
q1_start = 0
q1_end = -2*3.14
q0 = np.linspace(q0_start,q0_end,N)
q1 = np.linspace(q1_start,q1_end,N)
#print(q0)
#print(q1)
'''
N = 500
Sets the number of steps (or frames) in the simulation.

You are planning to simulate 500 time steps.

q0_start = 0
q0_end = 1.57
These define the starting and ending position for the first joint (q0).

1.57 radians ≈ 90 degrees → meaning the first joint will go from 0° to 90°.

q1_start = 0
q1_end = -2 * 3.14
These define the start and end positions for the second joint (q1).

-2 * 3.14 ≈ -6.28 radians → ~-360°, so the second joint will rotate one full turn in the negative direction.

q0 = np.linspace(q0_start, q0_end, N)
q1 = np.linspace(q1_start, q1_end, N)
✅ What it does:
Generates N = 500 equally spaced values from start to end for each joint.
q0: smoothly interpolates from 0 → 90°
q1: smoothly interpolates from 0 → -360°
🔁 These arrays are then used in your simulation loop:

data.qpos[0] = q0[i]
data.qpos[1] = q1[i]
So at each time step i, your manipulator's joints are set to the corresponding q0[i], q1[i].
'''

#set the controller
mj.set_mjcb_control(controller)

#initialize
data.qpos[0] = q0_start
data.qpos[1] = q1_start
i = 0
time = 0
dt = 0.001
'''
🧠 Explanation Line by Line
🔹 data.qpos[0] = q0_start
Sets the initial position of joint 0 to q0_start (which is 0 radians in your earlier code).
data.qpos is an array that holds generalized coordinates (joint positions) for all DoFs in the model.
🔹 data.qpos[1] = q1_start
Sets the initial position of joint 1 to q1_start (also 0 radians in your code).
🔹 i = 0
This is the frame index.
You’ll increment i during simulation to move through q0[i] and q1[i] arrays — one step at a time.
🔹 time = 0
Initializes simulation time.
You’ll manually increment this with dt to keep track of simulated time.
🔹 dt = 0.001
This is your time step, or how much simulated time passes per iteration.
0.001 seconds = 1 ms → corresponds to a 1000 Hz simulation rate.
✅ Purpose of This Block
To initialize joint positions, simulation time, and loop counters before entering the simulation loop.
'''

while not glfw.window_should_close(window):
    time_prev = time
    '''
    🧩 Code Summary
    while not glfw.window_should_close(window):  # 👈 Outer loop
        time_prev = time

        while (time - time_prev < 1.0 / 60.0):   # 👈 Inner loop
            # simulate one step at dt=0.001
            ...
    🔁 Purpose of the outer loop
    while not glfw.window_should_close(window):
    This is your main simulation + rendering loop.
    Runs until the GLFW window is closed.
    In each loop:
    Advances simulation by 1/60 seconds worth of time.
    Renders a single frame to screen (OpenGL).
    Handles mouse, keyboard, etc.
    🖼️ One rendered frame per outer loop — synced with monitor refresh (60 FPS).
    ⏱️ Purpose of the inner loop
    while (time - time_prev < 1.0 / 60.0):
    Simulates a small amount of physics multiple times within a single frame.
    dt = 0.001, so ~16 steps needed to simulate 1/60th of a second.
    Here you're doing:
    mj.mj_forward(model, data)
    time += dt
    Why?
    Real-time rendering (60 FPS) is too slow for accurate physics.
    🔁 So we “substep” the physics several times per frame to improve simulation accuracy.
    🧠 Analogy
    Imagine a car driving and you're drawing it on paper 60 times a second (outer loop).
    But physics changes faster — say, 1,000 times a second, like:
    Acceleration
    Contact forces
    Torques
    So for each drawing (frame), you update physics ~16 times (inner loop).
    🔄 Normally, You Would Use:
    mj.mj_step(model, data)
    instead of mj_forward() — because mj_step():
    Advances full simulation (forces, velocity, contacts)
    Moves simulation data.time forward
    But in your code, you're manually controlling joint positions with:
    data.qpos[0] = q0[i]
    So you're only using mj_forward() to compute kinematics (no dynamics).
    ✅ Summary: Two While Loops
    Loop	Frequency	Purpose
    Outer Loop (while not glfw.window_should_close)	~60 Hz	Renders a frame, polls user input
    Inner Loop (while time - time_prev < 1/60.0)	~1000 Hz (dt=0.001)	Advances simulation in fine steps within a single frame
    This pattern ensures:
    ✅ Smooth rendering
    ✅ High simulation accuracy
    ✅ Proper control and interaction speed

    '''

    while (time - time_prev < 1.0/60.0):
        data.qpos[0] = q0[i]
        data.qpos[1] = q1[i]
        mj.mj_forward(model,data)
        time +=dt
        # mj.mj_step(model, data)

    i +=1
    print(data.site_xpos[0])
    if (i>=N):
        break

    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    #print(viewport_height, viewport_width)

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)
    # get key board and mouse 
    glfw.poll_events()
glfw.terminate()
    


