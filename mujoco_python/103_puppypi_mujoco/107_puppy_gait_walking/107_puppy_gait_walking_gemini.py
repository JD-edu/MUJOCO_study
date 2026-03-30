import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time

# Initialize MuJoCo
xml_name = "scene.xml" # Use your puppy.xml file
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_name) # Corrected path
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Initialize GLFW
glfw.init()
window = glfw.create_window(1200, 900, "Puppy Simple Gait Walking", None, None)
glfw.make_context_current(window)

# Viewer setup
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# Camera position (adjusted for better viewing of a quadraped)
cam.lookat[:] = [0, 0, 0.1] # Look slightly above ground
cam.distance = 1.0 # Closer view
cam.elevation = -30 # Look down more
cam.azimuth = 45 # Slightly angled view

# PD gains
kp = 40.0  # Position gain
kd = 2.0   # Damping gain

# --- Gait Walking Parameters (Simple Example) ---
# Actuator order from puppy.xml:
# rb_joint1, rb_joint2 (Right Back Hip, Knee)      -> Indices 0, 1
# lb_joint1, lb_joint2 (Left Back Hip, Knee)       -> Indices 2, 3
# rf_joint1, rf_joint2 (Right Front Hip, Knee)     -> Indices 4, 5
# lf_joint1, lf_joint2 (Left Front Hip, Knee)      -> Indices 6, 7

# Stand-up position (aligned with actuator order)
stand_qpos = np.array([
    0.5, 0.4,   # RB Hip, Knee (actuator 0, 1)
    0.5, 0.4,   # LB Hip, Knee (actuator 2, 3)
    0.5, 0.3,   # RF Hip, Knee (actuator 4, 5)
    0.5, 0.3    # LF Hip, Knee (actuator 6, 7)
])

# Step parameters
step_height = 0.05 # How high the foot lifts
gait_cycle_time = 0.5 # Time for one full gait cycle (e.g., all 4 legs complete their sequence)

# --- Define Actuator Indices for Each Leg ---
# This mapping makes the gait logic clearer and less error-prone.
# Based on the assumed order: rb, lb, rf, lf
LEG_JOINT_INDICES = {
    "rb": {"hip": 0, "knee": 1},
    "lb": {"hip": 2, "knee": 3},
    "rf": {"hip": 4, "knee": 5},
    "lf": {"hip": 6, "knee": 7}
}

# --- Gait Phase Order (Example: Trotting gait - diagonal pairs lift) ---
# This is a simple sequential lift pattern, not a real trot.
# A more realistic trot would involve simultaneous lifting of diagonal pairs.
# For simplicity here, we'll keep the sequential lift to demonstrate the indexing.
# A classic trot involves (RF & LB) then (LF & RB).
# Let's define a sequential pattern for demonstration based on your request.
# Example: RF_then_LB -> LF_then_RB -> RF_then_LB -> LF_then_RB
# Let's adjust to be more explicit with individual leg lift timing.

# Sequential Lift Phases (adjust order and durations as desired)
# Phase 0.00 - 0.25: Lift Right Front (rf)
# Phase 0.25 - 0.50: Lift Left Back (lb)
# Phase 0.50 - 0.75: Lift Left Front (lf)
# Phase 0.75 - 1.00: Lift Right Back (rb)

# Main simulation loop
frame_counter = 0
while not glfw.window_should_close(window):
    time_prev = data.time
    
    # --- GAIT CONTROL LOGIC ---
    phase = (data.time % gait_cycle_time) / gait_cycle_time
    current_desired_qpos = np.copy(stand_qpos) # Start each cycle from stand position

    # Right Front (RF) leg lift
    if 0.00 <= phase < 0.25:
        t_segment = (phase - 0.00) / 0.25 # Scale phase to 0-1 for this segment
        lift_angle = -0.2 * np.sin(np.pi * t_segment) # Sinusoidal lift: 0 -> -0.5 (max lift) -> 0
        
        current_desired_qpos[LEG_JOINT_INDICES["rf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rf"]["hip"]] + lift_angle
        current_desired_qpos[LEG_JOINT_INDICES["rf"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["rf"]["knee"]] + lift_angle

    # Left Back (LB) leg lift
    elif 0.25 <= phase < 0.50:
        t_segment = (phase - 0.25) / 0.25
        lift_angle = -0.2 * np.sin(np.pi * t_segment)
        
        current_desired_qpos[LEG_JOINT_INDICES["lb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lb"]["hip"]] + lift_angle
        current_desired_qpos[LEG_JOINT_INDICES["lb"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["lb"]["knee"]] + lift_angle
        
    # Left Front (LF) leg lift
    elif 0.50 <= phase < 0.75:
        t_segment = (phase - 0.50) / 0.25
        lift_angle = -0.2 * np.sin(np.pi * t_segment)
        
        current_desired_qpos[LEG_JOINT_INDICES["lf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lf"]["hip"]] + lift_angle
        current_desired_qpos[LEG_JOINT_INDICES["lf"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["lf"]["knee"]] + lift_angle

    # Right Back (RB) leg lift
    elif 0.75 <= phase < 1.0:
        t_segment = (phase - 0.75) / 0.25
        lift_angle = -0.2 * np.sin(np.pi * t_segment)
        
        current_desired_qpos[LEG_JOINT_INDICES["rb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rb"]["hip"]] + lift_angle
        current_desired_qpos[LEG_JOINT_INDICES["rb"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["rb"]["knee"]] + lift_angle


    # --- PD CONTROL: compute torques for all actuators ---
    for i in range(model.nu): # Iterate through all actuators
        # The index `i` directly corresponds to the actuator's position in the XML and in data.ctrl / data.sensordata
        
        qpos = data.sensordata[i*3]     # Assuming sensor for position is at i*3
        qvel = data.sensordata[i*3+1]   # Assuming sensor for velocity is at i*3 + 1
        
        # Compute torque based on the current desired position for this actuator
        torque = kp * (current_desired_qpos[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque

    # --- Simulate ---
    while (data.time - time_prev) < (1.0/60.0): # Run at 60Hz visual update
        mj.mj_step(model, data)

    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()