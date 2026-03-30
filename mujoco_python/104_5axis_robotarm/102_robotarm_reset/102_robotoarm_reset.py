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

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "jdCobot100", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
cam = mj.MjvCamera()
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# 카메라 위치 설정 (optional)
cam.lookat[:] = [0, 0, 0]
cam.distance = 1.0
cam.elevation = -20
cam.azimuth = 90

# 시행착오법으로  최적의 값을 찾았음, 역시 desired_position의 각 값이 각 joint에 어떤 것에 
# 해당하는 것인지도 시행착오법으로 찾았음 
reset_positions = np.array([
    0,  
    1.5708,      
    1.5708,  
    1.5708,    
    1.5708,  
    1.5708,       
])

target_positions = np.array([
    1.5708,  
    1,      
    1,  
    1,    
    1,  
    1,       
])

# PD gains
kp = 100.0  # Position gain
kd = 5.0   # Damping gain

for i in range(model.nu):
    joint_id = model.actuator_trnid[i][0]
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr] = reset_positions[i]

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time
    


    # --- PD CONTROL: compute torques ---
    for i in range(model.nu):
        #qpos = data.sensordata[i*3]        # Sensor data: joint angle
        #qvel = data.sensordata[i*3+1]  # Sensor data: joint velocity
        joint_id = model.actuator_trnid[i][0]
        qpos_adr = model.jnt_qposadr[joint_id]
        qvel_adr = model.jnt_dofadr[joint_id]

        qpos = data.qpos[qpos_adr]
        qvel = data.qvel[qvel_adr]
        pos_error = target_positions[i] - qpos
        vel_error = -qvel

        torque = kp * pos_error + kd * vel_error
        data.ctrl[i] = torque  # match ctrlrange!
        
    print(data.ctrl[0])
    
    # --- Simulate ---
    while (data.time - time_prev) < (1.0/600.0):
        mj.mj_step(model, data)

    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
