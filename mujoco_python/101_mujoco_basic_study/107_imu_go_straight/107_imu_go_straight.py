import mujoco as mj
from mujoco.glfw import glfw
import os

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

key_state = {'w': False, 'a': False, 's': False, 'd': False}

def key_callback(window, key, scancode, action, mods):
    global key_state
    if action == glfw.PRESS or action == glfw.RELEASE:
        state = (action == glfw.PRESS)
        if key == glfw.KEY_W: 
            key_state['w'] = state
        elif key == glfw.KEY_A: 
            key_state['a'] = state
        elif key == glfw.KEY_S: 
            key_state['s'] = state
        elif key == glfw.KEY_D: 
            key_state['d'] = state

def mouse_button(window, button, act, mods):
    global button_left, button_middle, button_right
    button_left = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS
    button_middle = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS
    button_right = glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx, lasty, button_left, button_middle, button_right
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos
    if not (button_left or button_middle or button_right): return
    width, height = glfw.get_window_size(window)
    mod_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    if button_right:
        action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, scene, cam)

# Load MJCF
xml_name = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname, xml_name)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)
cam = mj.MjvCamera()
opt = mj.MjvOption()

# GLFW
glfw.init()
window = glfw.create_window(640, 480, "mj Custom Viewer", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=1000)
ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
glfw.set_key_callback(window, key_callback)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Camera config
cam.lookat[:] = [0, 0, 0]
cam.distance = 13
cam.elevation = -45
cam.azimuth = 90
width, height = glfw.get_framebuffer_size(window)
viewport = mj.MjrRect(0, 0, width, height)

keep_going = False

# Main Loop
while not glfw.window_should_close(window):
    mj.mj_step(model, data)

    velocity = 30.0
    left_vel = 0.0
    right_vel = 0.0
    left_handycap = 0.7

    try:
        acc_id = model.sensor(name='imu_acc').id
        gyro_id = model.sensor(name='imu_gyro').id
        acc_data = data.sensordata[acc_id:acc_id+3]
        gyro_data = data.sensordata[gyro_id:gyro_id+3]
        gyro_z = gyro_data[2]  # yaw rate

        #print(f"IMU ACC: {acc_data}, GYRO: {gyro_data}")
        print(f"{gyro_z:.1f}")

        if key_state['w']:
           keep_going = True
        '''
        if key_state['s']:
            left_vel -= velocity
            right_vel -= velocity
        if key_state['a']:
            left_vel -= velocity
            right_vel += velocity
        if key_state['d']:
            left_vel += velocity
            right_vel -= velocity
        '''

    except Exception as e:
        print("IMU 센서 읽기 오류:", e)

    try:
        left_id = model.actuator(name='left-velocity-servo')
        right_id = model.actuator(name='right-velocity-servo')

        if keep_going == True:
            left_vel += velocity * left_handycap
            right_vel += velocity

            # YAW 보정: 오른쪽으로 쏠리면 왼쪽 속도 감소
            correction_gain = 30.0  # 조절 가능
            if gyro_z < 0:
                right_vel += gyro_z * correction_gain
            else:
                left_vel -= gyro_z * correction_gain
            #print(f"{left_vel:.1f} {right_vel:.1f}")
            data.ctrl[left_id.id] = left_vel 
            data.ctrl[right_id.id] = right_vel
            keep_going = False
        else:
            data.ctrl[left_id.id] = 30
            data.ctrl[right_id.id] = -30


       
    except Exception as e:
        print("Actuator 이름 오류:", e)

    mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, mj.mjtCatBit.mjCAT_ALL, scene)
    mj.mjr_render(viewport, scene, ctx)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.destroy_window(window)
glfw.terminate()
