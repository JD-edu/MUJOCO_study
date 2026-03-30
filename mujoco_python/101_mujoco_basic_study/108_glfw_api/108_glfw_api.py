import mujoco as mj
from mujoco.glfw import glfw
import os

# 전역 변수
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# 키보드 상태
key_state = {
    'w': False, 'a': False, 's': False, 'd': False
}

def key_callback(window, key, scancode, action, mods):
    """키보드 입력 콜백 함수"""
    global key_state
    
    if action == glfw.PRESS:
        if key == glfw.KEY_W:
            key_state['w'] = True
        elif key == glfw.KEY_A:
            key_state['a'] = True
        elif key == glfw.KEY_S:
            key_state['s'] = True
        elif key == glfw.KEY_D:
            key_state['d'] = True
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
    
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:
            key_state['w'] = False
        elif key == glfw.KEY_A:
            key_state['a'] = False
        elif key == glfw.KEY_S:
            key_state['s'] = False
        elif key == glfw.KEY_D:
            key_state['d'] = False

def mouse_button_callback(window, button, action, mods):
    """마우스 버튼 콜백 함수"""
    global button_left, button_middle, button_right
    
    button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
    
    # 마우스 위치 업데이트
    glfw.get_cursor_pos(window)

def cursor_pos_callback(window, xpos, ypos):
    """마우스 이동 콜백 함수"""
    global lastx, lasty, button_left, button_middle, button_right
    
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos
    
    # 버튼이 눌려있지 않으면 무시
    if not (button_left or button_middle or button_right):
        return
    
    # 윈도우 크기 가져오기
    width, height = glfw.get_window_size(window)
    
    # Shift 키 상태 확인
    mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                 glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
    
    # 동작 결정
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
    
    # 카메라 이동
    mj.mjv_moveCamera(model, action, dx/height, dy/height, scene, cam)

def scroll_callback(window, xoffset, yoffset):
    """마우스 스크롤 콜백 함수"""
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 * yoffset, scene, cam)

def main():
    global model, data, cam, scene
    
    # MJCF 모델 로딩
    xml_path = "2wheel_robot.xml"  # XML 파일 경로
    model = mj.MjModel.from_xml_path(xml_path)
    data = mj.MjData(model)
    
    # GLFW 초기화
    if not glfw.init():
        raise Exception("GLFW 초기화 실패")
    
    # 윈도우 생성
    window = glfw.create_window(1200, 900, "MuJoCo 시뮬레이션", None, None)
    if not window:
        glfw.terminate()
        raise Exception("윈도우 생성 실패")
    
    # OpenGL 컨텍스트 설정
    glfw.make_context_current(window)
    glfw.swap_interval(1)  # V-Sync 활성화
    
    # 콜백 함수 등록
    glfw.set_key_callback(window, key_callback)
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    glfw.set_cursor_pos_callback(window, cursor_pos_callback)
    glfw.set_scroll_callback(window, scroll_callback)
    
    # MuJoCo 뷰어 구성 요소 초기화
    scene = mj.MjvScene(model, maxgeom=1000)
    cam = mj.MjvCamera()
    ctx = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
    
    # 카메라 초기 설정
    mj.mjv_defaultCamera(cam)
    cam.lookat[:] = [0, 0, 0.5]
    cam.distance = 3.0
    cam.elevation = -20
    cam.azimuth = 90
    
    # 메인 루프
    while not glfw.window_should_close(window):
        # 물리 시뮬레이션 스텝
        mj.mj_step(model, data)
        
        # 키보드 입력에 따른 제어
        velocity = 10.0
        left_vel = 0.0
        right_vel = 0.0
        
        if key_state['w']:  # 전진
            left_vel += velocity
            right_vel += velocity
        if key_state['s']:  # 후진
            left_vel -= velocity
            right_vel -= velocity
        if key_state['a']:  # 좌회전
            left_vel -= velocity
            right_vel += velocity
        if key_state['d']:  # 우회전
            left_vel += velocity
            right_vel -= velocity
        
        # 액추에이터 제어 (예시)
        try:
            left_id = model.actuator(name='left-velocity-servo')
            right_id = model.actuator(name='right-velocity-servo')
            data.ctrl[left_id.id] = left_vel
            data.ctrl[right_id.id] = right_vel
        except:
            pass  # 액추에이터가 없으면 무시
        
        # 프레임버퍼 크기 가져오기
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
        
        # 장면 업데이트
        mj.mjv_updateScene(model, data, mj.MjvOption(), None, cam, 
                          mj.mjtCatBit.mjCAT_ALL, scene)
        
        # 렌더링
        mj.mjr_render(viewport, scene, ctx)
        
        # 버퍼 교체 및 이벤트 처리
        glfw.swap_buffers(window)
        glfw.poll_events()
    
    # 정리
    glfw.destroy_window(window)
    glfw.terminate()

if __name__ == "__main__":
    main()