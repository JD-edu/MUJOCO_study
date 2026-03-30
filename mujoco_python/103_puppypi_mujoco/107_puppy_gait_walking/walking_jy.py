import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import math

# Initialize
xml_path = "scene.xml"
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# Viewer
glfw.init()
window = glfw.create_window(1200, 900, "Unitree Walking", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
cam = mj.MjvCamera()
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# 카메라 위치 설정 (로봇을 따라가도록)
cam.lookat[:] = [0, 0, 0]
cam.distance = 1.0
cam.elevation = -20
cam.azimuth = 90

# 기본 서있는 자세 (Standing pose)
standing_positions = np.array([
    0.5,  0.2,     # back Right  hip, knee
    0.5,  0.2,     # back Left  hip, knee 
    0.5,  0.1,     # Front Right  hip, knee
    0.5,  0.1,     # Front Left  hip, knee 
])

# 보행 파라미터 (전진 최적화)
walking_params = {
    'step_frequency': 0.4,       # 걸음 주파수 증가로 더 빠른 전진
    'step_length': 0.15,         # 걸음 길이 증가로 더 큰 전진
    'knee_lift': 0.8,            # 무릎 들어올리기 증가
    'hip_forward': 0.4,          # 엉덩이 앞으로 밀기 증가
    'body_lean': 0.1,            # 몸체 앞으로 기울이기
}

# PD gains (전진을 위해 조정)
kp = 50.0  # Position gain 증가
kd = 3.0   # Damping gain 증가

# 보행 상태 변수
walking_time = 0.0

def generate_forward_gait(time, frequency, leg_index):
    """
    전진에 최적화된 보행 패턴 생성
    """
    # 각 다리별 위상 (전진을 위한 최적화)
    phase_offsets = [0.0, 0.5, 0.25, 0.75]  # BR, BL, FR, FL
    
    phase = (time * frequency + phase_offsets[leg_index]) % 1.0
    
    # 스윙기와 지지기 비율 (40% 스윙, 60% 지지)
    swing_phase = 0.4
    stance_phase = 0.6
    
    if phase < swing_phase:
        # 스윙기: 발을 들어올리고 앞으로 크게 이동
        swing_progress = phase / swing_phase
        
        # 앞으로 크게 이동하는 hip 오프셋
        hip_offset = walking_params['step_length'] * (swing_progress - 0.5)
        
        # 발을 높이 들어올리기
        knee_offset = walking_params['knee_lift'] * np.sin(swing_progress * np.pi)
        
        # 추가 전진 부스트
        forward_boost = walking_params['hip_forward'] * swing_progress
        
    else:
        # 지지기: 땅을 뒤로 밀어서 추진력 생성
        stance_progress = (phase - swing_phase) / stance_phase
        
        # 뒤로 밀면서 추진력 생성
        hip_offset = walking_params['step_length'] * (0.5 - stance_progress * 0.8)
        knee_offset = 0.0
        
        # 지지기에서도 약간의 전진 동작
        forward_boost = walking_params['hip_forward'] * (1.0 - stance_progress) * 0.3
    
    return hip_offset + forward_boost, knee_offset

def get_walking_positions(time):
    """
    전진에 최적화된 관절 위치 계산
    """
    positions = standing_positions.copy()
    
    # 각 다리별로 전진 최적화된 패턴 적용
    for i in range(4):  # 4개의 다리
        hip_idx = i * 2      # 엉덩이 관절 인덱스
        knee_idx = i * 2 + 1 # 무릎 관절 인덱스
        
        # 전진 최적화된 보행 패턴 계산
        hip_offset, knee_offset = generate_forward_gait(time, walking_params['step_frequency'], i)
        
        # 관절 위치 업데이트
        positions[hip_idx] = standing_positions[hip_idx] + hip_offset
        positions[knee_idx] = standing_positions[knee_idx] + knee_offset
        
        # 앞다리는 더 적극적으로 앞으로 (전진 최적화)
        if i >= 2:  # 앞다리 (FR, FL)
            positions[hip_idx] += walking_params['body_lean']
    
    return positions

def update_camera_follow_robot(data, cam):
    """
    로봇을 따라가도록 카메라 업데이트
    """
    robot_pos = data.qpos[0:3]  # x, y, z 위치
    
    # 카메라가 로봇을 따라가도록 설정
    cam.lookat[0] = robot_pos[0]  # x축으로 따라가기
    cam.lookat[1] = robot_pos[1]  # y축으로 따라가기
    cam.lookat[2] = 0.0           # z축은 고정

# 키보드 입력 처리 (전진 최적화)
def keyboard_callback(window, key, scancode, act, mods):
    global walking_params
    
    if act == glfw.PRESS:
        if key == glfw.KEY_SPACE:
            # 스페이스바: 보행 시작/정지
            if walking_params['step_frequency'] > 0:
                walking_params['step_frequency'] = 0.0
                print("보행 정지")
            else:
                walking_params['step_frequency'] = 0.8
                print("보행 시작")
        elif key == glfw.KEY_UP:
            # 위 화살표: 속도 증가
            walking_params['step_frequency'] = min(2.0, walking_params['step_frequency'] + 0.1)
            print(f"보행 속도: {walking_params['step_frequency']:.1f} Hz")
        elif key == glfw.KEY_DOWN:
            # 아래 화살표: 속도 감소
            walking_params['step_frequency'] = max(0.0, walking_params['step_frequency'] - 0.1)
            print(f"보행 속도: {walking_params['step_frequency']:.1f} Hz")
        elif key == glfw.KEY_W:
            # W키: 걸음 길이 증가 (더 빠른 전진)
            walking_params['step_length'] = min(0.3, walking_params['step_length'] + 0.02)
            print(f"걸음 길이: {walking_params['step_length']:.2f}")
        elif key == glfw.KEY_S:
            # S키: 걸음 길이 감소
            walking_params['step_length'] = max(0.05, walking_params['step_length'] - 0.02)
            print(f"걸음 길이: {walking_params['step_length']:.2f}")
        elif key == glfw.KEY_A:
            # A키: 전진 강도 증가
            walking_params['hip_forward'] = min(0.8, walking_params['hip_forward'] + 0.05)
            print(f"전진 강도: {walking_params['hip_forward']:.2f}")
        elif key == glfw.KEY_D:
            # D키: 전진 강도 감소
            walking_params['hip_forward'] = max(0.1, walking_params['hip_forward'] - 0.05)
            print(f"전진 강도: {walking_params['hip_forward']:.2f}")
        elif key == glfw.KEY_R:
            # R키: 리셋
            mj.mj_resetData(model, data)
            print("시뮬레이션 리셋")

# 키보드 콜백 등록
glfw.set_key_callback(window, keyboard_callback)

# 사용법 출력
print("=== 4족 로봇 전진 최적화 시뮬레이션 ===")
print("조작법:")
print("- 스페이스바: 보행 시작/정지")
print("- 위/아래 화살표: 속도 조절")
print("- W/S: 걸음 길이 조절")
print("- A/D: 전진 강도 조절")
print("- R: 리셋")
print("- ESC: 종료")
print("전진에 최적화된 보행 패턴입니다!")
print("보행을 시작하려면 스페이스바를 누르세요!")

# Main loop
while not glfw.window_should_close(window):
    time_prev = data.time
    
    # 보행 시간 업데이트
    walking_time = data.time
    
    # 현재 시간에 따른 목표 관절 위치 계산
    if walking_params['step_frequency'] > 0:
        desired_positions = get_walking_positions(walking_time)
    else:
        desired_positions = standing_positions  # 정지 시에는 기본 자세
    
    # --- PD CONTROL: compute torques ---
    for i in range(model.nu):
        qpos = data.sensordata[i*3]        # Sensor data: joint angle
        qvel = data.sensordata[i*3+1]      # Sensor data: joint velocity
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque

    # --- Simulate ---
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)

    # 카메라가 로봇을 따라가도록 업데이트
    update_camera_follow_robot(data, cam)

    # --- Render ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

glfw.terminate()
