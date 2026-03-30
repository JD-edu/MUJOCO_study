import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os

# ---------------------------------------------------------------------------- #
# 1. 초기화 (Initialization)
# ---------------------------------------------------------------------------- #
xml_path = "scene.xml"  # MuJoCo 모델 파일 경로
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
model = mj.MjModel.from_xml_path(abspath)
data = mj.MjData(model)

# ---------------------------------------------------------------------------- #
# 2. 뷰어 설정 (Viewer Setup)
# ---------------------------------------------------------------------------- #
glfw.init()
window = glfw.create_window(1200, 900, "Unitree Stand - Trot Gait", None, None)
glfw.make_context_current(window)
mj.mjv_defaultCamera(cam := mj.MjvCamera())
mj.mjv_defaultOption(opt := mj.MjvOption())
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

# 카메라 위치 설정 (로봇을 잘 볼 수 있도록 조정)
cam.lookat[:] = [0, 0, 0.2] # 로봇의 중심 높이를 약간 위로 조정
cam.distance = 1.5          # 로봇과의 거리
cam.elevation = -25         # 카메라 상하 각도
cam.azimuth = 45           # 카메라 좌우 각도a

# ---------------------------------------------------------------------------- #
# 3. 보행 파라미터 및 제어 게인 정의 (Gait Parameters & Control Gains)
# ---------------------------------------------------------------------------- #
# PD 제어 게인
kp = 50.0  # 위치 게인 (강하게 따라가도록)
kd = 5.0   # 댐핑 게인 (진동을 줄이도록)

# 보행 파라미터
gait_period = 0.25       # 전체 보행 주기 (초) - 이 값을 줄이면 로봇이 더 빨리 움직입니다.
swing_ratio = 0.3       # 보행 주기 중 스윙(Swing) 단계가 차지하는 비율 (0.0 ~ 1.0)
lift_height = 0.03      # 발을 들어 올리는 최대 높이 (미터) - 너무 높으면 불안정해집니다.
step_length = 0.3      # 한 걸음의 길이 (미터) - 너무 길면 로봇이 넘어질 수 있습니다.

# 초기 desired_positions 설정 (로봇의 스탠스 자세)
# 이 값들은 'scene.xml'에 정의된 기본 관절 위치와 일치시키거나,
# 로봇이 안정적으로 서는 최적의 각도로 시행착오를 통해 찾은 값입니다.
# 순서: Back Right(RH) Hip, Knee, Back Left(LH) Hip, Knee, Front Right(RF) Hip, Knee, Front Left(LF) Hip, Knee
initial_desired_positions = np.array([
    0.5, 0.2,     # RH: Back Right  hip, knee
    0.5, 0.2,     # LH: Back Left   hip, knee
    0.5, 0.1,     # RF: Front Right hip, knee
    0.5, 0.1,     # LF: Front Left  hip, knee
])

# 현재 주기 내의 시간 추적 변수
gait_time = 0.0

# ---------------------------------------------------------------------------- #
# 4. 궤적 계산 함수 (Trajectory Calculation Function)
# ---------------------------------------------------------------------------- #
def calculate_swing_trajectory(progress, lift_height, step_length, initial_hip_angle, initial_knee_angle):
    """
    스윙 단계에서 발의 궤적에 따른 힙과 무릎의 목표 각도를 계산합니다.
    Args:
        progress (float): 스윙 단계의 진행률 (0.0 ~ 1.0).
        lift_height (float): 발을 들어 올릴 최대 높이.
        step_length (float): 발을 앞으로 보낼 최대 길이.
        initial_hip_angle (float): 스탠스 시 힙의 기본 각도.
        initial_knee_angle (float): 스탠스 시 무릎의 기본 각도.
    Returns:
        tuple: (목표 힙 각도, 목표 무릎 각도)
    """
    # 발의 수직 오프셋 (Sine 파형으로 부드럽게 올렸다 내림)
    z_offset_ratio = np.sin(progress * np.pi) # 0.0 -> 1.0 (중간) -> 0.0
    
    # 발의 수평 오프셋 (선형적으로 앞으로 이동)
    # 로봇의 몸통이 앞으로 나아가는 것을 고려하여, 발이 뒤에서 앞으로 이동해야 함.
    # 즉, 스탠스 단계에서는 발이 뒤로 움직여야 몸통이 앞으로 갑니다.
    # 스윙 단계에서는 발이 앞으로 이동하여 다음 착지를 준비합니다.
    x_offset_ratio = progress 

    # 조인트 각도 변환 (이 값들은 로봇 모델에 따라 크게 달라지므로, 조정 필수!)
    # 힙 조인트는 주로 X축(앞뒤) 움직임에 기여
    # 무릎 조인트는 Z축(상하) 움직임에 기여 (굽혀지면서 발을 들어 올림)
    
    # hip_offset_factor: 힙이 얼마나 움직여야 step_length를 만족하는지 (시행착오로 조절)
    # knee_offset_factor: 무릎이 얼마나 굽혀져야 lift_height를 만족하는지 (시행착오로 조절)
    hip_offset_angle = x_offset_ratio * step_length * 2.0   # 발을 앞으로 내미는 효과
    knee_offset_angle = z_offset_ratio * -lift_height * 6.0 # 발을 들어 올리는 효과 (무릎이 굽혀지는 방향이 음수라고 가정)

    # 기본 스탠스 각도에서 오프셋을 적용
    # 힙은 앞으로 나아가므로 초기값에서 빼거나 더해야 함 (로봇 축 방향에 따라 다름)
    # 여기서는 힙이 앞으로 움직일 때 각도가 감소한다고 가정하고 뺍니다.
    # 만약 앞으로 움직일 때 각도가 증가한다면 더해야 합니다. (로봇 모델의 관절 축 방향 확인!)
    target_hip_angle = initial_hip_angle - hip_offset_angle # 이 부호를 변경해야 할 수 있습니다.

    # 무릎은 발을 들어 올릴 때 더 굽혀져야 하므로 (음수 오프셋), 초기값에서 더합니다.
    target_knee_angle = initial_knee_angle + knee_offset_angle
    
    return target_hip_angle, target_knee_angle

# ---------------------------------------------------------------------------- #
# 5. 메인 시뮬레이션 루프 (Main Simulation Loop)
# ---------------------------------------------------------------------------- #
while not glfw.window_should_close(window):
    time_prev = data.time

    # --- 시뮬레이션 스텝 (Simulate Step) ---
    # 실제 시간이 아닌 시뮬레이션 내부 시간으로 스텝 진행
    while (data.time - time_prev) < (1.0/60.0):
        mj.mj_step(model, data)
        # gait_time 업데이트: 시뮬레이션 스텝마다 보행 시간 진행
        gait_time = (gait_time + model.opt.timestep) % gait_period

    # --- 보행 단계 계산 (Gait Phase Calculation) ---
    phase_time = gait_time / gait_period  # 보행 주기 내 현재 진행률 (0.0 ~ 1.0)
    
    # 어떤 다리 쌍이 활성화되고(스윙/스탠스), 현재 스윙 단계인지 판별
    is_lf_rh_active = False # Left Front / Right Hind
    is_rf_lh_active = False # Right Front / Left Hind
    is_swing_phase = False  # 현재 활성화된 다리 쌍이 스윙 단계인지
    current_swing_phase_progress = 0.0 # 현재 스윙 단계의 진행률 (0.0 ~ 1.0)

    # 4단계 트롯 보행: LF/RH -> RF/LH 순서
    # 각 단계는 전체 주기의 1/4 (0.25)씩 할당되나, 스윙/스탠스는 swing_ratio에 따라 달라짐
    
    # LF/RH 다리 쌍 활성화 구간 (전체 주기의 0% ~ 50%)
    if 0 <= phase_time < 0.5:
        is_lf_rh_active = True
        # LF/RH 스윙 단계 (전체 주기의 swing_ratio/2 만큼)
        if phase_time < swing_ratio / 2: 
            is_swing_phase = True
            current_swing_phase_progress = phase_time / (swing_ratio / 2)
        # else: LF/RH 스탠스 단계 (나머지 시간)
    # RF/LH 다리 쌍 활성화 구간 (전체 주기의 50% ~ 100%)
    else:
        is_rf_lh_active = True
        # RF/LH 스윙 단계 (전체 주기의 swing_ratio/2 만큼)
        if (phase_time - 0.5) < swing_ratio / 2: 
            is_swing_phase = True
            current_swing_phase_progress = (phase_time - 0.5) / (swing_ratio / 2)
        # else: RF/LH 스탠스 단계

    # --- desired_positions 업데이트 (Update desired_positions) ---
    # 먼저 모든 다리를 기본 스탠스 자세로 초기화
    desired_positions = np.copy(initial_desired_positions)

    # 활성화된 다리 쌍에 따라 목표 각도 설정
    if is_lf_rh_active:
        if is_swing_phase:
            # LF (Front Left) 다리 스윙 궤적 계산
            hip_fl, knee_fl = calculate_swing_trajectory(
                current_swing_phase_progress, lift_height, step_length, 
                initial_desired_positions[6], initial_desired_positions[7] # LF 힙, 무릎 기본 각도
            )
            desired_positions[6] = hip_fl   # Front Left hip
            desired_positions[7] = knee_fl  # Front Left knee

            # RH (Back Right) 다리 스윙 궤적 계산 (LF와 대각선으로 함께 움직임)
            hip_br, knee_br = calculate_swing_trajectory(
                current_swing_phase_progress, lift_height, step_length, 
                initial_desired_positions[0], initial_desired_positions[1] # RH 힙, 무릎 기본 각도
            )
            desired_positions[0] = hip_br   # Back Right hip
            desired_positions[1] = knee_br  # Back Right knee

    elif is_rf_lh_active:
        if is_swing_phase:
            # RF (Front Right) 다리 스윙 궤적 계산
            hip_fr, knee_fr = calculate_swing_trajectory(
                current_swing_phase_progress, lift_height, step_length, 
                initial_desired_positions[4], initial_desired_positions[5] # RF 힙, 무릎 기본 각도
            )
            desired_positions[4] = hip_fr   # Front Right hip
            desired_positions[5] = knee_fr  # Front Right knee

            # LH (Back Left) 다리 스윙 궤적 계산 (RF와 대각선으로 함께 움직임)
            hip_bl, knee_bl = calculate_swing_trajectory(
                current_swing_phase_progress, lift_height, step_length, 
                initial_desired_positions[2], initial_desired_positions[3] # LH 힙, 무릎 기본 각도
            )
            desired_positions[2] = hip_bl   # Back Left hip
            desired_positions[3] = knee_bl  # Back Left knee

    # --- PD 제어: 토크 계산 (PD Control: Compute Torques) ---
    # model.nu는 액츄에이터의 개수를 나타냅니다.
    # 각 액츄에이터는 해당 조인트의 각도와 속도를 센서에서 읽어 PD 제어를 수행합니다.
    for i in range(model.nu):
        # model.nu가 8이므로, i는 0부터 7까지.
        # sensordata는 각 조인트의 [위치, 속도, 가속도] 순서로 저장되어 있다고 가정.
        # 따라서 i*3은 위치, i*3+1은 속도.
        qpos = data.sensordata[i*3]         # 센서 데이터: 조인트 각도
        qvel = data.sensordata[i*3+1]       # 센서 데이터: 조인트 속도
        
        # PD 제어 공식: Torque = Kp * (목표위치 - 현재위치) + Kd * (목표속도 - 현재속도)
        # 목표속도는 0.0으로 설정하여 현재 속도를 줄이도록 합니다.
        torque = kp * (desired_positions[i] - qpos) + kd * (0.0 - qvel)
        data.ctrl[i] = torque # 계산된 토크를 액츄에이터에 적용

    # --- 렌더링 (Render) ---
    # MuJoCo 시뮬레이션 결과를 화면에 그립니다.
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)
    glfw.swap_buffers(window)
    glfw.poll_events()

# 시뮬레이션 종료 시 GLFW 자원 해제
glfw.terminate()