import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time

# -------------------- MuJoCo Viewer Callbacks --------------------
class MuJoCoViewerCallbacks:
    def __init__(self, model, cam, scn, data):
        self.model = model
        self.cam = cam
        self.scn = scn
        self.data = data

        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.lastx = 0
        self.lasty = 0

    def keyboard_callback(self, window, key, scancode, action, mods):
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(window, True)
        if key == glfw.KEY_R and action == glfw.PRESS:
            mj.mj_resetData(self.model, self.data)
            self._set_initial_pose_in_data() # 리셋 후 초기 자세를 다시 적용
            mj.mj_forward(self.model, self.data) # 변경사항 적용

    def mouse_button_callback(self, window, button, action, mods):
        self.button_left = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
        self.button_middle = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
        self.button_right = (glfw.get_mouse_button(window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)
        self.lastx, self.lasty = glfw.get_cursor_pos(window)

    def mouse_move_callback(self, window, xpos, ypos):
        dx = xpos - self.lastx
        dy = ypos - self.lasty
        self.lastx = xpos
        self.lasty = ypos

        if not (self.button_left or self.button_middle or self.button_right):
            return

        width, height = glfw.get_window_size(window)
        mod_shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS or
                     glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)

        action = mj.mjtMouse.mjMOUSE_NONE
        if self.button_right:
            action = mj.mjtMouse.mjMOUSE_MOVE_H if mod_shift else mj.mjtMouse.mjMOUSE_MOVE_V
        elif self.button_left:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H if mod_shift else mj.mjtMouse.mjMOUSE_ROTATE_V
        elif self.button_middle:
            action = mj.mjtMouse.mjMOUSE_ZOOM
        
        mj.mjv_moveCamera(self.model, action, dx / height, dy / height, self.scn, self.cam)
        
    def scroll_callback(self, window, xoffset, yoffset):
        action = mj.mjtMouse.mjMOUSE_ZOOM
        mj.mjv_moveCamera(self.model, action, 0.0, -0.05 * yoffset, self.scn, self.cam)

    # -------------------- 초기 자세 설정 함수 --------------------
    def _set_initial_pose_in_data(self):
        """
        로봇의 초기 자세 (qpos)를 설정하고 물리 엔진을 업데이트합니다.
        이 함수는 시뮬레이션 시작 전에 딱 한 번 호출되어야 합니다.
        로봇의 발이 지면에 닿고 안정적으로 서있는 자세여야 합니다.
        """
        print(f"모델의 qpos 자유도 수 (model.nq): {self.model.nq}") 
        
        if self.model.nq >= 7: # 최소한 베이스 위치/방향을 제어할 수 있는 모델인지 확인
            initial_z_height = 0.18 # PuppyPi 모델에 따라 조정. 지면과의 초기 충돌 방지
            self.data.qpos[0] = 0.0  # x 위치
            self.data.qpos[1] = 0.0  # y 위치
            self.data.qpos[2] = initial_z_height # Z 위치

            self.data.qpos[3] = 1.0  # qw (쿼터니언: 회전 없음)
            self.data.qpos[4] = 0.0  # qx
            self.data.qpos[5] = 0.0  # qy
            self.data.qpos[6] = 0.0  # qz

            # 2. 각 조인트의 초기 각도 설정
            # ***** 이 값은 가장 중요합니다. MuJoCo viewer에서 직접 확인한 안정적인 서있는 자세 값을 사용하세요. *****
            standing_joint_angles = np.array([
                0.5, 0.4,   # RB Hip, Knee (액추에이터 0, 1)
                0.5, 0.4,   # LB Hip, Knee (액추에이터 2, 3)
                0.5, 0.3,   # RF Hip, Knee (액추에이터 4, 5)
                0.5, 0.3    # LF Hip, Knee (액추에이터 6, 7)
            ])
            
            num_joint_dofs = self.model.nq - 7 
            if num_joint_dofs > 0:
                if num_joint_dofs == len(standing_joint_angles):
                    self.data.qpos[7:] = standing_joint_angles
                elif num_joint_dofs < len(standing_joint_angles):
                    self.data.qpos[7 : 7 + num_joint_dofs] = standing_joint_angles[:num_joint_dofs]
                    print(f"경고: 모델의 조인트 수({num_joint_dofs})가 설정된 초기 각도({len(standing_joint_angles)})보다 적습니다. 일부만 적용됩니다.")
                else:
                    self.data.qpos[7 : 7 + len(standing_joint_angles)] = standing_joint_angles
                    print(f"정보: 모델에 추가 조인트가 있을 수 있으며, 이들은 기본값 (0)으로 유지됩니다. ({num_joint_dofs - len(standing_joint_angles)}개)")
            else:
                print("경고: 모델에 조인트 자유도가 없습니다 (nq - 7 <= 0). 조인트 각도를 설정할 수 없습니다.")
            
            print(f"로봇 초기 Z 위치를 {initial_z_height:.3f}m로 설정했습니다.")
        else:
            print(f"경고: 모델의 자유도(nq={self.model.nq})가 너무 낮아 베이스 위치를 설정할 수 없습니다. (최소 7 필요)")

# -------------------- Main Function --------------------
def main():
    # 1. XML 파일 경로 설정
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    XML_FILENAME = "scene.xml"
    XML_PATH = os.path.join(SCRIPT_DIR, XML_FILENAME)

    # 2. MuJoCo 모델 로드
    try:
        model = mj.MjModel.from_xml_path(XML_PATH)
        data = mj.MjData(model)
        print(f"\n모델 '{XML_FILENAME}' 로드 성공: {XML_PATH}")
    except Exception as e:
        print(f"\n오류: MuJoCo 모델 로드 실패!")
        print(f"  경로: {XML_PATH}")
        print(f"  에러: {e}")
        if "meshes" in str(e) or "failed to load" in str(e).lower():
            print("\n팁: XML 파일에 'meshdir=\"meshes\"'가 포함되어 있습니다.")
            print("    해당 .stl 메쉬 파일들이 XML 파일과 같은 디렉토리 내의 'meshes' 폴더 안에 있는지 확인해주세요.")
        return

    # 3. GLFW 초기화 및 윈도우 생성
    if not glfw.init():
        print("GLFW 초기화 실패")
        return

    window_width, window_height = 1280, 960
    window_title = f"MuJoCo Simulation: {XML_FILENAME}"
    window = glfw.create_window(window_width, window_height, window_title, None, None)
    if not window:
        glfw.terminate()
        print("창 생성 실패")
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # 4. MuJoCo 뷰어 관련 객체 초기화
    cam = mj.MjvCamera()
    opt = mj.MjvOption()
    scn = mj.MjvScene(model, 1000)
    con = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_100)

    # 5. 기본 카메라 및 렌더링 옵션 설정
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)

    # 6. 초기 카메라 시점 조정
    cam.azimuth = 90
    cam.elevation = -45
    cam.distance = 2.0
    cam.lookat[0] = 0.0
    cam.lookat[1] = 0.0
    cam.lookat[2] = 0.5 

    # 7. 콜백 함수 객체 생성 및 초기 자세 설정
    viewer_callbacks = MuJoCoViewerCallbacks(model, cam, scn, data)
    viewer_callbacks._set_initial_pose_in_data() 
    mj.mj_forward(model, data) 

    # 8. 콜백 함수 등록
    glfw.set_key_callback(window, viewer_callbacks.keyboard_callback)
    glfw.set_cursor_pos_callback(window, viewer_callbacks.mouse_move_callback)
    glfw.set_mouse_button_callback(window, viewer_callbacks.mouse_button_callback)
    glfw.set_scroll_callback(window, viewer_callbacks.scroll_callback) 
    

    # -------------------- PD 제어 및 보행 매개변수 --------------------
    # PD 이득 (Gains) - 로봇의 반응성 및 안정성에 가장 큰 영향
    kp = 100.0  # 위치 이득 (Position gain) - **더 높여서 빠르게 반응하도록**
    kd = 8.0    # 감쇠 이득 (Damping gain) - **진동 억제를 위해 약간 더 높임**

    # 액추에이터 순서와 일치하는 스탠드업 자세
    # ***** 이 값은 가장 중요합니다. 실제 PuppyPi가 안정적으로 서있는 각도를 찾아 업데이트해야 합니다. *****
    stand_qpos = np.array([
        0.5, 0.4,   # RB Hip, Knee
        0.5, 0.4,   # LB Hip, Knee
        0.5, 0.3,   # RF Hip, Knee
        0.5, 0.3    # LF Hip, Knee
    ])

    # 스텝 파라미터 - 보행 궤적의 형태 (성큼성큼)
    step_height = 0.04 # 발을 들어 올리는 높이 (약간 높여 장애물 회피)
    step_length = 0.15 # 발을 앞으로 내미는 길이 (**상당히 늘림**)
    gait_cycle_time = 0.8 # 전체 보행 주기 시간 (초) - **줄여서 더 빠르게 걷도록**

    # --- 각 다리 액추에이터/조인트 인덱스 정의 ---
    LEG_JOINT_INDICES = {
        "rb": {"hip": 0, "knee": 1}, # Right Back
        "lb": {"hip": 2, "knee": 3}, # Left Back
        "rf": {"hip": 4, "knee": 5}, # Right Front
        "lf": {"hip": 6, "knee": 7}  # Left Front
    }

    # 9. 메인 시뮬레이션 및 렌더링 루프
    while not glfw.window_should_close(window):
        time_prev = data.time
        
        # --- GAIT CONTROL LOGIC (TROT) ---
        phase = (data.time % gait_cycle_time) / gait_cycle_time
        current_desired_qpos = np.copy(stand_qpos) # 매 주기마다 스탠드 자세에서 시작

        # RF (Right Front) & LB (Left Back) 동시 스윙 페이즈 (0.0 ~ 0.5)
        if 0.0 <= phase < 0.5:
            t_swing = (phase / 0.5) # 스윙 페이즈를 0-1로 스케일링
            
            lift_vertical_component = step_height * np.sin(np.pi * t_swing)
            # 발을 앞뒤로 스윙하는 수평 동작 (앞으로 더 많이 내딛도록 강조)
            # `step_length`가 이미 커졌으므로, 이 계수 조정으로 더 큰 보폭을 만듭니다.
            swing_horizontal_component = -step_length * np.cos(np.pi * t_swing)

            # RF 다리 제어
            current_desired_qpos[LEG_JOINT_INDICES["rf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rf"]["hip"]] + lift_vertical_component + swing_horizontal_component
            current_desired_qpos[LEG_JOINT_INDICES["rf"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["rf"]["knee"]] - lift_vertical_component * 0.5 

            # LB 다리 제어 (RF와 대칭적으로 움직임)
            current_desired_qpos[LEG_JOINT_INDICES["lb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lb"]["hip"]] + lift_vertical_component - swing_horizontal_component 
            current_desired_qpos[LEG_JOINT_INDICES["lb"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["lb"]["knee"]] - lift_vertical_component * 0.5

            # --- 스탠스(지지) 다리의 추진력 기여 ---
            # LF (Left Front) & RB (Right Back)는 현재 지지 다리입니다.
            # 이 다리들이 몸통을 앞으로 더 강하게 밀어내도록 힙 조인트의 목표 각도를 점진적으로 뒤로 이동시킵니다.
            stand_push_amplitude = 0.08 # 지지 다리가 뒤로 밀어내는 양 (라디안) - **증가**
            t_stand = t_swing 

            current_desired_qpos[LEG_JOINT_INDICES["lf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lf"]["hip"]] - stand_push_amplitude * t_stand
            current_desired_qpos[LEG_JOINT_INDICES["rb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rb"]["hip"]] - stand_push_amplitude * t_stand


        # LF (Left Front) & RB (Right Back) 동시 스윙 페이즈 (0.5 ~ 1.0)
        elif 0.5 <= phase < 1.0:
            t_swing = ((phase - 0.5) / 0.5) 
            
            lift_vertical_component = step_height * np.sin(np.pi * t_swing)
            swing_horizontal_component = -step_length * np.cos(np.pi * t_swing) 

            # LF 다리 제어
            current_desired_qpos[LEG_JOINT_INDICES["lf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lf"]["hip"]] + lift_vertical_component + swing_horizontal_component
            current_desired_qpos[LEG_JOINT_INDICES["lf"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["lf"]["knee"]] - lift_vertical_component * 0.5

            # RB 다리 제어 (LF와 대칭적으로 움직임)
            current_desired_qpos[LEG_JOINT_INDICES["rb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rb"]["hip"]] + lift_vertical_component - swing_horizontal_component
            current_desired_qpos[LEG_JOINT_INDICES["rb"]["knee"]] = stand_qpos[LEG_JOINT_INDICES["rb"]["knee"]] - lift_vertical_component * 0.5

            # --- 스탠스(지지) 다리의 추진력 기여 ---
            # RF (Right Front) & LB (Left Back)는 현재 지지 다리입니다.
            stand_push_amplitude = 0.08 # 지지 다리가 뒤로 밀어내는 양 (라디안) - **증가**
            t_stand = t_swing 

            current_desired_qpos[LEG_JOINT_INDICES["rf"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["rf"]["hip"]] - stand_push_amplitude * t_stand
            current_desired_qpos[LEG_JOINT_INDICES["lb"]["hip"]] = stand_qpos[LEG_JOINT_INDICES["lb"]["hip"]] - stand_push_amplitude * t_stand

        # --- PD CONTROL: compute torques for all actuators ---
        for i in range(model.nu): # 모든 액추에이터 반복
            joint_id = model.actuator_trnid[i, 0]
            
            qpos_current = data.qpos[model.jnt_qposadr[joint_id]] 
            qvel_current = data.qvel[model.jnt_dofadr[joint_id]] 
            
            torque = kp * (current_desired_qpos[i] - qpos_current) + kd * (0.0 - qvel_current)
            data.ctrl[i] = torque
            
            # # 디버깅을 위한 토크 출력 (필요시 주석 해제)
            # if i == LEG_JOINT_INDICES["rf"]["hip"]: 
            #     print(f"RF_Hip_Torque: {torque:.2f}")

        # --- Simulate ---
        while (data.time - time_prev) < (1.0/60.0):
            mj.mj_step(model, data)
        
        # # 디버깅을 위한 로봇 몸통 속도 출력 (필요시 주석 해제)
        # root_body_id = model.body_name2id("base_link") 
        # if root_body_id != -1 and model.body_dofnum[root_body_id] >= 3:
        #     root_vel_x = data.qvel[model.body_dofadr[root_body_id]]
        #     print(f"Root X velocity: {root_vel_x:.4f} m/s")

        # --- Render ---
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scn)
        mj.mjr_render(viewport, scn, con)

        glfw.swap_buffers(window)
        glfw.poll_events()

    # 10. GLFW 종료
    glfw.terminate()
    print("시뮬레이션 종료.")

if __name__ == "__main__":
    main()