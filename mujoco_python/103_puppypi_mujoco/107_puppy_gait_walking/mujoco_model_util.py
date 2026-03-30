import mujoco as mj
import numpy as np
from mujoco.glfw import glfw

class Slider:
    def __init__(self, x, y, value):
        self.slider_start_x = x
        self.slider_start_y = y
        self.dragging_slider = False
        self.slider_value = value
        self.slider_width = 200
        self.slider_height = 20

    def set_dragging_true(self, click_x, click_y):
        if (self.slider_start_x <= click_x <= self.slider_start_x + self.slider_width) and (self.slider_start_y <= click_y <= self.slider_start_y + self.slider_height):
                self.dragging_slider = True
    
    def set_dragging_false(self):
        self.dragging_slider = False
    
    def set_slide_value(self, click_x):
        new_value = (click_x - self.slider_start_x) / self.slider_width
        self.slider_value = np.clip(new_value, 0.0, 1.0)

    def draw_slide(self):
        mj.mjr_rectangle(
            mj.MjrRect(self.slider_start_x, self.slider_start_y, self.slider_width, self.slider_height),
            0.4, 0.4, 0.4, 1.0
        )
        knob_x = int(self.slider_start_x + self.slider_value * self.slider_width) - 5
        mj.mjr_rectangle(
            mj.MjrRect(knob_x, self.slider_start_y - 5, 10, self.slider_height + 10),
            0.8, 0.2, 0.2, 1.0
        )

def check_model_data(model, data):
    print("=== mj_data Overview ===")
    print(f"Number of qpos     : {model.nq}")
    print(f"Number of qvel     : {model.nv}")
    print(f"Number of actuators: {model.nu}")
    print(f"Number of sensors  : {model.nsensor}")
    print()

    # Generalized position and velocity
    print("--- Generalized Coordinates ---")
    print(f"qpos shape: {data.qpos.shape} -> {data.qpos}")
    print(f"qvel shape: {data.qvel.shape} -> {data.qvel}")
    print()

    # Actuator control input
    print("--- Actuator Inputs (data.ctrl) ---")
    print(f"ctrl shape: {data.ctrl.shape} -> {data.ctrl}")
    print()

    # Sensor data
    print("--- Sensor Data (data.sensordata) ---")
    print(f"sensordata shape: {data.sensordata.shape}")
    print(f"sensordata: {data.sensordata}")
    print()

    # Force applied to bodies (external forces, e.g. ElasticBand)
    print("--- External Forces (data.xfrc_applied) ---")
    print(f"xfrc_applied shape: {data.xfrc_applied.shape} ->{data.xfrc_applied}")
    print()

    # Joint forces (for diagnostics)
    print("--- Applied Joint Forces (data.qfrc_applied) ---")
    print(f"qfrc_applied shape: {data.qfrc_applied.shape} -> {data.qfrc_applied}")
    print()

    # Center of mass velocity
    print("--- Subtree COM Linear Velocity (data.subtree_linvel) ---")
    print(f"subtree_linvel shape: {data.subtree_linvel.shape}")
    print()

    # Optional: contact forces if using collision
    print("--- Contacts (data.ncon) ---")
    print(f"Number of active contacts: {data.ncon}")
