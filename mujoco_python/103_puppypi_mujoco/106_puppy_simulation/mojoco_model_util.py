import mujoco as mj

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