# check_model_data(model, data)` 

This is a utility that prints out a summary of key characteristics of a loaded MuJoCo model. Here's what each line means:

---

### 📌 Line-by-Line Breakdown:
print(f"Number of qpos     : {model.nq}")
```

* **`model.nq`**: The number of **position variables (`qpos`)** in the model.
* These describe the generalized coordinates of your robot (e.g., joint angles, base position and orientation).
* For example: A 3D free-floating body has 7 (`3 for position + 4 for orientation (quaternion)`).

```python
print(f"Number of qvel     : {model.nv}")
```

* **`model.nv`**: The number of **velocity variables (`qvel`)**.
* These are the time derivatives of `qpos`, describing linear and angular velocities.

```python
print(f"Number of actuators: {model.nu}")
```

* **`model.nu`**: The number of **actuators** in your model.
* This tells you how many control inputs (`data.ctrl`) you can apply (e.g., motor torques, position commands).

```python
print(f"Number of sensors  : {model.nsensor}")
```

* **`model.nsensor`**: The number of **sensors** defined in your model.
* These could be joint position sensors, force sensors, accelerometers, etc., defined in the MJCF.

---

### Result 

Number of qpos     : 15
Number of qvel     : 14
Number of actuators: 8
Number of sensors  : 26

✅ Number of qpos: 15

This means the model has 15 position variables (qpos), broken down as:

    7 for root body (freejoint):

        3 for position (x, y, z)

        4 for orientation (quaternion: w, x, y, z)

    8 for joint positions:

        lb_joint1, lb_joint2

        rb_joint1, rb_joint2

        rf_joint1, rf_joint2

        lf_joint1, lf_joint2

    Total: 7 (root) + 8 (joint angles) = 15

✅ Number of qvel: 14

This means the model has 14 velocity variables (qvel), broken down as:

    6 for root body velocity:

        3 for linear velocity

        3 for angular velocity

    8 for joint velocities (same 8 joints as above)

    Total: 6 (root) + 8 (joints) = 14

✅ Number of actuators: 8

This corresponds to the 8 <motor> actuators defined in the <actuator> section:

    2 actuators per leg × 4 legs:

        lb_joint1, lb_joint2

        rb_joint1, rb_joint2

        rf_joint1, rf_joint2

        lf_joint1, lf_joint2

Each joint has its own motor.
✅ Number of sensors: 26

Here’s how the 26 sensors are composed:

    For each of the 8 actuators, 3 sensors are defined:

        actuatorpos

        actuatorvel

        actuatorfrc

        8 actuators × 3 = 24

    Plus 2 additional sensors:

        framequat on site name="imu"

        gyro on site name="imu"

    Total: 24 (actuator sensors) + 2 (IMU sensors) = 26

🔎 Summary Table
Component	Count	Explanation
qpos	15	7 (free joint of root) + 8 (leg joints)
qvel	14	6 (root velocity) + 8 (leg joint velocities)
actuators	8	One motor per joint
sensors	26	3 per actuator (×8 = 24) + 2 IMU sensors (gyro + orientation)

Here is a clean and structured explanation of the **code** and **result** using your MuJoCo XML model:

---

### ✅ Code Explanation

```python
print("--- Generalized Coordinates ---")
print(f"qpos shape: {data.qpos.shape} -> {data.qpos}")
print(f"qvel shape: {data.qvel.shape} -> {data.qvel}")
```

This code prints the **shape and values** of two core simulation state vectors:

| Variable | Meaning                                      |
| -------- | -------------------------------------------- |
| `qpos`   | Generalized **position vector** of the model |
| `qvel`   | Generalized **velocity vector** of the model |

---

### 🧾 Result Breakdown (from your model)

```text
qpos shape: (15,) -> 
[0.         0.         0.02081127   # root position (x, y, z)
 1.         0.         0.         0.  # root orientation (quaternion w, x, y, z)
 0.         0.         0.         0.         0.         0.         0.        ]  # 8 joint angles
```

#### 🔹 qpos (15 values total)

* **First 7 values** → root body:

  * `0, 0, 0.02081127`: base position in world (z ≈ 2cm above ground)
  * `1, 0, 0, 0`: unit quaternion (no rotation)
* **Next 8 values** → joint positions:

  * `lb_joint1`, `lb_joint2`, `rb_joint1`, `rb_joint2`, `rf_joint1`, `rf_joint2`, `lf_joint1`, `lf_joint2`
  * All set to zero in this example

---

```text
qvel shape: (14,) -> 
[0. 0. 0. 0. 0. 0.     # root linear + angular velocity
 0. 0. 0. 0. 0. 0. 0. 0.]  # joint velocities (8 joints)
```

#### 🔹 qvel (14 values total)

* **First 6 values** → root body velocity:

  * Linear velocity: `vx, vy, vz`
  * Angular velocity: `wx, wy, wz`
  * All zero → robot is stationary
* **Next 8 values** → joint velocities:

  * Same joint order as `qpos`
  * All zero → joints are not moving

---

### 🔚 Summary Table

| Vector | Size | Content                                                 |
| ------ | ---- | ------------------------------------------------------- |
| `qpos` | 15   | 7 for root (position + orientation), 8 for joint angles |
| `qvel` | 14   | 6 for root velocity, 8 for joint velocities             |


Here is a structured explanation of your **code** and **output** using your MuJoCo XML model:

---

### ✅ Code Explanation

```python
print("--- Actuator Inputs (data.ctrl) ---")
print(f"ctrl shape: {data.ctrl.shape} -> {data.ctrl}")
```

This code prints the **shape** and **current values** of the **actuator control inputs** in the simulation.

| Variable    | Description                                                                                                                           |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `data.ctrl` | The control input vector sent to the robot's actuators (motors). These values are typically updated every timestep to move the robot. |

---

### 🧾 Result Breakdown

```text
ctrl shape: (8,) -> [0. 0. 0. 0. 0. 0. 0. 0.]
```

#### 🔹 `ctrl shape: (8,)`

* There are **8 actuators** in your robot model (as defined in `<actuator>` section):

  * `lb_joint1`, `lb_joint2`
  * `rb_joint1`, `rb_joint2`
  * `rf_joint1`, `rf_joint2`
  * `lf_joint1`, `lf_joint2`
* Each element in `data.ctrl` corresponds to one actuator.

#### 🔹 `data.ctrl` values: `[0. 0. 0. 0. 0. 0. 0. 0.]`

* All values are **zero**, meaning no torque or control command is being sent.
* If you update `data.ctrl[:] = [...]`, the actuators (motors) will respond based on the control mode and these values.

---

### 📌 Actuator Mapping

| Index | Actuator (Joint) |
| ----- | ---------------- |
| 0     | lb\_joint1       |
| 1     | lb\_joint2       |
| 2     | rb\_joint1       |
| 3     | rb\_joint2       |
| 4     | rf\_joint1       |
| 5     | rf\_joint2       |
| 6     | lf\_joint1       |
| 7     | lf\_joint2       |

---

### 🔚 Summary

| Field       | Value                                                   | Description                               |
| ----------- | ------------------------------------------------------- | ----------------------------------------- |
| `data.ctrl` | shape = (8,)                                            | 8 control inputs, one per joint motor     |
| values      | `[0. ... 0]`                                            | No torque or position command applied yet |
| usage       | Torque or position commands, depending on control model | 


                                         |

### Question: 
Regarding data.ctrl, My robot is 4 foots robot. Then data.ctrl values are position angle? 

Great question — let’s clarify what the values in `data.ctrl` mean for your 4-legged robot.

---

### ✅ Your Setup

Your XML shows this for actuators:

```xml
<actuator>
  <motor name="lb_joint1" joint="lb_joint1" ctrllimited="true" ctrlrange="-0.5 0.5" gear="1" />
  ...
</actuator>
```

This tells us:

* You're using `<motor>` actuators (not `<position>` or `<velocity>`).
* The motors apply **torque** to the joints based on `data.ctrl` values.
* `gear="1"` and `ctrlrange="-0.5 0.5"` → input values in `data.ctrl` are **interpreted as desired torques** in the range `-0.5` to `0.5` Nm (Newton-meters).

---

### ❗ So to answer:

> **Are `data.ctrl` values position angles?**

🔴 **No.**
The values in `data.ctrl` are **torque commands**, **not position angles**.

---

### 🧭 If You Want Position Control:

If you want the inputs in `data.ctrl` to behave like **desired joint positions**, you need to use **position actuators**, like:

```xml
<position name="lb_joint1_pos" joint="lb_joint1" kp="10" ctrlrange="-2 2" />
```

Or combine a `<motor>` actuator with a `joint` that has built-in **passive stiffness** (PD control effect).

---

### 📌 Summary

| Feature             | Your Model              |
| ------------------- | ----------------------- |
| Actuator type       | `<motor>`               |
| `data.ctrl` meaning | **Torque** (in Nm)      |
| Values represent    | Effort to rotate joints |
| Position control?   | ❌ Not by default        |

---

Here’s a structured explanation of your **code** and **output** for `data.sensordata`, using your MuJoCo XML file:

---

### ✅ Code Explanation

```python
print("--- Sensor Data (data.sensordata) ---")
print(f"sensordata shape: {data.sensordata.shape}")
print(f"sensordata: {data.sensordata}")
```

| Variable                | Description                                                                                            |
| ----------------------- | ------------------------------------------------------------------------------------------------------ |
| `data.sensordata`       | A flat 1D array containing all sensor readings, concatenated in order of their declaration in the XML. |
| `data.sensordata.shape` | Shows how many values are present in total.                                                            |

---

### 🧾 Result Breakdown

```text
sensordata shape: (31,)
sensordata: [0. 0. 0. ... 0.]
```

There are **31 sensor values** in total — each element in the array corresponds to a sensor reading. These are ordered exactly as defined in the `<sensor>` block of your XML.

---

### 🔎 Sensor List from XML

You defined:

* **8 actuators** × 3 sensors each:

  * `actuatorpos` → 1 value each
  * `actuatorvel` → 1 value each
  * `actuatorfrc` → 1 value each
    → **Total = 8 × 3 = 24**

* **1 orientation sensor** using `framequat`:

  * Outputs a **quaternion** (4 values)

* **1 gyro sensor** using `gyro`:

  * Outputs **angular velocity (3 values)**

> **Total: 24 (actuators) + 4 (quaternion) + 3 (gyro) = 31 values**

---

### 📌 Sensor Mapping Summary

| Index Range | Sensor Type      | Count | Description                   |
| ----------- | ---------------- | ----- | ----------------------------- |
| 0–23        | actuator sensors | 24    | pos, vel, frc for 8 actuators |
| 24–27       | framequat (IMU)  | 4     | Quaternion: w, x, y, z        |
| 28–30       | gyro (IMU)       | 3     | Angular velocity: wx, wy, wz  |

---

### 🧠 Notes

* All values are `0.0` here because the robot is stationary and no torque has been applied yet.
* To access a specific sensor (e.g., `rf_joint2` velocity), you must find its index based on declaration order.

---
### Regarding xfrc 

Here’s a structured explanation of your **code**, **result**, and the meaning of `xfrc_applied`, using your MuJoCo XML model:

---

### ✅ Code Explanation

```python
print("--- External Forces (data.xfrc_applied) ---")
print(f"xfrc_applied shape: {data.xfrc_applied.shape}")
```

| Variable                                              | Description                                                                           |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `data.xfrc_applied`                                   | A 2D array storing **external forces and torques** applied to each body in the model. |
| Each row (shape `[nbody, 6]`)                         | Corresponds to one body. Each row contains:                                           |
| `[fx, fy, fz, tx, ty, tz]`                            |                                                                                       |
| → **force** (N) + **torque** (Nm), in **world frame** |                                                                                       |

---

### 🧾 Result Breakdown

```text
xfrc_applied shape: (10, 6)
```

This tells us:

* The model has **10 bodies**.
* For each body, you can apply an external **6D wrench**:

  * First 3 values: **force vector** (x, y, z)
  * Last 3 values: **torque vector** (x, y, z)

---

### 🔍 What is `xfrc_applied` used for?

It allows you to **externally apply forces and torques** to specific bodies during simulation.

Example:

```python
data.xfrc_applied[2] = [10, 0, 0, 0, 0, 0]  # Apply 10N force in +X to body 2
```

This is useful for:

* Pulling the robot with a virtual rope
* Simulating wind or collisions
* Testing balance recovery

> These forces are cleared automatically each step unless you apply them manually again.

---

### 🔎 Why 10 bodies?

From your `<worldbody>` structure, MuJoCo internally counts **each `<body>` tag**, including nested links. Based on your model:

| Body Names (nested structure)                                  |
| -------------------------------------------------------------- |
| root (has freejoint)                                           |
| lb\_link1, lb\_link2                                           |
| rb\_link1, rb\_link2                                           |
| rf\_link1, rf\_link2                                           |
| lf\_link1, lf\_link2                                           |
| **Total = 1 (root) + 4 legs × 2 = 9**, plus root base → **10** |

---

### 🧠 Summary

| Field             | Value                                                  | Meaning                                                         |
| ----------------- | ------------------------------------------------------ | --------------------------------------------------------------- |
| `xfrc_applied`    | (10, 6)                                                | External `[fx, fy, fz, tx, ty, tz]` for each of 10 bodies       |
| Default Values    | All zeros                                              | No external forces applied yet                                  |
| Typical Use Cases | Apply wind, push robot, ElasticBand, simulate contacts |                                                                 |
| Coordinate Frame  | World                                                  | Forces/torques are expressed in **world frame**, not body frame |

---

Here's a clear and structured explanation of the **code**, **output**, and the meaning of `qfrc_applied` based on your MuJoCo model:

---

### ✅ Code Explanation

```python
print("--- Applied Joint Forces (data.qfrc_applied) ---")
print(f"qfrc_applied shape: {data.qfrc_applied.shape} -> {data.qfrc_applied}")
```

| Variable            | Description                                                                                                                        |
| ------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `data.qfrc_applied` | A 1D array of **forces or torques applied directly to DoFs** (degrees of freedom), in generalized coordinates (i.e., joint space). |
| Each element        | Corresponds to a single DoF from the `qvel` vector (same shape).                                                                   |

---

### 🧾 Result Breakdown

```text
qfrc_applied shape: (14,) -> 
[0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]
```

* The shape `(14,)` matches your model’s `qvel` size → 14 total DoFs:

  * **6 DoFs** for the root (free joint): `vx, vy, vz, wx, wy, wz`
  * **8 DoFs** for the 8 joints in the legs: `lb_joint1`, ..., `lf_joint2`

* All values are currently **0**, meaning no external joint-space forces have been applied yet.

---

### 🔍 What is `qfrc_applied`?

It’s a diagnostic or direct control field that lets you apply **generalized forces** (linear and angular) directly to each DoF.

Unlike `data.ctrl`, which sends commands to actuators (motors), `data.qfrc_applied` directly bypasses the actuator system and writes joint forces into the simulation:

```python
data.qfrc_applied[6] = 1.0  # Apply +1 Nm torque to the 1st joint after root
```

> These forces are **reset to zero every timestep** unless reapplied.

---

### 🧠 When to Use `qfrc_applied`

* For **diagnostics**, e.g., debugging inverse dynamics
* For **manual torque control** without actuators
* For **controller testing** in joint space
* For educational purposes (applying raw generalized forces)

---

### 🔁 Comparison: `qfrc_applied` vs `xfrc_applied`

| Feature        | `qfrc_applied`                        | `xfrc_applied`                             |
| -------------- | ------------------------------------- | ------------------------------------------ |
| Applies to     | DoFs (joints + root)                  | Bodies (links)                             |
| Shape          | `(nv,)` (here: 14)                    | `(nbody, 6)` (here: 10 × 6)                |
| Units          | Linear force or torque, joint-space   | Spatial wrench: `[fx, fy, fz, tx, ty, tz]` |
| Frame          | Generalized coordinates (joint-based) | World frame                                |
| Use case       | Joint-level torque input or debugging | External world-space forces (e.g. pulling) |
| Reset behavior | Auto-cleared every timestep           | Also auto-cleared unless reapplied         |

---

### 📌 Summary Table

| Field          | Value                                        | Meaning                             |
| -------------- | -------------------------------------------- | ----------------------------------- |
| `qfrc_applied` | shape (14,)                                  | Generalized joint forces (0 = none) |
| Related to     | Joint torques and root linear/angular forces |                                     |
| Typical state  | All 0 unless manually applied                |                                     |












