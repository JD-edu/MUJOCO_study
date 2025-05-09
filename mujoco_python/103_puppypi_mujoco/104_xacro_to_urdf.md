## ✅ **Option 1: Use `mjcf_from_urdf` (Official Tool)**

**MuJoCo now provides an official converter** in the Python API:

```bash
pip install mujoco
```

### 🔁 Convert URDF to MJCF using Python

```python
from mujoco import mjcf_from_urdf

model = mjcf_from_urdf("your_robot.urdf")
model.export_xml("your_robot_converted.xml")
```

> 📌 **Note**: This works best when your URDF is simple and doesn't use too many ROS plugins.

---

## ✅ **Option 2: Convert `.xacro → .urdf` First, Then Use MJCF Converter**

1. Convert Xacro to URDF:

   ```bash
   ros2 run xacro xacro your_robot.xacro -o your_robot.urdf
   ```

2. Then use the `mjcf_from_urdf()` method (as shown above).

---

## ✅ **Option 3: Use robosuite or Isaac Gym’s Parser (Advanced)**

If your robot is complex and uses ROS-specific features (e.g., `transmission`, `gazebo_ros_control`), you may need a **custom parser** or **manual cleanup** after conversion.

* `robosuite` and `dm_control` also provide custom URDF parsers to MJCF
* You can borrow logic from their repos if you need custom handling

---

## 🛠️ **Manual Conversion (if needed)**

Sometimes you **must clean up manually**, especially for:

| URDF/Xacro Element    | MJCF Equivalent              |     |         |
| --------------------- | ---------------------------- | --- | ------- |
| `<link>`              | `<body>`                     |     |         |
| `<joint>`             | `<joint>` (with `type`)      |     |         |
| `<inertial>`          | `mass`, `inertia` attributes |     |         |
| `<visual>`            | \`\<geom type="mesh          | box | ...">\` |
| `<collision>`         | Optional `<geom>`            |     |         |
| `<transmission>`      | 🔥 Not supported (skip!)     |     |         |
| `<material>` (colors) | `<material>` in `<asset>`    |     |         |

---

## 🧪 Example Workflow

```bash
# Step 1: Convert Xacro to URDF
ros2 run xacro xacro your_robot.xacro -o your_robot.urdf

# Step 2: Run Python script to convert URDF to MJCF
python -c "
from mujoco import mjcf_from_urdf
model = mjcf_from_urdf('your_robot.urdf')
model.export_xml('your_robot_converted.xml')
"
```
