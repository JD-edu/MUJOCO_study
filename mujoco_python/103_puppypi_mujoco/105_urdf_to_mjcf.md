# ✅ URDF → MJCF Conversion Summary

---

## 🔧 1. Install `urdf2mjcf` Tool

First, install the tool with `pip`:

```bash
pip install urdf2mjcf
```

> ⚠️ If you see a warning like:
>
> ```
> WARNING: The script urdf2mjcf is installed in '/home/youruser/.local/bin' which is not on PATH.
> ```
>
> Then add this to your `.bashrc` or `.zshrc`:
>
> ```bash
> export PATH="$HOME/.local/bin:$PATH"
> source ~/.bashrc
> ```

---

## 📦 2. Prepare the URDF Model

Make sure your URDF is **fully self-contained**, meaning:

* All mesh paths are relative or absolute and valid.
* If using `.xacro`, convert it first:

  ```bash
  ros2 run xacro xacro your_robot.urdf.xacro > your_robot.urdf
  ```

---

## ▶️ 3. Convert URDF to MJCF

Run the conversion:

```bash
urdf2mjcf your_robot.urdf
```

This creates an MJCF `.xml` file and copies meshes to a folder like:

```
output/
├── your_robot.xml
├── meshes/
│   └── *.STL (or .obj)
```

> 🔁 If you get errors, check for unsupported URDF tags (like `<transmission>`) or missing mesh files.

---

## 🧪 4. Validate MJCF File

Test whether the MJCF loads properly:

```bash
python -m mujoco.viewer your_robot.xml
```

If you get a segmentation fault or GLFW error on Wayland:

```bash
export MUJOCO_GL=egl   # or try: osmesa or glfw
```

---

## 🧘 5. Fix Post-Conversion Issues

### ✅ Fix Crawling or Falling Over:

* MJCF models often **crawl or twitch** if actuators aren't controlled.
* In Python, explicitly set:

  ```python
  data.ctrl[:] = 0
  ```

### ✅ Fix STL Mesh Issues:

* MuJoCo prefers `.obj` format. If the viewer crashes:

  * Convert STL to OBJ (e.g., using Blender or `assimp`).
  * Update `<mesh file="...">` paths in the MJCF `<asset>` section.

---

## 🧱 6. Add Controllers or Visual Debug Tools

You can now:

* Use `mujoco.viewer` to view passively
* Write your own control loop with `mj_step()` in Python
* Attach sensors, sites, or plots for debugging

---

## 🧰 Optional: MJCF Refinement Tasks

| Task                      | Description                                  |
| ------------------------- | -------------------------------------------- |
| Add `<inertial>` manually | If urdf2mjcf skips them or produces `mass=0` |
| Refactor joint hierarchy  | Clean up nested bodies for readability       |
| Tune damping/friction     | Add `<default>` settings for smooth motion   |
| Define materials          | Use `<material>` and `rgba` for coloring     |

---

## 📚 Resources

* [MuJoCo Documentation](https://mujoco.readthedocs.io/)
* [urdf2mjcf GitHub](https://github.com/deepmind/mujoco/tree/main/tools/urdf)
* [MuJoCo Python Viewer](https://github.com/deepmind/mujoco)

