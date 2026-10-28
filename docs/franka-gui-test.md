# M0-C Franka scene: GUI procedure

**Prerequisite:** Isaac Sim Full 6.2.0 has finished loading and the M0 smoke scene has shown a dynamic blue cube falling when **Play** is pressed. If only stage creation is verified, check physics first; do not assume the scene simulates correctly.

## 1. Retrieve files

In Windows **Command Prompt**, from the existing working repository:

~~~bat
cd %USERPROFILE%\Desktop\robot-flow-planner
git pull origin main
~~~

Do not install another Python interpreter or request elevated permissions solely for running these scripts.

## 2. Start a new stage and load the Franka

In Isaac Sim: **Stop** the timeline, **File > New** (discard/save the temporary smoke scene as desired), then **Window > Script Editor**. Paste:

~~~python
from pathlib import Path
path = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_franka_scene.py"
exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"))
~~~

A new subtree at \`/World/RFP_Franka\` should include \`Arm\`, \`Ground\`, \`Obstacle_Left\`, and \`Obstacle_Right\`. The Franka robot USD is a **referenced** asset; it may load asynchronously or fail if asset/Nucleus network access is blocked. Selecting the subtree in Stage then pressing **F** usually frames it.

The script does not modify shared locked/Nucleus assets or attempt to open an external Python executable.

### If assets cannot be resolved

The official Isaac Sim 6.2 GUI alternative is **Create > Robots > Franka Emika Panda Arm**. The referenced asset path in the script is the official 6.2 documented path:

\`/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda\`

Do not assume an empty black viewport proves the asset cannot load: inspect Stage and Console error messages.

## 3. Read the robot's joint state

Press **Play**, then paste into Script Editor:

~~~python
from pathlib import Path
path = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_franka_inspect.py"
exec(compile(path.read_text(encoding="utf-8"), str(path), "exec"))
~~~

The expected **total** is 9 degrees of freedom: **7 arm joints plus 2 finger/gripper joints**. This step only reads joint positions; it does not move the arm.

If joint inspection errors, capture the first traceback. The articulation may not have finished initializing, the reference may be unresolved, or physics may not be playing.

## 4. Exit criteria and limitations

- Isaac Sim GUI creates scene prims without Windows administrator privileges.
- Physics smoke test visibly moves a dynamic cube and interacts with ground.
- Franka robot is visibly present and joint state can be read while playing.
- Do not claim the boxes are collision-free, or that end-effector planning works. There is **no path planner or dynamic-obstacle tracker** in M0-C yet.
- The Franka asset and script examples are currently **unverified on this user's installation**; report errors and adapt before writing control commands.

Reference: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/introduction/quickstart_isaacsim_robot.html
