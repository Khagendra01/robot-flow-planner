# Isaac Sim GUI smoke test (non-admin workflow)

The Isaac Sim 6.2 **GUI opening is not equivalent to confirming physics and scene scripting work**. If the screenshot still shows `96% hydracreate` and `0.00 FPS`, finish diagnosing initialization before running scripts.

## What the red locks mean

The red-padlock folders under `omniverse://` are usually read-only shared/Nucleus assets. They cannot be edited in place; that is distinct from Windows administrator permissions. We can create and save our *own* USD stages in a writable folder such as the user's Documents or Desktop. A blank stage and primitive boxes do not require connecting to a Nucleus server.

## First: check the app, without launching another executable

1. After the application has fully loaded and the viewport responds, use **File > New**.
2. Open **Window > Script Editor** (NVIDIA's official GUI scripting workflow).
3. Update your local checkout with `git pull origin main` if network works.
4. Paste this **one-line command** into the Isaac Sim Script Editor and press Run:

~~~python
from pathlib import Path
script = Path.home() / "Desktop" / "robot-flow-planner" / "scripts" / "isaac_sim_gui_smoke.py"
exec(compile(script.read_text(encoding="utf-8"), str(script), "exec"))
~~~

This runs in Isaac Sim's **embedded Python process**; it does *not* invoke `python.exe` through Windows, does not write to Nucleus, and does not download Franka assets. It creates four boxes and a physics scene at `/World/RFP_Smoke`, using the built-in USD/PhysX APIs.

5. Find `/World/RFP_Smoke` in the Stage pane, select it and press **F** to frame objects. The black viewport alone is not enough to declare failure.
6. Press **Play**: the blue `Drop_Test` cube should fall. Orange obstacles and gray floor remain fixed. Press **Stop** after observing.
7. Save the USD scene under a **writable local path** in your user Documents/Desktop, not a red-locked Content Browser location.

If the Script Editor is unavailable or shows an error, read the Console and note the **exact** error text. Do not attempt to defeat security controls.

## Distinguish Windows blockers

- **UAC / admin password for Python/installer**: do not run unknown binaries elevated; use the already installed app's Script Editor for a minimal test, or contact the device administrator if the component genuinely requires elevated installation.
- **Firewall/network prompt**: network permissions may affect asset downloads, Nucleus, and extension fetching, not necessarily the local USD/PhysX smoke test. Request approved access instead of disabling protective rules.
- **Read-only red locks**: shared asset write permissions, not an invitation to alter them. Save projects locally.
- **Black viewport, initialization not finished**: inspect Console errors and NVIDIA's logs before changing GPU/toolkit versions.

## Logs and fallback

NVIDIA documents Windows application logs under:

`%USERPROFILE%\.nvidia-omniverse\logs\Kit\Isaac-Sim`

Open this folder in Explorer:

~~~bat
explorer "%USERPROFILE%\.nvidia-omniverse\logs\Kit\Isaac-Sim"
~~~

If you see `Access denied`, `Failed to start process`, `hydracreate`, `renderer`, or `omni.kit` errors, copy a few surrounding lines from the newest log. Do not share passwords, tokens or private hostnames.

Official references:
- https://docs.isaacsim.omniverse.nvidia.com/6.2.0/python_scripting/environment_setup.html
- https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/troubleshooting.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_faq.html

**Scope:** This is stage/physics smoke testing only; it does not spawn a Franka robot, provide continuous collision checking, or validate real robot control.
