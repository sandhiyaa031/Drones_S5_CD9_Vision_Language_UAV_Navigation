#!/usr/bin/env bash
# Isolated Python environment for the MuJoCo simulation (no sudo, no
# system-wide change). The venv lives in simulation/mujoco/.venv and only
# `mujoco` is installed into it; numpy, OpenCV, SciPy and pytest come from the
# system Python (--system-site-packages), which is also what the rescue
# autonomy modules use. ROS 2 is NOT needed and need not be running.
# Usage: simulation/mujoco/setup_env.sh        (then: source .venv/bin/activate)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
python3 -m venv --system-site-packages "$HERE/.venv"
"$HERE/.venv/bin/pip" install --quiet "mujoco>=3.2,<4"
"$HERE/.venv/bin/python" - <<'PY'
import mujoco, numpy, cv2, scipy
print('mujoco', mujoco.__version__, '| numpy', numpy.__version__,
      '| opencv', cv2.__version__, '| scipy', scipy.__version__)
PY
