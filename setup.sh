#!/usr/bin/env bash
# One-time setup: dependencies, Python venv and model weights.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$PKG_DIR")"
WS_DIR="$(dirname "$SRC_DIR")"
VENV="$PKG_DIR/.venv"
PY="$VENV/bin/python3"

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "ROS 2 Jazzy is not sourced. Run: source /opt/ros/jazzy/setup.bash" >&2
    exit 1
fi

echo "==> Cloning missing source dependencies into $SRC_DIR"
clone() {  # clone <dir> <url> [branch]
    if [[ -d "$SRC_DIR/$1/.git" ]]; then
        echo "    $1 already exists, skipping"
    else
        rm -rf "$SRC_DIR/$1"
        git clone ${3:+-b "$3"} "$2" "$SRC_DIR/$1"
    fi
}
clone actor_pose_plugin https://github.com/ahmetraufoktay/actor_pose_plugin.git
clone 2d_lidar_person_detection https://github.com/TeamSOBITS/2d_lidar_person_detection.git jazzy-devel

echo "==> Installing ROS dependencies with rosdep"
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$SRC_DIR" --ignore-src --rosdistro jazzy -y --skip-keys "ament_python"

echo "==> Creating Python venv at $VENV"
if [[ ! -x "$PY" ]]; then
    python3 -m venv --system-site-packages "$VENV"
fi
"$PY" -m pip install --upgrade pip
"$PY" -m pip install -r "$PKG_DIR/requirements.txt"

DR_WEIGHTS="$SRC_DIR/2d_lidar_person_detection/dr_spaam_ros/weights"
mkdir -p "$DR_WEIGHTS"
if compgen -G "$DR_WEIGHTS/*.pth" >/dev/null; then
    echo "==> DR-SPAAM weights already present"
else
    echo "==> Downloading DR-SPAAM weights"
    "$PY" -m gdown https://drive.google.com/drive/folders/1OI99VfUBkmRSijgmMYYku9Pc_nS3v8sj \
        -O "$DR_WEIGHTS" --folder
    find "$DR_WEIGHTS" -mindepth 2 -type f -name "*.pth" -exec mv -t "$DR_WEIGHTS" {} + 2>/dev/null || true
    if ! compgen -G "$DR_WEIGHTS/*.pth" >/dev/null; then
        echo "ERROR: no .pth files found in $DR_WEIGHTS after download." >&2
        echo "       Google Drive may be rate-limiting; wait a few minutes and re-run ./setup.sh" >&2
        exit 1
    fi
fi

cat <<EOF

Setup finished. Next:

  source "$VENV/bin/activate"
  cd "$WS_DIR"
  colcon build --symlink-install
  source install/setup.bash
  ros2 launch social_nav simulation.launch.py
EOF
