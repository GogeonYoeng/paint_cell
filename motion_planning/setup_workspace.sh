#!/usr/bin/env bash
# 다른 PC 에서 최초 1회 설정: conda 환경 생성 → colcon 워크스페이스에 패키지 심볼릭 링크 → conda colcon 으로 빌드 → 검증.
#   bash motion_planning/setup_workspace.sh            # 전부
#   bash motion_planning/setup_workspace.sh --no-conda # conda 환경이 이미 있으면
# 전제: ROS2 Humble 이 /opt/ros/humble 에 설치돼 있고 (ros-humble-desktop + robot_state_publisher, rviz2),
#       conda(anaconda/miniconda/miniforge) 가 있다. 워크스페이스 위치는 PAINT_CELL_WS (기본 ~/paint_cell_ws).
set -euo pipefail
MP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${PAINT_CELL_WS:-$HOME/paint_cell_ws}"
ENV_NAME="${PAINT_CELL_CONDA_ENV:-paintcell}"

if [[ "${1:-}" != "--no-conda" ]]; then
    if [[ "$(type -t conda 2>/dev/null)" != "function" ]]; then
        for c in "${CONDA_ROOT:-}" "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/miniforge3" /opt/conda; do
            [[ -n "$c" && -x "$c/bin/conda" ]] && eval "$("$c/bin/conda" shell.bash hook)" && break
        done
    fi
    if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        echo "[setup] conda 환경 $ENV_NAME 이미 있음 (갱신: conda env update -n $ENV_NAME -f $MP/environment.yml)"
    else
        echo "[setup] conda 환경 $ENV_NAME 생성 (environment.yml)"
        conda env create -n "$ENV_NAME" -f "$MP/environment.yml"
    fi
fi

echo "[setup] 워크스페이스 $WS/src 에 패키지 링크"
mkdir -p "$WS/src"
for pkg in paint_viz dsr_description2; do
    ln -sfn "$MP/$pkg" "$WS/src/$pkg"
done

# 활성화 (conda → ROS 순서) 후 conda 안의 colcon 으로 빌드. 시스템 /usr/bin/colcon 을 쓰면 노드가 시스템 python 으로 설치돼
# conda 의 pinocchio/casadi 를 못 찾는다.
set +u; source "$MP/activate_env.sh"; set -u
COLCON="$CONDA_PREFIX/bin/colcon"
if [[ ! -x "$COLCON" ]]; then
    echo "[setup] conda 환경에 colcon 이 없다 — pip install colcon-common-extensions" >&2; exit 1
fi
echo "[setup] colcon build ($COLCON)"
(cd "$WS" && "$COLCON" build --packages-select dsr_description2 paint_viz)
set +u   # ROS setup.bash 들은 미정의 변수(COLCON_TRACE 등)를 참조한다
source "$WS/install/setup.bash"

echo "[setup] 검증"
python -c "import pinocchio, casadi, scipy, pandas, yaml, rclpy; print('  python 모듈 OK:', pinocchio.__version__, casadi.__version__)"
ros2 run paint_viz gantry_env --name x7y4z3 --get urdf.mesh | sed 's/^/  URDF: /'
test -f "$(ros2 pkg prefix paint_viz)/share/paint_viz/urdf/tool_assy.stl" && echo "  메시 설치 OK (package://paint_viz/urdf)"
echo "[setup] 완료. 매 셸마다: source $MP/activate_env.sh"
