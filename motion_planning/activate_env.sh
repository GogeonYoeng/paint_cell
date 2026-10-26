#!/usr/bin/env bash
# 도장셀 하이브리드 환경 활성화 — 반드시 `source motion_planning/activate_env.sh` 로 실행할 것.
#
# ★ 활성화 순서 절대 변경 금지 ★
#   (1) conda activate paintcell   →   (2) source /opt/ros/humble/setup.bash   →   (3) 워크스페이스 install/setup.bash
# 이유: ROS setup.bash 가 PYTHONPATH 에 시스템 ROS 경로를 append 한다. conda 를 나중에 켜면 conda 가
#       PATH/PYTHONPATH 앞에 끼어들면서 ROS 경로 해석이 꼬여 rclpy import 가 깨진다.
#
# 조정 가능한 환경변수 (전부 선택):
#   CONDA_ROOT           conda 설치 루트 (기본: ~/anaconda3, ~/miniconda3, ~/miniforge3, /opt/conda 순서로 탐색)
#   PAINT_CELL_CONDA_ENV conda 환경 이름 (기본 paintcell — environment.yml 의 name)
#   ROS_SETUP            ROS2 setup.bash (기본 /opt/ros/humble/setup.bash)
#   PAINT_CELL_WS        colcon 워크스페이스 (기본 ~/paint_cell_ws; src/ 에 paint_viz, dsr_description2 심볼릭 링크)
# 설정 후 내보내는 값: PAINT_CELL_REPO (저장소 루트), PAINT_CELL_WS, NLP4CM

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "[activate_env] 이 스크립트는 'source activate_env.sh' 로 실행하세요." >&2
    exit 1
fi

_mp_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PAINT_CELL_REPO="${PAINT_CELL_REPO:-$(cd "$_mp_dir/.." && pwd)}"
export PAINT_CELL_WS="${PAINT_CELL_WS:-$HOME/paint_cell_ws}"
export NLP4CM="${NLP4CM:-$_mp_dir/nlp4cm}"

# (0) conda 셸 훅 — conda 가 "셸 함수" 로 정의돼 있지 않으면 설치 루트에서 훅을 로드한다.
if [[ "$(type -t conda 2>/dev/null)" != "function" ]]; then
    _conda_root="${CONDA_ROOT:-}"
    if [[ -z "$_conda_root" ]]; then
        for _c in "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/miniforge3" "$HOME/mambaforge" /opt/conda /opt/anaconda3 /opt/miniconda3; do
            [[ -x "$_c/bin/conda" ]] && _conda_root="$_c" && break
        done
    fi
    if [[ -n "$_conda_root" ]]; then
        eval "$("$_conda_root/bin/conda" shell.bash hook 2>/dev/null)" || source "$_conda_root/etc/profile.d/conda.sh"
    elif command -v conda >/dev/null 2>&1; then
        eval "$(conda shell.bash hook)"
    else
        echo "[activate_env] conda 를 찾지 못했다. CONDA_ROOT=<conda 설치 루트> 를 설정할 것" >&2
        return 1
    fi
    unset _conda_root _c
fi

# (1) conda 환경 — 먼저!
conda activate "${PAINT_CELL_CONDA_ENV:-paintcell}" || {
    echo "[activate_env] conda activate ${PAINT_CELL_CONDA_ENV:-paintcell} 실패 — 'conda env create -f $_mp_dir/environment.yml' 먼저" >&2
    return 1
}

# (2) 시스템 ROS2 Humble — conda 다음에!
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# (3) 워크스페이스 오버레이 (빌드되어 있으면)
if [[ -f "$PAINT_CELL_WS/install/setup.bash" ]]; then
    source "$PAINT_CELL_WS/install/setup.bash"
fi

echo "[activate_env] $(basename "$CONDA_PREFIX")(conda) + ROS2 ${ROS_DISTRO:-?} 활성화 (python: $(which python), repo: $PAINT_CELL_REPO, ws: $PAINT_CELL_WS)"
unset _mp_dir
