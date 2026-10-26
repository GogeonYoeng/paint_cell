# motion_planning — planner 경로 CSV → 8축(갠트리 2 + Doosan A0912 6) 모션 → RViz 검증

`painting_trajectory_planner` 가 만든 row 별 TCP 경로 CSV 를 받아 **4 cm 노드 NLP(관절 궤적) → row 사이 전이(stitch) →
충돌 검사 → 툴체인저 포함 통합 시퀀스 → RViz 재생** 까지 만드는 쪽이다. 저장소 하나(`vehicle_painting_robot`)를 받으면
planner 와 모션이 같은 디렉터리 규칙으로 돈다. 이 문서는 **다른 PC 에서 받아 바로 돌리는 사람** 기준으로 쓴다.

```
painting_trajectory_planner/path_planner/test/csv/<part>/trajectory_NNN.csv      ← planner 출력 (두 파이프라인의 접점)
        │  nlp4cm/env.sh prepare_input : TCP 시간 프로파일(t, path_speed, path_acceleration) 추가
        ▼
motion_planning/nlp4cm/input/<part>/trajectory_NNN.csv                           ← NLP 입력
        │  nlp4cm/run_all.sh  →  robot_control/.../trajectory_nlp.py (CasADi/Ipopt, 8축)
        ▼
motion_planning/nlp4cm/out_<env>/<part>/trajectory_NNN_nlp_{joint,command}.csv   ← row 별 관절 궤적 (4 cm 노드 / 1 ms 명령)
        │  run_all.sh 가 이어서 stitch_transitions (row 사이 들어올림→이동→내림 전이)
        ▼
motion_planning/nlp4cm/stitched_<env>/<part>/seq_NNNN_{rowNNN,moveNNN_NNN}_joint.csv
        │  viz/collision_all.sh (check_collisions)         nlp4cm/sequence_all.sh (build_sequence)
        ▼                                                   ▼
motion_planning/nlp4cm/collision_<env>/<part>.log          motion_planning/nlp4cm/sequence_<env>/<part>/   ← 홈→장착→도장→반납→홈
                                                             seq_NNNN_<segment>_<stage>_joint.csv, sequence.json, sequence_command.csv(1 ms)
        RViz:  viz/play.sh <part>   (row+전이)      /   ros2 launch paint_viz paint_viz.launch.py ... (통합 시퀀스)
```

`<env>` 는 갠트리 환경 이름이다. **현재 셀은 `x7y4z3`** (레일 7 x 4 m, 높이 3 m) 이고 기본값이다. 저장소에는 x7y4z3 의
NLP·stitch·시퀀스 산출물이 5 파츠 모두 들어 있어 **재계산 없이 바로 RViz 로 볼 수 있다.**

---

## 1. 디렉터리 규칙

| 위치 (저장소 기준) | 내용 | 만드는 것 | git |
|---|---|---|---|
| `painting_trajectory_planner/path_planner/test/data/<part>.ply` | 부품 점군 (스캔 좌표) | 스캔 | 포함 |
| `painting_trajectory_planner/path_planner/test/csv/<part>/trajectory_NNN.csv` | **planner 출력** row 별 TCP 경로 | planner (`test/scripts/test.py`, workstation) | 포함 |
| `motion_planning/nlp4cm/input/<part>/` | planner CSV + 시간 프로파일 | `env.sh prepare_input` (run_all.sh 가 자동 호출) | 포함 |
| `motion_planning/nlp4cm/out_<env>/<part>/` | NLP 관절 궤적 + `plan_metadata.json` | `run_all.sh`, `run_fender_flat.sh` | x7y4z3 포함 |
| `motion_planning/nlp4cm/stitched_<env>/<part>/` | row + 전이 이어붙인 궤적 (10 ms 전이, 1 ms row) | run_all.sh (자동), `viz/stitch_all.sh` (재-stitch) | x7y4z3 포함 |
| `motion_planning/nlp4cm/collision_<env>/<part>.log` | 충돌 검사 로그 | `viz/collision_all.sh` | x7y4z3 포함 |
| `motion_planning/nlp4cm/log_<env>/` | NLP/stitch 로그 | run_all.sh | x7y4z3 포함 |
| `motion_planning/nlp4cm/sequence_<env>/<part>/` | 통합 시퀀스 stage CSV + `sequence.json` | `sequence_all.sh` | x7y4z3 포함 (**`sequence_command.csv` 는 제외**, 1~2 분에 재생성) |
| `motion_planning/nlp4cm/viz/scene_<part>.yaml` | 부품 배치(legacy 월드 기준)·점군·경로 위치 | 사람 | 포함 |
| `motion_planning/nlp4cm/viz/env_<env>/` | 환경별 scene yaml | `env.sh scene_file()` 이 매번 생성 | 제외 |
| `motion_planning/paint_viz/` | ROS2 패키지 (RViz, stitch, 충돌, 시퀀스, URDF 생성) + URDF/STL | — | 포함 (메시 65 MB) |
| `motion_planning/dsr_description2/` | Doosan A0912 메시 패키지 (vendored) | — | 포함 |
| `robot_control/doosan_a0912_controller/test/scripts/trajectory_nlp.py` | 8축 NLP 솔버 (nlp4cm 가 호출) | — | 포함 |
| `~/paint_cell_ws/` (`$PAINT_CELL_WS`) | colcon 워크스페이스. `src/` 는 위 두 패키지의 **심볼릭 링크** | `setup_workspace.sh` | 저장소 밖 |

legacy 환경(`out/`, `stitched/`, ... 접미사 없음) 과 `baseline_*`, `zsweep` 은 비교용이라 git 에서 제외돼 있다 (`.gitignore`).
yaml/스크립트에는 `/home/<user>` 절대경로가 없다. 설정 파일의 `$REPO`, `$NLP4CM`, `$PAINT_CELL_WS` 는
`paint_viz.gantry_env.expand_project_path` 가 치환한다 (저장소 루트는 스크립트 위치에서 자동 계산).

### 1.1 planner 와 모션의 접점: CSV 위치와 스키마

**위치.** planner 는 `path_planner/test/csv/<part>/` 에 row 별 `trajectory_NNN.csv` 와 전체를 합친
`painting_trajectory.csv` 를 쓴다 (`test/scripts/test.py` 의 `output_dir`, workstation 백엔드의 `csv_output_dir`).
모션 쪽은 **같은 `<part>` 이름**으로 `nlp4cm/input/<part>` 를 만들고 이후 모든 산출물도 `<part>` 디렉터리를 쓴다.
파츠 이름 = 디렉터리 이름 = `--part` 인자다. 새 파츠는 planner 가 `test/csv/<새이름>/` 에 쓰기만 하면 모션 쪽에서 아래
"새 파츠 추가" 두 줄로 붙는다.

**스키마.** planner 출력과 NLP 입력의 차이는 열 3개다.

```
planner : row_index,point_index,frame_id,  s,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w,                              paint
NLP 입력: row_index,point_index,frame_id,t,s,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w,path_speed,path_acceleration,paint
```

`nlp4cm/add_time_profile.py` 가 아크길이 `s` 에 사다리꼴 TCP 속도 프로파일을 씌워 `t, path_speed, path_acceleration` 을
붙인다 (정지 출발·정지 도착, 등속 **0.42 m/s**, 가감속 1.2 m/s²; `env.sh` 의 `TIME_PROFILE_V/A`). 나머지 열은 그대로다.
좌표는 둘 다 **부품 스캔 좌표계**이고, 셀 안 배치(`part_xyz/part_rpy`)는 모션 쪽 `run_all.sh` 와 `viz/scene_<part>.yaml`
이 정한다 (planner 는 배치를 모른다).

`run_all.sh` 는 파츠마다 `prepare_input` 을 먼저 호출해 planner CSV 가 `input/<part>` 보다 새로우면 다시 만든다. 그래서
**planner 를 다시 돌린 뒤 `run_all.sh` 만 실행하면 된다.** planner 출력이 없는 PC 에서는 저장소에 든 `input/<part>` 를 그대로 쓴다.

**새 파츠 추가.**
1. `test/data/<part>.ply` 와 `test/csv/<part>/trajectory_*.csv` (planner).
2. `nlp4cm/run_all.sh` 의 `case` 에 배치 한 줄 (`run <part> "<x,y,z>" "<r,p,y>"`, legacy 월드 기준 값 — x7y4z3 는 z +0.781 등 환경 오프셋을 `place_pose` 가 자동 적용) 과 `nlp4cm/viz/scene_<part>.yaml` (기존 파일 복사 후 이름·배치만 수정).

---

## 2. 환경 설정 (다른 PC, 최초 1회)

전제: Ubuntu 22.04, **ROS2 Humble** 이 `/opt/ros/humble` 에 설치 (`ros-humble-desktop` 수준: rviz2, robot_state_publisher,
joint_state_publisher_gui, ament_cmake), conda(anaconda/miniconda/miniforge). Python 패키지(pinocchio, casadi 등)는 conda 로 받는다.

```bash
git clone <this repo> ~/paint_cell/vehicle_painting_robot
bash ~/paint_cell/vehicle_painting_robot/motion_planning/setup_workspace.sh
#  = conda env create -f environment.yml  (환경 이름 paintcell, 수 분)
#  + ~/paint_cell_ws/src/{paint_viz,dsr_description2} 심볼릭 링크
#  + conda 안의 colcon 으로 빌드  + 검증 출력
```

이후 **매 셸마다**:

```bash
source ~/paint_cell/vehicle_painting_robot/motion_planning/activate_env.sh
```

(nlp4cm 스크립트들은 이걸 스스로 source 하므로 `./run_all.sh` 처럼 바로 실행해도 된다.)

하이브리드 환경 규칙 — 어기면 `import rclpy` 또는 `import pinocchio` 가 깨진다:
- 활성화 순서는 **conda activate → ROS setup.bash → 워크스페이스 setup.bash**. `activate_env.sh` 가 이 순서를 보장한다.
- conda 환경은 **Python 3.10** (시스템 rclpy 의 ABI), **numpy < 2** (ROS2 Humble 메시지 변환).
- `colcon build` 는 **conda 환경 안의 colcon** (`$CONDA_PREFIX/bin/colcon`) 으로. 시스템 `/usr/bin/colcon` 으로 빌드하면 노드가
  시스템 python 으로 설치되어 `ros2 run` 시 pinocchio 를 못 찾는다. `setup_workspace.sh` 가 이렇게 빌드한다.
- 코드를 고친 뒤에는 `cd $PAINT_CELL_WS && colcon build --packages-select paint_viz` (설치본은 복사본이다).

환경변수 (전부 선택):

| 변수 | 기본 | 뜻 |
|---|---|---|
| `PAINT_CELL_WS` | `~/paint_cell_ws` | colcon 워크스페이스 |
| `GANTRY_ENV` | `x7y4z3` | 갠트리 환경 (`legacy`, `spec_c`, `x7y4z3`; 정의 `paint_viz/config/gantry/<name>.yaml`) |
| `CONDA_ROOT` | 자동 탐색 | conda 설치 루트 |
| `PAINT_CELL_CONDA_ENV` | `paintcell` | conda 환경 이름 |
| `ROS_SETUP` | `/opt/ros/humble/setup.bash` | ROS2 setup |
| `PARTS` | 5 파츠 | run_all/stitch_all/collision_all/sequence_all 이 처리할 파츠 목록 |

planner 자체(`path_planner`, open3d/plotly)는 별도 python 환경을 쓴다 — `painting_trajectory_planner/README.md` 참고.
모션 파이프라인은 planner 를 실행하지 않고 그 **출력 CSV 만** 읽으므로 두 환경을 섞을 필요가 없다.

---

## 3. 실행

### 3.1 저장소에 든 결과를 바로 보기 (RViz)

```bash
cd ~/paint_cell/vehicle_painting_robot/motion_planning/nlp4cm
./viz/play.sh ES300h_hood 2.0            # row + 전이 재생 (stitched_x7y4z3). 두 번째 인자 = 배속
./viz/play.sh ES300h_front_bumper 2.0    # ES300h_trunk | NX350h_right_fender | NX350h_right_fender_flat
```

툴체인저 포함 통합 시퀀스 (홈 → 슬롯 1 장착 → 도장 → 반납 → 홈):

```bash
source ../activate_env.sh
PART=ES300h_hood
ros2 launch paint_viz paint_viz.launch.py gantry_env:=x7y4z3 \
    joint_csv_dir:=$NLP4CM/sequence_x7y4z3/$PART joint_csv_pattern:="seq_*_joint.csv" \
    scene:=$NLP4CM/viz/env_x7y4z3/scene_$PART.yaml speed_scale:=2.0
```

(`viz/env_x7y4z3/scene_*.yaml` 은 `play.sh` 나 `run_all.sh` 를 한 번 돌리면 생긴다. 바로 쓰려면
`source nlp4cm/env.sh; scene_file ES300h_hood` 로 만들 수 있다.)

갠트리·로봇 형상만: `ros2 launch paint_viz cell_view.launch.py gantry_env:=x7y4z3 gui:=true`

### 3.2 전체 재생성 (planner 를 다시 돌렸을 때)

```bash
cd motion_planning/nlp4cm
./run_all.sh                 # 4 파츠: prepare_input → NLP → stitch   (파츠당 수 분, 전체 15~30 분)
./run_fender_flat.sh         # 눕힌 펜더 (NX350h_right_fender 의 CSV 를 Rx+90 배치로)
./viz/collision_all.sh       # 충돌 검사 (파츠당 ~15 분, 생략 가능 — 시퀀스 빌더가 stage 별 검사를 따로 한다)
./sequence_all.sh            # 통합 시퀀스 + sequence_command.csv
PARTS="ES300h_hood" ./run_all.sh     # 일부만
```

로그는 `log_x7y4z3/<part>.log`, `<part>_stitch.log`, `collision_x7y4z3/<part>.log`, `sequence_x7y4z3/<part>.log`.

### 3.3 현재 결과 (x7y4z3, 갠트리 0.5 m/s · 1.6 m/s², 2026-10-26)

| 파츠 | 수렴 row | 도장+전이 | 전이 | 통합 시퀀스 총 | 충돌 | 비고 |
|---|---|---|---|---|---|---|
| ES300h_hood | 32/32 | 135 s | 23 s | 340 s | 없음 | |
| ES300h_trunk | 16/16 | 99 s | 33 s | 309 s | 없음 | row 가 Y 방향이라 gantry_y 가 0.5 m/s 한계에 자주 닿음 (부품 Y 배치 재검토 대상) |
| ES300h_front_bumper | 13/14 | 111 s | 20 s | 313 s | 없음 | 마지막 row Infeasible, 팔 특이점(σ 0.01)·관절 한계 overshoot ≤0.12° — planner 배치/경로 쪽 결정 대기 |
| NX350h_right_fender (세움) | 18/19 | 76 s | 43 s | 300 s | 없음 | 마지막 row Infeasible, 특이점 근접 |
| NX350h_right_fender_flat (눕힘, Rx+90) | 19/19 | 66 s | 32 s | 275 s | 없음 | 세운 배치보다 모든 지표가 낫다 |

---

## 4. 구성요소

- **`paint_viz/`** (ROS2 ament_python): `stitch_transitions`(전이), `check_collisions`, `build_sequence`(툴체인저 통합),
  `gantry_env`(환경 yaml 조회), `make_gantry_urdf`(URDF 생성), `dock_path`, RViz launch 3종. 상세는 `paint_viz/README.md`.
  URDF 메시는 `package://paint_viz/urdf/*.stl`, `package://dsr_description2/meshes/...` 로 참조한다.
- **`paint_viz/config/gantry/<env>.yaml`** — 갠트리 치수·원점·툴체인저·팔 장착·부품 배치 오프셋·**갠트리 모터 한계
  (`motor_limits`: 0.5 m/s, 1.6 m/s²)·NLP 가중치(`nlp`)** 를 한 곳에 둔다. trajectory_nlp / stitch / sequence 가 전부 여기서 읽는다.
- **`paint_viz/config/sequence/paint_with_tool.yaml`** — 통합 시퀀스 segment 순서·속도.
- **`nlp4cm/`** — 배치 스크립트. `env.sh`(경로·환경 활성화·prepare_input·scene_file), `run_all.sh`, `run_fender_flat.sh`,
  `sequence_all.sh`, `viz/{play,stitch_all,collision_all}.sh`, `add_time_profile.py`, `analyze.py`(요약 JSON).
- **`robot_control/doosan_a0912_controller/test/scripts/trajectory_nlp.py`** — 8축 NLP (CasADi + Ipopt). 4 cm 노드에서
  위치 ±2 mm·자세 1° 안, 관절/갠트리 한계, 팔 특이점 회피, 툴 간섭, 갠트리 부드러움 항을 풀고 1 ms 명령(TCP 등속)을 만든다.
  `--gantry-env` 하나로 URDF·한계·가중치가 정해진다.
- **`environment.yml` / `activate_env.sh` / `setup_workspace.sh`** — 2절.

## 5. 알려진 상태

- **전이는 들어올림/이동/내림 모서리에서 잠깐 멈춘다** (구간별 rest-to-rest, 2026-10-20). 한 번에 시간 배분하면 모서리 가속도
  스파이크 한 점이 전이 전체를 기어가게 했기 때문이다 (범퍼 1.4 m 전이 40 s → 5.6 s).
- 1 ms 명령의 갠트리 가속도에 2~5 m/s² 스파이크가 남아 있다 (명령 IK 최소노름 + 10 ms 전이의 1 ms 보간). 평균 거동은
  한계 안이고, 개선 후보는 명령 IK 가중 최소노름과 전이 경로의 cubic 보간이다.
- 범퍼·세운 펜더의 마지막 row Infeasible 과 트렁크 gantry_y 포화는 **부품 배치/경로 설계** 문제라 planner 쪽과 같이 봐야 한다.

## 6. git 운용

- 추적: 코드·설정·URDF/메시·planner CSV·`input/`·x7y4z3 산출물(`out_/stitched_/collision_/log_/sequence_x7y4z3`, 단 `sequence_command.csv` 제외).
- 제외(`.gitignore`): legacy 산출물, `baseline_*`, `zsweep`, `viz/env_*`, `sequence_command.csv`, `.bak*`, 워크스페이스 `build/ install/`.
- 산출물을 갱신해 커밋할 때는 `run_all.sh → sequence_all.sh` 를 돌린 뒤 `out_/stitched_/sequence_x7y4z3` 를 같은 커밋에 넣는다
  (NLP 는 결정적이라 같은 입력·설정이면 바이트 동일한 결과가 난다 — 다른 PC 에서 재현 검증에 쓸 수 있다).
- 큰 파일: 최대 36 MB(`gantry_base.stl`). GitHub 100 MB 제한 안이다. `sequence_command.csv` 는 100~200 MB 라 제외했다.
