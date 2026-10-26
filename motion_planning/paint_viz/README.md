# paint_viz

> 위치: `vehicle_painting_robot/motion_planning/paint_viz` (colcon 워크스페이스 `~/paint_cell_ws/src/paint_viz` 는 여기로의 심볼릭 링크).
> 환경 설정·디렉터리 규칙·planner 와의 접점은 `motion_planning/README.md` 를 먼저 볼 것. 아래 예시의 `$NLP4CM` 은
> `motion_planning/nlp4cm` (activate_env.sh 가 내보냄), `$PAINT_CELL_WS` 는 colcon 워크스페이스(기본 `~/paint_cell_ws`)다.

`vehicle_painting_robot` 의 도장 부품 점군, planner가 만든 TCP 경로, 그리고 로봇이
그 경로를 따라 움직이는 모습을 RViz2로 띄우는 ROS2(Humble) 패키지.

## 빠른 실행

```bash
source ~/paint_cell/activate_env.sh        # conda paintcell + ROS2 Humble
cd $PAINT_CELL_WS && colcon build --symlink-install
source install/setup.bash

# 1) 8축 셀 URDF 생성 (최초 1회)
#    기본값은 gantry.stp 실측: 스트로크 4.392 x 2.142 m, 장착 높이 2.066 m 천장 장착.
#    다른 갠트리 환경은 `--env <name>` (아래 "갠트리 환경 선택" 절).
ros2 run paint_viz make_gantry_urdf \
  --arm-urdf ~/paint_cell/vehicle_painting_robot/robot_control/doosan_a0912_controller/urdf/a0912.white.urdf \
  --output $PAINT_CELL_REPO/motion_planning/paint_viz/urdf/paint_cell_8axis.urdf

# 2) 경로 -> 관절 궤적 (부품/배치는 config/scene.yaml)
ros2 run paint_viz solve_viz_ik \
  --scene $PAINT_CELL_REPO/motion_planning/paint_viz/config/scene.yaml \
  --urdf  $PAINT_CELL_REPO/motion_planning/paint_viz/urdf/paint_cell_8axis.urdf \
  --output ~/paint_cell_ws/viz_ik --stride 4

# 3) RViz2
ros2 launch paint_viz paint_viz.launch.py speed_scale:=8.0
```

`colcon` 은 반드시 conda 환경 것이어야 한다 (`which colcon` →
`~/anaconda3/envs/paintcell/bin/colcon`). 시스템 colcon으로 빌드하면 노드가
시스템 Python으로 실행돼 pinocchio를 못 찾는다.

## row 사이 전이 궤적 (`stitch_transitions`)

**2026-09-15 부터 모든 모션 생성에 포함된다.** `nlp4cm/run_all.sh` 와 `run_fender_flat.sh` 는 파츠별 NLP 가 끝나면
바로 `stitch_transitions` 를 돌려 `stitched/<part>` 를 만들고(`log/<part>_stitch.log`), `viz/play.sh` 는 stitched 가
out 보다 새로우면 그것을 재생한다. 리트랙트 방향은 `--retract-dir spray`(기본) = 양끝 TCP 분사축 반대 방향이라
수평 부품에서는 월드 +Z 와 같고 세워진 범퍼에서도 표면에서 물러난다 (`world_z` 로 예전 방식 선택 가능).

planner 는 row 를 하나씩 만들고 IK/NLP 도 row 단위로 푼다. 그래서 row N 끝과
row N+1 시작 사이가 **비어 있고**, 재생 노드는 그 사이를 다음 row 첫 자세로
순간 이동한다. 즉 "경로를 따라간다"가 row 안에서만 성립한다.

`stitch_transitions` 가 그 사이를 실제 이동 궤적으로 채워 전체 시퀀스를 잇는다.

```bash
ros2 run paint_viz stitch_transitions \
  --joint-csv-dir ~/paint_cell_ws/nlp8_all --pattern "*_nlp_joint.csv" \
  --urdf $PAINT_CELL_REPO/motion_planning/paint_viz/urdf/paint_cell_8axis.urdf \
  --output ~/paint_cell_ws/stitched

ros2 launch paint_viz paint_viz.launch.py joint_csv_dir:=~/paint_cell_ws/stitched
```

전이 형상은 **들어올림 → 수평 이동 → 내림**이다 (분사 OFF 구간이라 표면을 따라갈
이유가 없고, 부품을 긁지 않아야 한다). 자세는 slerp 로 잇는다.

핵심은 IK 를 **양끝에서 각각 풀어 smoothstep 으로 섞는 것**이다. 8축은 널스페이스가
2차원이라 한쪽에서만 warm start 로 풀면 같은 TCP 자세라도 갠트리가 다른 위치로
수렴해 다음 row 와 이음새가 벌어진다 (실측: 최대 16 deg). 양끝 블렌딩은 가중치가
0→1 이라 **양 끝단이 정확히 일치**한다.

시간 배분은 rest-to-rest smoothstep 이고, 해석식만 믿지 않고 **만든 샘플에서 속도·
가속도를 재서 한계를 넘으면 총 시간을 늘려 다시 만든다** (해석식만 쓰면 실측에서
가속도가 한계의 4.3배까지 나왔다).

시간 배분은 **들어올림 / 이동 / 내림 세 구간에 각각** 건다 (2026-10-20, `time_parameterize_segments`).
한 번에 씌우면 폴리라인 모서리에서 관절 속도 방향이 한 샘플 안에 뒤집혀 가속도 스파이크가 나고,
그 한 점을 한계 아래로 누르느라 전이 전체가 기어간다 (범퍼 1.4 m 전이 40.6 s, 최고 속도가 한계의 12 %,
샘플의 99.9 % 가 가속도 한계의 절반 미만). 가속도 한계를 5 → 1.6 m/s² 로 내리자 1/√a 로 늘어나 드러났다.
구간별이면 모서리에서 속도 0 이라 꺾임이 무관하고 이동 구간이 한계까지 속도를 낸다. 로그의
`T=5.14s (0.62+3.90+0.62)` 가 구간별 시간이다. `sequence.py` 의 Cartesian 직선(retract 0)은 모서리가 없어 종전과 동일.

| 대상 | 인자 | 기본값 |
|---|---|---|
| 들어올릴 높이 | `--retract` | 0.03 m (아래 "리트랙트 높이는 왜 0.03 인가" 참고) |
| 데카르트 샘플 간격 | `--step` | 0.01 m |
| 출력 샘플 간격 | `--control-dt` | 0.01 s (planner 와 동일) |
| 회전축 속도/가속도 | `--max-joint-velocity-deg-s` / `-acceleration-deg-s2` | 70 / 500 |
| 직동축 속도/가속도 | `--max-gantry-velocity-m-s` / `-acceleration-m-s2` | 1.0 / 5.0 |

### ES300h_hood 실측 (NLP 입력, gantry.stp 실측 형상 + retract 0.03)

```
세그먼트        63개 (row 32 + move 31)
경계 불연속     0.000000        <- 전 62곳에서 정확히 연속
속도 한계 대비  0.76
가속도 한계 대비 1.00
총 시간         496.1s = 도장 460.2s + 전이 35.9s
전이 추종오차   최대 1.71 mm
```

### 리트랙트 높이는 왜 0.03 인가

예전 기본값 0.10 m 는 근거 없이 큰 값이었다. **실제로 필요한 높이**는 row N 끝에서
row N+1 시작까지를 직선으로 이었을 때 그 선분이 양끝보다 부품에 더 가까워지는
깊이인데, 재보면 이렇다.

```
전이 31곳 중 21곳 = 0.00 mm      (직선이 이미 안전)
최악 (30->31)     = 3.42 mm
평균              = 0.28 mm
```

planner 가 인접 row 를 지그재그로 내놓아 전이 이동거리가 50~83 mm 뿐이고, 그 짧은
구간에서 후드 표면이 솟지 않기 때문이다. 0.10 m 는 필요량의 29배였다.

리트랙트 스윕 실측 — **최소여유는 리트랙트와 무관하게 294.9 mm 로 고정**이다.
최소점은 항상 row 양끝(내려앉은 지점)에서 결정되므로 리트랙트를 올려도 안전 여유는
1 mm 도 늘지 않고, traverse(수평 이동) 구간 여유만 mm 당 약 1 초를 주고 사게 된다.

| retract | 전이 총 시간 | 부품 최소여유 | traverse 여유 |
|---|---|---|---|
| 0 mm | 15.5 s | 294.9 mm | 295.6 mm |
| 10 mm | 22.0 s | 294.9 mm | 302.6 mm |
| **30 mm** | **35.9 s** | 294.9 mm | 314.9 mm |
| 50 mm | 48.8 s | 294.9 mm | 320.9 mm |
| 100 mm | 78.5 s | 294.9 mm | 337.4 mm |

위 여유는 **TCP(`link_6` 원점)에서 부품 점군까지의 거리**다. `check_collisions` 가 찍는
값은 로봇 충돌메시 **정점** 기준이라 더 작다. 그쪽으로도 확인했다 — 같은 stride 4 에서
retract 100 mm 와 30 mm 가 **둘 다 279.7 mm 로 완전히 동일**하고, 최소점이 나오는
세그먼트까지 `seq_0010_row005`(도장 row)로 같다. row 세그먼트는 NLP 입력에서 그대로
복사되므로 리트랙트와 무관하고, `move` 세그먼트는 어떤 리트랙트 값에서도 row 만큼
부품에 가까워지지 않는다.

가속도가 전 구간에서 한계에 포화(0.99~1.00)해 있다. 즉 전이 시간은 가속도 제한이
결정하고 경로 길이의 제곱근에 비례하므로, 리트랙트를 줄인 만큼 그대로 시간이 준다.
(이 측정은 전이 전체에 smoothstep 하나를 씌우던 때의 것이다. 2026-10-20 구간별 시간 배분 이후에는
들어올림·내림 구간만 리트랙트 높이에 비례하고 이동 구간은 무관하다.)

측정된 최악값 3.42 mm 의 약 9배인 **0.03 m** 를 기본값으로 쓴다 (0.10 대비 42.6 s 절감).

**다른 부품에 그대로 쓰지 말 것.** 이 값은 ES300h_hood 기하에서 나왔다. 곡률이 크거나
row 순서가 지그재그가 아닌 부품은 필요 높이를 다시 재야 한다 (부품 점군과 row 양끝
TCP 만 있으면 계산된다). 근본적으로는 전이마다 필요 높이를 계산하는 적응형이 맞고,
고정값은 그 전 단계다.

**분사건이 아직 URDF 에 없다는 것도 걸린다.** 위 여유는 `link_6` 플랜지 기준이고
planner `offset_distance` 가 0.3 m 라 사실상 "건 길이만큼의 스탠드오프"다. 건 형상이
들어오면 여유가 크게 줄므로 그때 이 값을 재검토해야 한다.

### 입력은 NLP 결과를 쓸 것

DP-IK(`trajectory_ik_pinocchio.py`) 결과로 이어붙이면 잘 안 된다. row 를 독립적으로
풀면서 **31곳 중 13곳에서 IK 분기가 갈리기 때문**이다(팔이 30 deg 이상 튐, 최대
188 deg). 그 간극을 메우려면 전이가 관절공간을 크게 돌아야 해서 전이 총 시간이
5,204s 로 늘고 추종오차도 208 mm 까지 커진다.

NLP 는 연속 최적화라 분기가 갈리는 경계가 **0곳**이다. 전체 시퀀스를 이으려면
NLP 결과를 입력으로 쓸 것. DP-IK 를 쓰려면 row 간 분기 일관성을 먼저 강제해야 한다.

## 충돌 검사 (`check_collisions`)

생성된 관절 궤적이 무언가와 부딪히는지 본다. 특히 전이 궤적은 표면을 따라가지
않고 들어올렸다 옮기는 경로라, 부품/프레임을 스치는지 확인해야 한다.

```bash
ros2 run paint_viz check_collisions \
  --joint-csv-dir ~/paint_cell_ws/stitched \
  --urdf $PAINT_CELL_REPO/motion_planning/paint_viz/urdf/paint_cell_8axis_mesh.urdf \
  --scene $PAINT_CELL_REPO/motion_planning/paint_viz/config/scene.yaml \
  --stride 2 --part-stride 4
```

세 가지를 본다.

| 검사 | 방법 | 비고 |
|---|---|---|
| 자기 충돌 + 프레임 간섭 | pinocchio/coal 충돌쌍 | 팔은 원본 `a0912_collision` 메시 |
| 부품 간섭 | 로봇 충돌 정점 vs 부품 점군 KD-tree 최근접 | 점군 해상도만큼 근사 |
| 바닥 간섭 | 로봇 충돌 정점 최저 z | |

**collision 형상은 URDF 에 들어 있다.** `make_gantry_urdf` 가 갠트리 기둥/레일/빔/
캐리지/장착플레이트에도 `<collision>` 을 넣는다(시각 형상과 동일). 그래서 RViz 에
보이는 것이 곧 검사 대상이다. 이 추가는 운동학에 영향이 없다(FK 차이 0 확인).

**인접 쌍은 제외한다.** 같은 관절에 붙었거나 부모-자식 관절에 붙은 형상은 설계상
맞닿아 있다(레일-빔, 빔-캐리지, 캐리지-base_link). 253쌍 중 183쌍만 검사한다.
`addAllCollisionPairs()` 후 `removeCollisionPair()` 로 빼는 방식은 일부가 남아
구조물이 전 샘플에서 충돌로 잡혔다 — 그래서 **허용 쌍만 직접 추가**한다.

부품 점군 대신 메시를 쓰지 않는 이유는 planner 입력이 PLY 점군이기 때문이다.
최근접 거리는 점군 해상도만큼의 오차가 있으므로 "여유(clearance)" 지표로 본다.

## TCP 는 스프레이건 노즐 팁 (`tcp` 프레임)

planner 경로는 표면에서 `offset_distance` 0.3 m 떨어진 **노즐 위치** 다. 툴 장착 URDF 에서
노즐 팁은 플랜지 `link_6` 에서 (+0.129, −0.061, +0.297) m, 거리 0.329 m 앞에 있으므로
`link_6` 을 TCP 로 두고 풀면 건이 부품 속으로 0.3 m 들어간다. 2026-09-15 부터:

- 툴이 붙은 URDF 4종(`paint_cell_8axis*.urdf`, `_bare` 제외)에 **`tcp` 링크** 를 추가했다.
  `spray_gun` 원점(메시 최상단 정점 (0,0,0) = 노즐 팁)과 같은 자리, +z = 분사 방향.
- `--tcp-frame` / `tcp_frame` 기본값은 전부 **`auto`** = `tcp` → `spray_gun` → `link_6` 중
  URDF 에 있는 첫 프레임 (`paint_viz/tcp_frame.py`). 툴이 없는 URDF(6축 단독, `_bare`)는
  종전대로 `link_6` 이라 실기 6축 출력은 바뀌지 않는다. 플랜지 궤적을 보려면 `link_6` 을 명시.
- 적용 범위: `paint_viz.launch`/`dp_ik_viz.launch`(tcp_trail), `trajectory_player`, `solve_viz_ik`,
  `solve_dp_ik`, `stitch_transitions`, 그리고 `trajectory_nlp.py` / `trajectory_ik_pinocchio.py`.
  `nlp4cm/run_all.sh`·`stitch_all.sh` 는 `--tcp-frame tcp` 를 명시한다. **`nlp4cm/out` 의 기존
  결과는 `link_6` 기준이므로 `run_all.sh` → `stitch_all.sh` → `collision_all.sh` 순으로 다시 풀어야 한다.**
- RViz 기본 설정은 TF 디스플레이에서 `tcp` 프레임만 축으로 보여 준다.

## 갠트리 환경 선택 (`gantry_env`)

갠트리 치수·조인트 원점 규약·팔 장착 오프셋·툴체인저 슬롯·부품 배치 오프셋은 전부
`config/gantry/<name>.yaml` 한 파일이 정의한다. 모든 스크립트는 `--gantry-env <name>`,
launch 는 `gantry_env:=<name>`, nlp4cm 셸 스크립트는 환경변수 `GANTRY_ENV` 하나로 고른다.
`--urdf` / `urdf:=` 를 명시하면 그대로 쓰고(종전 동작), 비우면 환경의 URDF 를 쓴다.

| 환경 | 레일 (X x Y, 높이) | 스트로크 | 조인트 원점 | 장착면 | URDF | 쓰는 곳 |
|---|---|---|---|---|---|---|
| `legacy` (기본) | 5.0 x 2.8 m, 2.314 m | 4.392 / 2.142 | 중앙(±), 축 정방향, yaw +90° | 2.066 m | `paint_cell_8axis.urdf`, `_mesh.urdf` (기존 파일, 재생성 안 함) | NLP 도장 파이프라인(nlp4cm), paint_viz/dp_ik_viz launch 기본 |
| `spec_c` | 3.0 x 2.4 m, 2.1805 m | 2.8264 / 2.2864 | 코너 [0, stroke], 두 축 반전, yaw −90°, 팔 +X 0.323 | 2.0275 m | `paint_cell_8axis_gantry_mesh_c.urdf`, `_c_bare.urdf` (수작업 최종본, 재생성 안 함) | 툴체인저 dock_path, cell_view launch 기본 |
| `x7y4z3` | 7.0 x 4.0 m, 3.0 m | 6.8264 / 3.8864 | spec_c 규약 상속 | 2.847 m | `paint_cell_8axis_x7y4z3.urdf`, `_mesh.urdf`, `_mesh_bare.urdf` (`make_gantry_urdf --env x7y4z3` 로 생성) | 신규 환경 |

```bash
ros2 run paint_viz gantry_env --list                                   # 환경 목록
ros2 run paint_viz gantry_env --name x7y4z3 --json                     # 치수/한계/슬롯 요약
ros2 run paint_viz gantry_env --name x7y4z3 --get urdf.mesh            # 파일 경로 조회 (셸 스크립트용)
ros2 run paint_viz gantry_env --name x7y4z3 --part-xyz="-0.751,-1.316,1.879"   # legacy 기준 배치 -> 이 환경 좌표

ros2 run paint_viz make_gantry_urdf --env x7y4z3                       # URDF 3종 생성 (프리미티브/메시/베어)
ros2 launch paint_viz cell_view.launch.py gantry_env:=x7y4z3           # 형상 확인
ros2 run paint_viz dock_path --gantry-env x7y4z3 --slot 1 --out-dir /tmp/dock_x7
GANTRY_ENV=x7y4z3 $NLP4CM/run_all.sh                    # NLP + stitch -> nlp4cm/out_x7y4z3, stitched_x7y4z3
GANTRY_ENV=x7y4z3 $NLP4CM/viz/collision_all.sh          # 충돌 검사 -> collision_x7y4z3
GANTRY_ENV=x7y4z3 $NLP4CM/viz/play.sh ES300h_hood       # RViz 재생
```

규칙
- **legacy 는 기존 결과와 바이트 동일**해야 한다. `make_gantry_urdf` 의 인자 없는 기본 호출, `--env legacy` 의
  갠트리 부분, nlp4cm `run_all.sh` 의 기본(`GANTRY_ENV` 미설정) 출력이 전부 종전과 같은지 검증했다(2026-10-13).
- 치수는 yaml 에만 둔다. `x7y4z3` 는 레일 길이(7/4 m)에서 spec_c 와 같은 슬라이더군 여유(X 0.1736 / Y 0.1136)를
  뺀 값을 스트로크로, 레일 높이 3 m 에서 spec_c 와 같은 0.153 m 를 뺀 값을 장착면으로 쓴다.
- 툴체인저 슬롯/스탠드는 anchor(캐리지 이동범위 끝 `x_max`,`y_min` + 팔 장착면 `mount_z`) 기준 상대좌표다.
  환경이 바뀌어도 팔 base-슬롯 관계가 같아서 dock_path 의 팔 해가 그대로 나온다 (x7y4z3 검증: spec_c 와 팔 6축 동일,
  gantry_y 만 +1.6 m). 바닥 기준 절대 높이로 두고 싶으면 `anchor.z: floor` 로 바꾸고 `dz` 를 절대값으로 적는다.
- 부품 배치(`part_xyz`)는 legacy 월드 기준 값을 그대로 쓰고 `part_placement.xyz_offset` 을 더한다. x7y4z3 는
  z +0.781 (= 2.847 − 2.066): 지그를 같이 올려 팔 base 대비 TCP 높이를 legacy 와 같게 둔다. 안 올리면 후드 경로가
  base 아래 1.7~1.9 m 라 A0912(1.395 m + 노즐 0.33 m) 도달 밖이다.
- 출력 추적: `trajectory_nlp.py` / `trajectory_ik_pinocchio.py` 가 출력 디렉터리에 `plan_metadata.json`
  (환경 이름·치수·조인트 한계·URDF·TCP·부품 배치·입력/출력 파일·인자) 을 남긴다.
- nlp4cm 는 legacy 면 종전 경로(`out/`, `stitched/`, `collision/`, `log/`), 그 외는 `out_<env>/` 식 접미사로 분리한다
  (`nlp4cm/env.sh`). scene yaml 은 `viz/env_<env>/scene_<part>.yaml` 로 part_xyz 만 바꿔 자동 생성된다.

## 툴체인저 + 도장 통합 시퀀스 (`build_sequence`)

홈 → 툴체인저 슬롯 접근·장착 → 이탈 → 도장 시작점 접근 → CSV 도장(stitched row+전이) → 후퇴 → 반납 → 홈을
한 시퀀스로 만든다. segment 순서·TCP·속도는 `config/sequence/<name>.yaml`, 갠트리/툴체인저/홈 자세는
`gantry_env` yaml 이 정한다 (`paint_viz/sequence.py`).

```bash
# 선행: GANTRY_ENV=x7y4z3 nlp4cm/run_all.sh  (stitched_x7y4z3/<part> 가 있어야 한다)
ros2 run paint_viz build_sequence --gantry-env x7y4z3 --part ES300h_hood \
    --out-dir $NLP4CM/sequence_x7y4z3/ES300h_hood          # [--sequence paint_with_tool] [--slot 1]
ros2 launch paint_viz paint_viz.launch.py gantry_env:=x7y4z3 \
    joint_csv_dir:=$NLP4CM/sequence_x7y4z3/ES300h_hood joint_csv_pattern:="seq_*_joint.csv" \
    scene:=$NLP4CM/viz/env_x7y4z3/scene_ES300h_hood.yaml
```

출력
- `seq_NNNN_<segment>_<stage>_joint.csv` — stage 별 CSV (stitch 와 같은 26열 SI). 도장 row/전이는 stitched 파일을 바이트 그대로 복사.
- `sequence.json` — 매니페스트: stage 순서·누적 시간·TCP 프레임·툴 상태·충돌 모델, 검증 결과(경계 점프, 충돌, 한계).
- `sequence_command.csv` — 실행용 단일 1 ms CSV (t, s, q, qd, qdd, segment, tool_attached, tcp_frame, TCP pose).
  TCP 쿼터니언(xyzw)은 이전 샘플과 내적이 음수면 부호를 뒤집어 반구 연속.

설계
- 모든 segment 가 rest-to-rest(도장 row 는 planner 가 양끝 속도 0, 나머지는 smoothstep)라 경계에서는 관절값 일치만
  보장하면 된다. 각 stage 는 이전 stage 의 마지막 q 에서 시작하고(warm start), Cartesian 구간은 양끝 IK + smoothstep
  블렌딩(`stitch.solve_transition_path`)이라 끝 q 가 다음 stage 의 첫 q 와 정확히 같다 (검증: 경계 점프 0).
- TCP 전환은 0 초 이벤트(`tool_attach` / `tool_detach`, `event_dwell_s` 만큼 정지). 관절값은 TCP 와 무관하므로 그 뒤
  segment 의 목표 프레임만 `tcp` / `link_6` 으로 바뀐다. 충돌 검사는 베어 URDF / 툴 URDF 를 나눠 쓰고, 툴을 든 동안
  그 슬롯의 파킹 플레이트 박스를 빼며, 슬롯 바로 위 수직 구간에서는 툴 어셈블리 박스와 스탠드 브래킷 쌍을 뺀다
  (브래킷 절개부를 지나가는 구조라 박스끼리 겹치는 것이 정상). 스프레이건은 항상 검사한다.
- 긴 이동은 "팔을 이송 자세로 접고 갠트리만 이동 → 팔 재배치(관절 보간) → 목표 앞에서 직선 접근". 접근/후퇴 방향은
  분사축 반대 방향 기준으로, 세운 부품(수평 성분 45° 이상)은 **수평 성분만** 써서 갠트리 평행이동으로 정확히 실행하고
  (팔이 끝까지 뻗은 하단 row 에서 더 내려갈 필요가 없다), 누운 부품은 월드 +Z 로 들어올린다 (`approach.lift_m` 0.15).
  툴을 든 채 슬롯을 떠날 때는 `dock.depart_lift_m`(0.35) 수직 상승 → `depart_retreat_xyz`(+Y 0.3) 수평 후퇴 → 재배치
  (0.15 m 만 올리고 돌리면 건이 옆 슬롯 플레이트/상단 브래킷을 쓸었다 — 2026-10-13 실측 침투 30~95 mm).
- 속도: 도장은 NLP 1 ms 명령(0.42 m/s) 그대로. 도킹 접기/재배치 관절 구간은 한계 ×0.25, 하강 TCP 0.02 m/s,
  상승·후퇴 0.05 m/s, 도장 접근/후퇴 0.1 m/s, 갠트리 이송 1.0 m/s (yaml `speeds`).
- 출력은 stage 별 CSV + 매니페스트를 기본으로 두었다. 도장 CSV 가 바이트 그대로 보존돼 회귀 비교·부분 재생성이
  쉽고, 통합 1 ms CSV 는 컨트롤러 입력 전용이다.

## 갠트리 + 로봇만 보기 (`cell_view.launch.py`)

부품/경로/재생 없이 URDF 형상과 TCP 위치만 확인하는 launch. 기본 자세는 **`trajectory_nlp.py` 의
`center_weight` 항이 당기는 관절범위 중앙값 q_mid = (lower+upper)/2** 다. 갠트리 모델에서 NLP 는
joint_3 / joint_5 bound 를 `joint_3=25:135,joint_5=25:135` 로 덮어쓰므로(`ARM_JOINT_BOUNDS_DEG`, auto)
그 둘의 중앙은 80°, 나머지 팔 관절은 URDF 대칭 한계라 0°, `gantry_x`/`gantry_y` 는 limit 중간값(최종본
`_c` 는 1.4132 / 1.1432 → 브리지·캐리지가 월드 XY 원점 위). `joint_bounds_deg` 인자로 bound 를 바꾸면
그 중앙으로, `arm_deg` 를 주면 그 값으로 띄운다. `cell_pose_info` 노드가 `/joint_states` 를 받아
`tcp` / `link_6` 월드 좌표를 로그와 RViz 텍스트 마커로 찍는다.

```bash
ros2 launch paint_viz cell_view.launch.py                    # _c URDF, NLP 중앙값 자세
ros2 launch paint_viz cell_view.launch.py gui:=true          # 슬라이더(joint_state_publisher_gui), 초기값 = 중앙값
ros2 launch paint_viz cell_view.launch.py joint_bounds_deg:="joint_3=40:120,joint_5=25:135"
ros2 launch paint_viz cell_view.launch.py arm_deg:="0,0,90,0,90,0" gantry_x:=0.5
ros2 launch paint_viz cell_view.launch.py urdf:=$(ros2 pkg prefix paint_viz)/share/paint_viz/urdf/paint_cell_8axis_mesh.urdf
```

`cell_view.launch.py` 의 `NLP_ARM_JOINT_BOUNDS_DEG` 는 `trajectory_nlp.py` 의 `ARM_JOINT_BOUNDS_DEG` 와
같은 값을 유지해야 한다 (한쪽을 바꾸면 다른 쪽도).

**페인트통 간섭.** 경로 회전이 분사축 둘레 회전(yaw)까지 고정하므로 컵(tcp +x, 0.36 m) 방향은 경로가 정하고,
NLP 는 center 항(joint_6→0°) 때문에 팔을 컵 쪽에 매달아 컵끝이 팔꿈치에 0.2 m 까지 붙었다. 경로축 부호 뒤집기나
joint_6 bound 로는 안 되고(팔이 따라 돌거나 수평 분사 파츠가 Infeasible), `trajectory_nlp.py --tool-clearance-weight`
(컵끝 = tcp +x 0.30 m 가 joint_3/joint_5 원점에서 0.35 m 이상, 소프트 페널티) 로 푼다. `run_all.sh` 가 100 을 준다.
로그 `tool_clearance=` 가 그 최소거리다.

## 보이는 것

| Display | 토픽 | 내용 |
|---|---|---|
| PartCloud | `/scene_publisher/part_cloud` | 부품 표면 점군 (PLY 정점 색 그대로) |
| PaintPaths | `/scene_publisher/paint_paths` | row별 TCP 경로. 분사 ON은 row 색, OFF는 회색 반투명 |
| RobotModel | `/robot_description` | 갠트리 2축 + A0912 6축 |
| TcpTrail | `/trajectory_player/tcp_trail` | 실제로 지나간 TCP(`tcp` = 노즐 팁) 자취 |
| TF | – | `tcp` 프레임 축만 표시 |
| Status | `/trajectory_player/status` | 현재 row / 진행 시간 |

## 셀 구조

갠트리 스트로크 **4.392 x 2.142 m**, A0912 를 캐리지에 **천장 장착**해서 아래로 뻗는다.
world 원점은 갠트리 프레임 중앙의 **바닥(z = 0)** 이다.

```
world ──(scene.yaml part_xyz/part_rpy)── part      ← 점군과 경로가 여기 붙는다
  └─ world_base (바닥, 받침 4 + 기둥 4 + X 박스빔 2 + X 레일 2)
       └─ gantry_x  prismatic X  ±2.196 m ─ gantry_x_link  (Y 이송 브리지, z=2.314 m)
            └─ gantry_y  prismatic Y  ±1.071 m ─ gantry_y_link  (캐리지, z=2.314 m)
                 └─ gantry_to_arm  fixed, rpy = (pi, 0, pi/2)  ← 뒤집고 90도 돌린다
                      └─ base_link (z=2.066 m) ── link_1..link_6  (아래로 뻗음)
```

치수는 전부 `gantry.stp` (원본 CAD, 저장소 밖) (Solid Edge 어셈블리, 갠트리+로봇 통합)
에서 뽑은 **실측값**이다. 바닥 datum 은 방진 레벨링 풋 하면이고, CAD 상 좌/우 열이
4 mm 어긋나 있어 평균을 썼다 — 아래 높이값은 ±2 mm 불확실성이 있다.

| 치수 | 값 | 근거 |
|---|---|---|
| X 스트로크 | ±2.196 m | 레일 5.0 m, 슬라이더군 외곽 ±0.304 m |
| Y 스트로크 | ±1.071 m | 레일 2.8 m, 슬라이더군 외곽 ±0.329 m |
| 기둥 | 244 x 175 x 1700 mm, X ±1.744 / Y ±1.5095 m | H형강 4주 |
| X 박스빔 | 5300 x 200 x 250 mm | 상면이 브리지 중심 -247 mm |
| 브리지 | 150 x 3300 x 250 mm | `single_box_beam_3500` (이름과 달리 실제 3300) |
| 캐리지 | 257 x 650 x 466 mm | 브리지를 감싸고 내려온다 |
| 팔 장착면 (`base_link`) | 2.066 m | `--mount-z`. CAD 베이스 원점 -248, 바닥 -2314 |

스트로크는 슬라이더 블록이 레일 끝에 닿는 **기하학적 한계**다. 엔드스톱/케이블베어
여유는 반영돼 있지 않으니 실기 적용 시 양단에서 덜어내야 한다. 랙기어 길이
(X 5010.7, Y 2610.6 mm)는 둘 다 이보다 넉넉해서 구속 조건이 아니다.

`gantry_to_arm` 의 `rpy = (pi, 0, pi/2)` 가 팔을 뒤집어 `base_link` 의 +Z(팔이 뻗는
방향)를 월드 -Z 로 돌리고, CAD 실측 장착 yaw 90도를 반영한다. planner 경로의
분사축이 부품 좌표계 -Z(아래)이므로 이 방향이 맞다.
검증: 모든 관절 0 에서 `link_6` 의 z축(분사) = `[0, 0, -1]`, 위치 z = 0.671 m
(장착면 2.066 m 에서 정확히 1.395 m 아래 = 최대 도달).

부품 배치를 TF로 분리했기 때문에 `config/scene.yaml` 의 `part_xyz`/`part_rpy` 하나만
바꾸면 **표시와 IK가 같은 배치**를 쓴다. launch와 `solve_viz_ik` 가 같은 파일을 읽는다.

## 왜 8축인가

`ES300h_hood` 경로는 XY로 1.57 x 1.66 m 를 덮는데 A0912 도달 반경은 약 1.40 m 다.
베이스를 고정하면 후드 전체를 한 자세에서 칠할 수 없다 (고정 베이스로 풀면
추종률 15%). 갠트리 직동 2축을 얹으면 같은 경로가 100% 추종된다.

현재 배치(`scene.yaml` 기본값)에서 ES300h_hood 32 row / 11,533 점, **gantry.stp 실측
형상** 기준 재측정 (`--stride 4`):

```
추종률          100.0 %  (위치 오차 최대 2.0 mm, 자세 오차 최대 0.11 deg)
gantry_x        -0.41 ~ +1.05 m   (±2.196 m 스트로크, 여유 1.15 m)
gantry_y        +0.07 ~ +1.07 m   (±1.071 m 스트로크, 여유 0.00 m)  ← 상한 포화
팔 6축 사용폭    29 ~ 96 deg      (관절 한계에서 멀다)
```

> ⚠️ **`gantry_y` 가 상한에 붙는다 — 전체 점의 20.9 %.**
> `part_xyz` 는 예전 가정값 ±1.5 m 스트로크에 맞춰 튜닝된 값인데, 실제 Y 스트로크는
> ±1.071 m 뿐이라 그 튜닝이 더 이상 유효하지 않다. 추종률이 100 % 인 것은 팔 6축이
> 흡수하기 때문이고, 실기에서는 하드스톱에 붙어 도는 상태라 그대로 쓰면 안 된다.
>
> 부품을 Y로 -0.57 m 옮겨보면 포화가 20.9 % → 3.6 % 로 줄지만 없어지지는 않는다.
> 즉 **부품 배치 문제만이 아니라 8축 여유자유도 해소(null-space)가 Y 한계로
> 흘러가는 것**이 같이 작용한다. `part_xyz` 재튜닝과 IK 의 관절한계 회피(예:
> null-space 에 중앙 복귀 항 추가)를 같이 봐야 한다.

**형상 가정은 이제 없다.** 스트로크·장착 높이·프레임 치수 전부 `model/gantry.stp`
실측이다. 남은 불확실성은 (1) 바닥 datum 의 CAD 좌/우 4 mm 어긋남(±2 mm),
(2) 스트로크가 엔드스톱 여유를 뺀 값이 아니라는 것 두 가지다.
팔 부분 운동학은 원본 `a0912.white.urdf` 와 동일함을 FK로 확인했다
(랜덤 2000 자세, 두 URDF 간 FK 차이 0).

## 메시가 없는 이유

원본 URDF는 형상을 `package://dsr_description2/meshes/...` 로 참조한다. 이 패키지가
apt 에도 `/opt/ros/humble` 에도 없어서, **두산 공식 저장소의 a0912 메시만 뽑아
`src/dsr_description2/` 에 최소 패키지로 넣어 뒀다** (a0912_white + a0912_collision,
28개 `.dae`, 15 MB). urdf/xacro/launch/다른 기종은 들어 있지 않다. 원본:
`https://github.com/doosan-robotics/doosan-robot2` (humble 브랜치).

그래서 URDF 가 두 개다.

| 파일 | 형상 | 만드는 법 |
|---|---|---|
| `paint_cell_8axis_mesh.urdf` | **원본 메시** (launch 기본값) | `make_gantry_urdf --keep-meshes` |
| `paint_cell_8axis.urdf` | 실린더+구 프리미티브 | `make_gantry_urdf` (기본) |

두 URDF의 **운동학은 완전히 동일**하다 (관절 이름·한계 일치, 랜덤 2000 자세 FK 차이 0).
형상만 다르므로 IK/NLP 는 어느 쪽을 써도 결과가 같다. `dsr_description2` 가 없는
환경으로 옮기면 프리미티브 쪽으로 실행하면 된다.

```bash
ros2 launch paint_viz paint_viz.launch.py \
  urdf:=$(ros2 pkg prefix paint_viz)/share/paint_viz/urdf/paint_cell_8axis.urdf
```

## solve_viz_ik 는 시각화 전용이다

실기 실행용은 `robot_control/.../trajectory_ik_pinocchio.py` 다. 그쪽은 DP 기반
후보 선택과 관절 점프/속도/가속도 실현성 게이트를 통과해야 하고, 현재 데이터에서는
그 게이트에서 멈춘다.

`solve_viz_ik` 는 RViz 애니메이션을 만들기 위한 대안으로, 이전 해를 warm start로
쓰는 damped least squares IK만 돌린다. **관절 속도·가속도 한계를 보장하지 않으므로
로봇에 그대로 넣으면 안 된다.** 대신 점마다 추종 오차를 재고 허용오차를 넘으면
미도달로 집계한다.

출력 CSV는 `t, s, <관절이름들>` (회전 rad, 직동 m). 갠트리 직동축이 섞여
`trajectory_ik_pinocchio.py` 의 deg 단일 단위 형식을 쓸 수 없다. 재생 노드는 두 형식을
모두 읽는다.

## DP-IK 를 따로 보기 (`solve_dp_ik` + `dp_ik_viz.launch.py`)

`solve_viz_ik` 는 이전 해를 warm start 로 쓰는 DLS IK **한 번**이라 관절 점프·속도·
가속도를 전혀 보지 않는다. 실기 실행용인 `trajectory_ik_pinocchio.py` 는 점마다 여러
분기해를 만들어 놓고 DP 로 실현 가능한 경로를 고르는데, 그 DP-IK 가 8축 셀에서 실제로
어떻게 움직이는지(그리고 어디서 막히는지) 보려면 별도 경로가 필요하다.

`solve_dp_ik` 가 그 IK 코어(다중 시드 DLS → 후보 필터링 → 점프/속도/가속도 DP)를
**그대로 옮긴** 것이다. 비용식·게이트·시드 오프셋·가중치 기본값이 원본과 같다.

```bash
ros2 run paint_viz solve_dp_ik \
  --scene $PAINT_CELL_REPO/motion_planning/paint_viz/config/scene.yaml \
  --urdf  $PAINT_CELL_REPO/motion_planning/paint_viz/urdf/paint_cell_8axis.urdf \
  --output ~/paint_cell_ws/dp_ik --stride 4

ros2 launch paint_viz dp_ik_viz.launch.py speed_scale:=8.0
```

기존 `paint_viz.launch.py` 와 **완전히 분리돼 있다** — 노드 이름(`dp_` 접두어), RViz
설정(`dp_ik_viz.rviz`), 입력 디렉터리(`~/paint_cell_ws/dp_ik`), 파일 패턴
(`*_dp_joint.csv`) 이 전부 다르다. 다만 `/joint_states` 와 `/tf` 는 공유이므로
**같은 도메인에서 둘을 동시에 띄우면 안 된다.** NLP 결과와 나란히 비교하려면
한쪽을 다른 도메인으로 띄운다.

```bash
ROS_DOMAIN_ID=7 ros2 launch paint_viz dp_ik_viz.launch.py
```

### 원본과 다른 점

| | `trajectory_ik_pinocchio.py` | `solve_dp_ik` |
|---|---|---|
| 입력 | CSV 파일 목록 + `--part-xyz/--part-rpy` | `scene.yaml` 의 `trajectory_dir` 전체 |
| 출력 | joint CSV + 1 kHz 명령 CSV | SI joint CSV 하나 |
| 로봇 실행 | `--execute` (6축 전용) | 없음 |
| DP 실패 | 예외로 중단 | `--on-failure skip`(기본)이면 그 row 만 버리고 계속 |

IK 계산 자체는 같지만 **실기 실행용이 아니다.** 1 kHz 명령 궤적을 만들지 않고, 저장하는
속도·가속도 점검값은 시간 기반 미분으로 사후에 잰 것이다.

### 막히는 지점을 보라고 만든 도구다

DP 가 게이트에서 막히면 그 row 만 건너뛰고 `dp_ik_summary.csv` 에 이유를 남긴다.
전부 막히면 종료코드 1 이다.

```
row,status,points,message
trajectory_000,dp-failed,33,점 1-2 사이에 점프/속도 실현 가능한 연결이 없다
```

**row 경계의 IK 분기 갈림도 같이 잰다.** README 위쪽 "입력은 NLP 결과를 쓸 것" 에서
지적한 문제를 눈으로 확인하려는 것이다. 요약의 `branch_jump` 열이 직전 row 마지막
자세와 현재 row 첫 자세의 최악 축 차이다 — 실측(`--stride 4`, 앞 3 row):

```
trajectory_000  ok
trajectory_001  ok   joint_4 45.427deg     <- 재생하면 여기서 팔이 튄다
trajectory_002  ok   gantry_y 0.057m
```

이 값이 크면 `stitch_transitions` 로 이어붙여도 전이가 관절공간을 크게 돌게 된다.

### 비용

점당 최대 48 시드 × 100 반복이라 `solve_viz_ik` 보다 훨씬 느리다. 실측 8축 셀
ES300h_hood 기준 **약 23 ms/점** (`--stride 4` 에서 row 하나 320점에 6~10 s).
전체 32 row 를 stride 4 로 돌리면 4~5 분쯤 걸린다. 먼저 `--max-rows 3` 으로
훑어보는 편이 낫다.

## 주요 파라미터

| 대상 | 인자 | 기본값 |
|---|---|---|
| launch | `speed_scale` | 1.0 (재생 배속) |
| launch | `urdf` | `urdf/paint_cell_8axis.urdf` |
| launch | `joint_csv_dir` | `~/paint_cell_ws/viz_ik` |
| launch | `rviz` | true |
| launch / 모든 솔버 | `tcp_frame` / `--tcp-frame` | `auto` (tcp → spray_gun → link_6) |
| cell_view.launch | `gantry_x` / `gantry_y` / `arm_deg` / `joint_bounds_deg` / `gui` | center / center / center / `joint_3=25:135,joint_5=25:135` / false |
| trajectory_nlp | `--tool-clearance-weight` / `--tool-clearance-m` / `--tool-tip-offset` | 0 (run_all.sh 는 100) / 0.35 m / `0.30,0,0` |
| solve_viz_ik | `--stride` | 1 (4면 점 1/4만 풀어 빠르게) |
| solve_viz_ik | `--position-tolerance` | 0.002 m |
| solve_dp_ik | `--stride` | 1 (점당 23 ms 라 4 권장) |
| solve_dp_ik | `--on-failure` | skip (DP 막힌 row 만 버리고 계속) |
| solve_dp_ik | `--position-tolerance` | 0.0001 m (원본과 동일, viz_ik 보다 20배 빡빡) |
| solve_dp_ik | `--ik-max-candidates-per-point` | 12 |
| dp_ik_viz.launch | `joint_csv_dir` | `~/paint_cell_ws/dp_ik` |
| make_gantry_urdf | `--x-travel` / `--y-travel` | 4.392 / 2.142 m (gantry.stp 실측) |
| make_gantry_urdf | `--mount-z` | 2.066 m (팔 장착면 높이, gantry.stp 실측) |
| make_gantry_urdf | `--env` | 없음 (주면 config/gantry/<env>.yaml 치수로 프리미티브/메시/베어 3종 생성) |
| 모든 스크립트 | `--gantry-env` | `$GANTRY_ENV` 또는 legacy. `--urdf` 생략 시 환경 URDF |
| launch 3종 | `gantry_env` | paint_viz/dp_ik_viz: legacy, cell_view: spec_c. `urdf` 비우면 환경 urdf.mesh |
| scene.yaml | `part_file` / `trajectory_dir` | ES300h_hood |
| scene.yaml | `part_xyz` | `[-0.751, -1.316, 1.879]` (후드 상면 = 바닥 1.00 m) |
