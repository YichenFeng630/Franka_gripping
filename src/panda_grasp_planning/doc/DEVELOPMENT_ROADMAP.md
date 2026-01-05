# Panda Grasp Planning + VLA  
## Simulation-first Development Roadmap (Revised)

**Last updated**: 2026-01-04  
**Maintainer**: Yichen Feng  

---

## 🎯 Development Strategy

This project follows a **simulation-first, real-robot-later** strategy.

- All **high-level decision making (VLA)** and **learning-based policies** are first
  developed and validated in simulation.
- **Real-robot experiments** are conducted only after a working simulation demo
  is available.
- **Classical perception and calibration** are finalized on the real robot.
- The existing **optimization-based grasp pipeline (V3)** is always preserved as
  a **safety fallback**.

This strategy minimizes hardware risk, accelerates iteration, and ensures
a clear comparison between optimization-based and learning-based methods.

---

## 🧭 Phase Overview (Revised)

| Phase | Sub-phase | Goal | Simulation | Real Robot |
|------|----------|------|------------|------------|
| **0** | 0S / 0R | Action & control interface frozen | ✅ | ✅ |
| **1** | 1S / 1R | Grasp + sorting baseline | ✅ | ⚠️ |
| **2** | 2S / 2R | VLA decision demo | ✅ | ⚠️ |
| **3** | 3S / 3R | Learning-based vision → action | ✅ | ⚠️ |
| **4** | — | Unified modes + safety | ✅ | ✅ |

Legend:
- ✅ Required
- ⚠️ Optional / partial
- ❌ Not required

---

## Phase 0 — Control & Interface Foundation

### Phase 0S — Simulation

**Goal**  
Freeze the **action space** and **execution interface** independently of hardware.

**Key tasks**
- Define unified action space  
  `[dx, dy, dz, droll, dpitch, dyaw, gripper]`
- Implement `ActionExecutor` (simulation version)
- Verify action execution in simulation (MoveIt + Gazebo / MuJoCo)
- Validate workspace limits and safety constraints

**Exit criteria**
- ActionExecutor can reliably execute small motions in simulation
- No dependency on real sensors or cameras

---

### Phase 0R — Real Robot (Deferred)

**Goal**  
Prepare hardware interfaces without blocking simulation progress.

**Key tasks**
- ZED2 ROS driver setup
- Eye-in-hand calibration (`panda_hand → camera_link`)

**Note**  
Phase 0R must **not block Phase 1S–3S**.

---

## Phase 1 — Grasp & Sorting Baseline (Optimization-based)

### Phase 1S — Simulation

**Goal**  
Establish a **fully functional grasp + sorting baseline** using the V4 pipeline.

**Status**: ✅ **COMPLETE** (2026-01-04)

**Strategy**
- No visual perception
- Use **ground-truth object pose and color**
- Focus on execution robustness and task logic

**Key Achievements**
- ✅ V4 Pipeline: 100% success rate (20/20 trials)
- ✅ Fixed critical V3 bugs:
  - Gripper width: 20mm → 43mm (contacts 45mm cube)
  - TCP offset: +0 → +10.3cm (finger tips above table)
  - State feedback: Estimated → Real-time monitoring
- ✅ Color system implemented (RED, BLUE, GREEN, YELLOW)
- ✅ Sorting state machine (color → bin mapping)
- ✅ Place-to-bin execution (Cartesian + gripper control)
- ✅ Microadjustment mechanism (handles edge grasps)

**Final Results**
```
Trials: 20
Success Rate: 100% (20/20) ✅
Avg Time: 15s per trial
Collisions: 0
Gripper State: Real-time feedback ✅
```

**Exit Criteria - ALL MET**
- [x] End-to-end grasp + place works reliably
- [x] Baseline performance metrics available
- [x] 100% success rate achieved
- [x] Zero collisions or safety incidents
- [x] Color sorting fully integrated
- [x] Real-time gripper state feedback

**Implementation Files**
- `scripts/v4_demo.py` (900 lines) - Main pipeline
- `modules/sorting/sorting_state_machine.py` (248 lines) - Sorting logic
- `franka_zed_gazebo/scripts/spawn_cubes.py` - Colored cube spawning

---

### Phase 1R — Real Robot (Perception Integration)

**Goal**  
Connect real perception to the V4 baseline pipeline.

**Status**: ⚠️ **PENDING** (Optional - not blocking Phase 2S)

**Key tasks**
- Color segmentation from ZED2 RGB
- Point cloud centroid estimation
- Publish `/target_cube_pose` to V4 pipeline

**Scope limitation**
- No requirement for high perception accuracy
- Parameter tuning expected later
- No feedback to simulation pipeline

**Note**: Phase 1R can be deferred. V4 baseline is complete and Phase 2S can proceed with ground-truth poses in simulation.

---

## Phase 2 — Vision-Language-Action (VLA)

### Phase 2S — Simulation (Core Demo Phase)

**Goal**  
Produce a **demonstrable VLA-based decision-making demo** leveraging the stable V4 baseline.

**Baseline Status**: ✅ Complete (Phase 1S finished)

**Role of VLA**
- Input:
  - Simulated RGB image
  - Grasp candidates (from V4 pipeline)
  - Language instruction (e.g., "pick the red cube")
- Output:
  - Object selection
  - Grasp candidate ranking or index

**Integration with V4 Baseline**
- V4 pipeline provides: stable grasp execution, gripper state feedback, place-to-bin logic
- VLA adds: semantic understanding of language instructions and scene context
- Result: Language-guided pick-and-place

**Explicit non-goals**
- No continuous control
- No probability calibration
- No replacement of motion planning (V4 remains responsible for execution)

**Exit criteria (Demo-ready)**
- Changing language instructions leads to different decisions
- VLA affects *what* is grasped and *where*, not *how it moves*
- Stable integration with V4 execution pipeline

**Demo readiness gate (recommended)**
- At least 3 instruction categories trigger distinct selections
  (e.g., color / size / spatial relation)
- Repeatability: >= 80% success across 10 fixed-scene trials in simulation
- No invalid outputs (empty candidates, unreachable grasp, or NaN scores)
- One-click demo script or a deterministic sequence for public viewing

**Next Steps**
1. Design VLA interface (input/output format)
2. Collect/synthesize training data from V4 baseline
3. Train lightweight VLA model
4. Integrate with v4_demo.py
5. Run demo with language instructions

---

### Phase 2R — Real Robot (Low-risk Transfer)

**Goal**  
Reuse the same VLA adapter on real images.

**Key tasks**
- Replace simulated RGB with ZED2 RGB
- Keep execution entirely in V3 pipeline

**Fallback**
- Any failure immediately reverts to Phase 1R baseline

---

## Phase 3 — Learning-based Vision → Action

### Phase 3S — Simulation (Main Learning Track)

**Goal**  
Demonstrate **image → action** learning in a controlled environment.

**Recommended order**
1. **ACT / Behavior Cloning**
   - Short-horizon imitation learning
2. **Diffusion Policy** (if time permits)
   - Smoother trajectories
   - Increased robustness

**Data source**
- Expert demonstrations generated by Phase 1S baseline

**Exit criteria**
- Learned policy can grasp objects in simulation
- Success rate lower than baseline is acceptable

---

### Phase 3R — Real Robot (Exploratory)

**Status**
- Optional / bonus
- Not required for core project success

**Possible tasks**
- Zero-shot policy evaluation
- Limited fine-tuning with safety gate

---

## Phase 4 — Unified Execution & Safety

**Goal**  
Enable systematic comparison between methods with guaranteed safety.

### Operation Modes

- **Mode 0**: Optimization baseline  
  `Perception → V3 pipeline`
- **Mode 1**: VLA-augmented planning  
  `Perception → VLA → V3 pipeline`
- **Mode 2**: Learning-based control  
  `Perception → Policy → ActionExecutor + Safety Gate`

### Safety Principles

- Workspace, velocity, and acceleration limits
- Collision checks
- NaN / instability detection
- Automatic fallback to Mode 0

---

## 📌 Summary

- Simulation is the **primary development and validation platform**
- Real-robot experiments are **incremental and controlled**
- Optimization-based planning remains the **safety backbone**
- Learning and VLA components are **modular, replaceable, and comparable**

This roadmap ensures that meaningful results are obtained even if
learning-based components are only partially deployed on the real robot.
