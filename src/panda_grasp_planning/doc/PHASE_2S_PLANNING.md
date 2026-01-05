# Phase 2S Planning - VLA Integration with Grasp Baseline

**Status**: Planning Phase  
**Baseline Dependency**: Phase 1S (v4_demo.py) ✅ READY  
**Target**: Demonstrable VLA-based decision-making

---

## 📋 Problem Statement

The Phase 1S baseline achieves 100% grasp success with color-based sorting. Now we need to add **semantic understanding** via VLA (Vision-Language-Action) model to enable language-guided object selection.

### What Changes?

| Aspect | Phase 1S | Phase 2S |
|--------|----------|---------|
| **Object Selection** | Color-based (fixed mapping) | Language-based (VLA decision) |
| **Scene Input** | Metadata only | RGB image + language instruction |
| **Decision Source** | SortingStateMachine | VLA model output |
| **Motion Execution** | Same (V4 pipeline) | Same (V4 pipeline) |

**Key Insight**: VLA makes *decisions* about WHAT to pick, not HOW to pick it. The V4 baseline handles all motion execution.

---

## 🎯 Core Requirements

### Input Specification

**VLA receives**:
1. **RGB Image** (960×720, from simulator or camera)
2. **Grasp Candidates** (list of 5-10 candidate poses from V4 pipeline)
3. **Language Instruction** (text string, e.g., "pick the red cube")

### Output Specification

**VLA produces**:
1. **Object ID** (which cube in the scene to pick)
2. **Grasp Candidate Rank** (optional: which grasp pose to prefer)
3. **Confidence Score** (0-1, for logging)

### Demo Success Criteria

- ✅ At least 3 distinct instruction categories work
  - Example 1: Color ("pick red", "pick blue")
  - Example 2: Size ("pick big", "pick small")
  - Example 3: Spatial ("pick left", "pick right")
- ✅ Repeatability: ≥80% success across 10 fixed-scene trials
- ✅ Safety: No invalid outputs (no out-of-reach grasps)
- ✅ One-click demo script ready

---

## 🛠️ Integration Architecture

```
┌─────────────────────────────────────┐
│   Language Instruction (text)       │
├─────────────────────────────────────┤
│   VLA Adapter Interface             │
│   - Receives RGB, candidates, text  │
│   - Calls VLA model                 │
│   - Returns object_id + grasp_idx   │
├─────────────────────────────────────┤
│   v4_demo.py (Phase 1S baseline)    │
│   - Spawn colored cubes             │
│   - Compute grasp candidates        │
│   - Execute VLA-selected grasp      │
│   - Place to appropriate bin        │
└─────────────────────────────────────┘
```

### Files to Create/Modify

| File | Status | Purpose |
|------|--------|---------|
| `vla_adapter.py` | NEW | VLA model inference interface |
| `vla_demo.py` | NEW | Demo script with language instructions |
| `v4_demo.py` | MODIFY | Add VLA decision point before execution |
| `test_vla_integration.py` | NEW | Validation and metrics collection |

---

## 📊 Data Collection Strategy

### Training Data Source
- Generate from Phase 1S baseline execution
- Capture RGB at each grasp attempt
- Label with: [image, object_id, instruction_type, success]
- Target: 100-200 labeled examples per instruction category

### Instruction Categories (Proposal)

**Color-based** (3 categories):
- "pick the red cube"
- "pick the blue cube"  
- "pick the green cube"

**Size-based** (2 categories, requires object variation):
- "pick the large cube"
- "pick the small cube"

**Spatial-based** (2 categories):
- "pick the cube on the left"
- "pick the cube on the right"

### Label Format
```json
{
  "image_path": "path/to/rgb.png",
  "instruction": "pick the red cube",
  "object_id": 2,
  "grasp_candidates": [
    {"pose": [x,y,z,qx,qy,qz,qw], "success": true},
    ...
  ],
  "selected_grasp_idx": 0,
  "trial_result": "success"
}
```

---

## 🤖 VLA Model Options

### Option A: Lightweight Vision-Language (Recommended for demo)
- **Model**: CLIP + small MLP
- **Inputs**: RGB feature (CLIP) + text embedding (CLIP) + pose embeddings
- **Output**: logits over grasp candidates
- **Pros**: Fast, interpretable, small model
- **Cons**: Requires training on grasp data

### Option B: Pre-trained VLM (Zero-shot)
- **Model**: GPT-4V or LLaVA
- **Approach**: Describe scene, ask "which object matches instruction?"
- **Pros**: Zero-shot, no training required
- **Cons**: Slower, may require API calls, less controllable

### Option C: Hybrid (Best for robustness)
- **Model**: CLIP for ranking + LLM for verification
- **Approach**: CLIP scores candidates, LLM confirms consistency
- **Pros**: Robust, interpretable
- **Cons**: More complex pipeline

**Recommendation for Phase 2S Demo**: Start with **Option A** (CLIP + MLP)

---

## 📝 Implementation Roadmap

### Step 1: Data Collection (1-2 days)
- Modify v4_demo.py to save RGB images and metadata
- Run 10-15 trials with diverse cube positions
- Create labeled dataset

### Step 2: Model Training (1-2 days)
- Implement CLIP-based feature extraction
- Train lightweight MLP on grasp ranking task
- Validate on held-out test set

### Step 3: VLA Adapter Integration (1 day)
- Create vla_adapter.py with inference interface
- Implement grasp candidate ranking
- Add confidence scoring

### Step 4: Demo Integration (1 day)
- Create vla_demo.py with language instruction loop
- Integrate with v4_demo.py execution
- Test 3 instruction categories

### Step 5: Validation & Documentation (1 day)
- Run ≥80% success test suite
- Document demo procedure
- Create one-click launch script

**Estimated Timeline**: 5-7 days (can overlap with other work)

---

## 🔍 Metrics to Track

| Metric | Target | Rationale |
|--------|--------|-----------|
| **Overall Success** | ≥80% | Reliable demo |
| **Per-instruction Success** | ≥75% each | Balanced performance |
| **Model Inference Time** | <500ms | Real-time feel |
| **False Positives** | <10% | Safety critical |
| **Confidence Accuracy** | Ranked correlation | Calibration check |

### Test Suite

```python
# For each of 3 instruction categories:
for instruction in ["red", "blue", "green"]:
  for trial in range(10):
    spawn_mixed_cubes()
    result = vla_demo.execute_instruction(instruction)
    metrics.log(result)
  assert success_rate >= 0.80
```

---

## 🚀 Success Criteria (Phase 2S Exit Gates)

- [ ] VLA model trained and validated
- [ ] vla_adapter.py integration complete
- [ ] vla_demo.py runs 3 distinct instruction categories
- [ ] ≥80% success rate across test suite
- [ ] Inference time <500ms per decision
- [ ] One-click demo script ready
- [ ] All results logged to CSV
- [ ] Documentation complete

**When all gates pass → Phase 2S demo ready for review**

---

## 🔗 Dependencies

**Hard Dependencies**:
- Phase 1S baseline (v4_demo.py) ✅ Ready
- Gazebo simulation + MoveIt ✅ Ready
- ROS environment ✅ Ready

**Soft Dependencies**:
- CLIP model (can be downloaded on-demand)
- PyTorch (already installed in dev container)
- OpenCV (for image preprocessing)

---

## 📞 Related Files

- **Baseline**: `/opt/ros_ws/src/panda_grasp_planning/scripts/v4_demo.py`
- **Sorting Logic**: `/opt/ros_ws/src/panda_grasp_planning/modules/sorting/sorting_state_machine.py`
- **Phase 1S Docs**: `/opt/ros_ws/src/panda_grasp_planning/doc/PHASE_1S_STATUS.md`
- **Roadmap**: `/opt/ros_ws/src/panda_grasp_planning/doc/DEVELOPMENT_ROADMAP.md`

---

**Next Action**: Review this plan and approve scope before beginning implementation.

Created: 2026-01-04  
Author: GitHub Copilot

