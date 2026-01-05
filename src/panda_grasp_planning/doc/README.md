# Documentation Overview

## 📖 Start Here

This folder contains all project documentation for the Franka Panda Grasp & Sort Pipeline.

**New to this project?** → Start with [INDEX.md](INDEX.md) for guided navigation.

**Want to run the demo?** → See [QUICK_START.md](QUICK_START.md)

**Need current status?** → Check [PHASE_1S_STATUS.md](PHASE_1S_STATUS.md) or [PHASE_1S_COMPLETE.md](PHASE_1S_COMPLETE.md)

**Planning Phase 2S?** → Read [PHASE_2S_PLANNING.md](PHASE_2S_PLANNING.md)

**Understanding the roadmap?** → See [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md)

---

## 📋 All Documents

| File | Purpose | Status |
|------|---------|--------|
| **INDEX.md** | Navigation guide | ✅ Current |
| **DEVELOPMENT_ROADMAP.md** | Project phases 0-3 | ✅ Up-to-date |
| **PHASE_1S_STATUS.md** | Baseline implementation details | ✅ Complete |
| **PHASE_1S_COMPLETE.md** | Phase 1S summary & checklist | ✅ Final |
| **PHASE_2S_PLANNING.md** | Phase 2S design & roadmap | ✅ Ready |
| **QUICK_START.md** | How to run the demo | ✅ Current |
| **IMPROVEMENTS_V3.md** | V3→V4 bug fixes (historical) | 📖 Reference |
| **VISION_SETUP.md** | ZED2 camera integration (Phase 1R) | 📖 Reference |
| **PHASE_0S_ASSESSMENT.md** | Initial assessment (historical) | 📖 Reference |
| **FILE_REORGANIZATION.md** | Old folder proposals | 🗑️ Deprecated |
| **PHASE_1S_PLACE_TO_BIN.md** | Superseded by STATUS.md | 🗑️ Deprecated |

---

## 🎯 Current Project Status

- **Phase 1S (Baseline)**: ✅ COMPLETE (100% grasp success, 20/20 trials)
- **Phase 1R (Real Robot)**: ⚠️ PENDING (optional, can defer)
- **Phase 2S (VLA)**: 📋 PLANNED (ready to begin)
- **Phase 3S (Multi-obj)**: 🔮 FUTURE

---

## 🚀 Quick Commands

```bash
# Navigate to demo location
cd /opt/ros_ws

# Start Gazebo simulation
roslaunch panda_grasp_complete.launch

# Run demo (in new terminal)
rosrun panda_grasp_planning v4_demo.py --trials=20

# Check results
cat src/panda_grasp_planning/test_results/v4_demo_*.csv | tail -5
```

See [QUICK_START.md](QUICK_START.md) for detailed instructions.

---

## 🔍 Key Implementation Files

- **Demo Script**: `/opt/ros_ws/src/panda_grasp_planning/scripts/v4_demo.py` (production baseline)
- **Sorting Logic**: `/opt/ros_ws/src/panda_grasp_planning/modules/sorting/sorting_state_machine.py`
- **Test Results**: `/opt/ros_ws/src/panda_grasp_planning/test_results/` (latest CSV files)

---

## 📞 Need Help?

1. **Confused about project scope?** → [DEVELOPMENT_ROADMAP.md](DEVELOPMENT_ROADMAP.md)
2. **Want to understand V4 improvements?** → [PHASE_1S_STATUS.md](PHASE_1S_STATUS.md#-major-v4-improvements-over-v3)
3. **Ready to try Phase 2S?** → [PHASE_2S_PLANNING.md](PHASE_2S_PLANNING.md)
4. **Having environment issues?** → [QUICK_START.md](QUICK_START.md#troubleshooting)
5. **Want to know all bugs that were fixed?** → [IMPROVEMENTS_V3.md](IMPROVEMENTS_V3.md)

---

## 📊 Phase 1S Highlights

- ✅ **Grasp Success Rate**: 100% (20/20 trials)
- ✅ **Cycle Time**: ~15 seconds per trial
- ✅ **Table Safety**: Zero collisions
- ✅ **Color Sorting**: RED/YELLOW→BIN_1, BLUE→BIN_2, GREEN→BIN_3
- ✅ **Gripper Feedback**: Real-time width tracking
- ✅ **Edge Handling**: Microadjustment for off-center grasps

---

**Last Updated**: 2026-01-04  
**Author**: GitHub Copilot  
**Status**: Ready for Phase 2S planning

