# FSAI Simulator

Team-owned Formula Student AI simulator.
EUFS sim2 is a pinned read-only dependency.
This repository owns the vehicle plant, launch entry, configuration and tests.

The official runtime is Ubuntu 22.04 and ROS 2 Humble.
macOS can compile and test `fsai_sim_core` only.

## What runs the car

`fsai_sim_core` advances vehicle state.
`FsaiSimulationNode` is the only ROS composition entry.
Isaac Sim and Gazebo do not integrate the vehicle.

Default plant: dynamic bicycle, 1 ms internal step, 5 ms outer step, RK4.
Reference vehicle numbers are unvalidated EUFS placeholders.

## Ubuntu 22.04

```bash
git clone <your-repo> FSAI
cd FSAI
tools/bootstrap_ubuntu.sh
tools/build.sh
tools/test.sh
tools/run_sim.sh --vehicle reference_bicycle --track skidpad_small --scenario straight_acceleration
```

`tools/bootstrap_ubuntu.sh` imports locked EUFS checkouts, disables EUFS push, and installs Humble dependencies.
Do not push this tree to EUFS.

Compatibility mode still uses the original EUFS core:

```bash
ros2 launch fsai_bringup upstream_compatibility.launch.py run_mode:=as_fast_as_possible max_steps:=20
```

Team plant:

```bash
ros2 launch fsai_bringup simulator.launch.py vehicle:=reference_bicycle core_type:=fsai run_mode:=realtime
```

## macOS

ROS 2 Humble is not required here.
Portable plant tests:

```bash
cmake -S simulator/src/fsai_sim_core -B /tmp/fsai-sim-core-build -DFSAI_SIM_CORE_STANDALONE=ON -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build /tmp/fsai-sim-core-build -j8
ctest --test-dir /tmp/fsai-sim-core-build --output-on-failure
bash tools/tests/test_ci_contract.sh
bash tools/tests/test_dependency_contract.sh
bash tools/tests/test_entry_points.sh
```

Do not treat skipped ROS/colcon/Gazebo checks as passing.

## Layout

- `simulator/src/fsai_sim_core` ROS-free plant
- `simulator/src/fsai_sim2_adapter` composition node and EUFS adapter
- `simulator/src/fsai_bringup` launch, vehicles, tracks, scenarios
- `simulator/src/fsai_interfaces` ROS messages
- `simulator/src/fsai_description` visualization URDF
- `refs/` local upstream copies, not the runtime checkout
- `simulator/fsai_sim.repos` locked dependency revisions
