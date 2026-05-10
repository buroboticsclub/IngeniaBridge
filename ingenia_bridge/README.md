# ingenia_bridge

ROS 2 per-joint bridge for Ingenia Everest EVS-XCR-C CANopen drives.
Designed to plug into the same orchestrator (`epos2_arm_controller`)
that drives the Maxon EPOS2 and Copley APV/APZ joints, so a
mixed-vendor arm just works.

## Joint assignment

| Joint  | Drive                    | CAN Node ID | Bridge                 |
|--------|--------------------------|-------------|------------------------|
| joint0 | Ingenia EVS-XCR-C        | 10          | `ingenia_joint_bridge` |

(Joint0 was previously planned as a Copley APZ at node 1; the Ingenia
drive replaces it. The Copley package is still available for the
APZ-based axes if/when those return.)

## Wire protocol (matches `copley_bridge` / `epos2_bridge`)

Per joint `jN`, the bridge offers:

**Services**
- `/ingenia/jN/clear_fault` — `std_srvs/Trigger`
- `/ingenia/jN/arm_ipm` — `std_srvs/Trigger` (full CiA 402 + IPM bring-up)
- `/ingenia/jN/disarm_ipm` — `std_srvs/Trigger`
- `/ingenia/jN/move_absolute_timed` — `epos2_bridge_interfaces/MoveAbsoluteTimed`

**Subscriptions**
- `/ingenia/jN/reduced_traj` — `std_msgs/Float64MultiArray` in `[q, v, dt, q, v, dt, ...]` format

**Publications**
- `/joint_states` — `sensor_msgs/JointState`
- `/ingenia/jN/fault` — `std_msgs/Bool`

**Action server**
- `/jointN_position_controller/follow_joint_trajectory` — `control_msgs/FollowJointTrajectory`

## Why this bridge looks different from `copley_bridge`

Copley packs an entire IPM segment (position + velocity + time) into a
single 64-bit object at `0x2010`, so one CAN frame = one segment.
Ingenia exposes the IPM record as **three separate manufacturer
objects** in the OD:

| Index    | Name                                             | Type   |
|----------|--------------------------------------------------|--------|
| `0x2181` | Interpolation data record - Position input       | int32  |
| `0x2182` | Interpolation data record - Velocity input       | float32 (rev/s) |
| `0x2183` | Interpolation data record - Time input           | uint32 |
| `0x2184` | Interpolation data record integrity check        | uint32 |
| `0x2185` | Interpolation data record status                 | uint32 |

That's 12 bytes per segment, so we use **two RPDOs** per segment:

- **RPDO1** stages position + time (8 bytes)
- **RPDO3** writes velocity + the integrity-check field, which commits
  the staged segment into the buffer (8 bytes)

The integrity-check protocol on `0x2184` isn't in the public docs we
have. The bridge exposes a `integrity_check_mode` parameter with three
strategies (`counter`, `constant`, `echo_pos`) so the right one can be
picked empirically by watching the `0x2185` status word respond on the
bench.

## Bring-up

### One-time per drive

1. Configure the drive in MotionLab (motor params, current limits, PID
   gains, encoder type / resolution). Save to flash.
2. Set the drive's CAN node ID to 10 and save to flash.
3. **Map PDOs.** The drive ships with the standard Ingenia layout
   (RPDO3 = ctrlword + target position, etc.) which is *not* what this
   bridge expects. With a CANopen master node already up on the bus:

   ```bash
   bash "$(ros2 pkg prefix ingenia_bridge)/share/ingenia_bridge/scripts/apply_ingenia_pdo_remap_one.sh" 10
   ```

   Then save to flash so the mapping persists across power cycles
   (the script prints the exact `ros2 service call` to do this).

### Every boot

```bash
ros2 launch ingenia_bridge ingenia_j0_bridge.launch.py
```

## Required configuration

The bridge **refuses to start** if these aren't overridden in the YAML:

- `encoder_qc_per_motor_rev` — encoder counts per motor revolution.
  For the stock EVS-XCR-C with a 17-bit absolute single-turn encoder
  this is **131072**. The sample YAML pre-fills this value.
- `gear_ratio_motor_per_joint_rev` — motor revolutions per joint
  revolution.

Optional but commonly tuned:

- `sign` — set `-1.0` if positive joint motion produces negative
  encoder counts.
- `integrity_check_mode` — see above; default `"counter"`.
- `time_units_per_second` — must match the drive's `0x60C2` setting.
  Default `1000.0` for the stock 1 ms time period.

## Bench bring-up checklist

1. CAN bus up at the same bitrate as the drive (`ip link set can0 up
   type can bitrate 1000000` -- confirm bitrate against MotionLab).
2. Run `apply_ingenia_pdo_remap_one.sh 10`.
3. Launch the bridge with `enable_on_startup: false` and watch
   `/joint_states` to confirm position telemetry tracks the encoder
   when you hand-rotate the shaft (drive disarmed and shaft free).
4. Call `/ingenia/j0/arm_ipm` and watch the log for the post-arm
   `0x2185 record_status` value:
   - `0x00000000` → integrity protocol is correct, you're done.
   - non-zero → re-launch with a different `integrity_check_mode`
     and try again. `candump can0` in a side terminal lets you see
     RPDO1/RPDO3 traffic and TPDO1 status replies.
5. Call `/ingenia/j0/move_absolute_timed` with a small target
   (e.g. 0.05 rad over 1.0 s) for the first motion test.
