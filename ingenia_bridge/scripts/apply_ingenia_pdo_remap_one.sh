#!/usr/bin/env bash
# Configure the PDOs needed by ingenia_joint_bridge on an Ingenia
# Everest EVS-XCR-C drive.
#
# Layout (vs. the Copley/Maxon "one packed segment" approach, the
# Ingenia drive splits the IPM record into three separate OD entries,
# so we need TWO RPDOs to send a segment, plus a third PDO that
# carries the integrity-check write that commits it):
#
#   RPDO1  ->  0x2181 IPM position input (i32) + 0x2183 IPM time input (u32)
#   RPDO2  ->  0x6040 controlword (16) + 0x6060 mode of operation (8)
#   RPDO3  ->  0x2182 IPM velocity input (f32) + 0x2184 IPM integrity check (u32)
#   TPDO1  ->  0x2185 IPM record status (32) + 0x6041 statusword (16) + 0x6061 mode display (8)
#   TPDO2  ->  0x6064 position actual (32) + 0x606C velocity actual (32)
#
# Prerequisites:
#   - ROS 2 (Jazzy) sourced; ros2_canopen master node running and exposing
#     /node_${NODE_ID}/sdo_write
#   - can-utils installed (for cansend NMT control)
#   - The drive is on the bus and at the expected NODE_ID
#
# Usage: ./apply_ingenia_pdo_remap_one.sh NODE_ID

set -eo pipefail

NODE_ID="${1:?usage: $0 NODE_ID}"
CAN_IFACE="${CAN_IFACE:-can0}"

write() {
  local idx="$1"
  local sub="$2"
  local data="$3"
  ros2 service call "/node_${NODE_ID}/sdo_write" canopen_interfaces/srv/COWrite \
       "{index: ${idx}, subindex: ${sub}, data: ${data}}"
}

RPDO1=$((0x200 + NODE_ID))
RPDO2=$((0x300 + NODE_ID))
RPDO3=$((0x400 + NODE_ID))
TPDO1=$((0x180 + NODE_ID))
TPDO2=$((0x280 + NODE_ID))

RPDO1_DISABLE=$((0x80000000 + RPDO1))
RPDO2_DISABLE=$((0x80000000 + RPDO2))
RPDO3_DISABLE=$((0x80000000 + RPDO3))
TPDO1_DISABLE=$((0xC0000000 + TPDO1))
TPDO2_DISABLE=$((0xC0000000 + TPDO2))

echo "Node ${NODE_ID}: entering Pre-Operational"
cansend "${CAN_IFACE}" 000#80$(printf "%02X" ${NODE_ID})

echo "Node ${NODE_ID}: disabling PDOs"
write 5120 1 "${RPDO1_DISABLE}"      # 0x1400 RPDO1 comm
write 5121 1 "${RPDO2_DISABLE}"      # 0x1401 RPDO2 comm
write 5122 1 "${RPDO3_DISABLE}"      # 0x1402 RPDO3 comm
write 6144 1 "${TPDO1_DISABLE}"      # 0x1800 TPDO1 comm
write 6145 1 "${TPDO2_DISABLE}"      # 0x1801 TPDO2 comm

echo "Node ${NODE_ID}: mapping RPDO1 = 0x2181 (pos i32) + 0x2183 (time u32)"
write 5632 0 0                       # 0x1600 sub0 = 0 (disable mapping)
write 5632 1 562102304               # 0x21810020
write 5632 2 562233376               # 0x21830020
write 5632 0 2                       # two mapped objects

echo "Node ${NODE_ID}: mapping RPDO2 = 0x6040 ctrlword + 0x6060 mode"
write 5633 0 0                       # 0x1601
write 5633 1 1614807056              # 0x60400010
write 5633 2 1616904200              # 0x60600008
write 5633 0 2

echo "Node ${NODE_ID}: mapping RPDO3 = 0x2182 (vel f32) + 0x2184 (integrity u32)"
write 5634 0 0                       # 0x1602
write 5634 1 562167840               # 0x21820020
write 5634 2 562298912               # 0x21840020
write 5634 0 2

echo "Node ${NODE_ID}: mapping TPDO1 = 0x2185 + 0x6041 + 0x6061"
write 6656 0 0                       # 0x1A00
write 6656 1 562364448               # 0x21850020
write 6656 2 1614872592              # 0x60410010
write 6656 3 1616969736              # 0x60610008
write 6656 0 3

echo "Node ${NODE_ID}: mapping TPDO2 = 0x6064 + 0x606C"
write 6657 0 0                       # 0x1A01
write 6657 1 1617166368              # 0x60640020
write 6657 2 1617690656              # 0x606C0020
write 6657 0 2

echo "Node ${NODE_ID}: re-enabling PDOs"
write 5120 1 "${RPDO1}"
write 5120 2 255                     # asynchronous (event-driven)
write 5121 1 "${RPDO2}"
write 5121 2 255
write 5122 1 "${RPDO3}"
write 5122 2 255
write 6144 1 $((0x40000000 + TPDO1))
write 6144 2 255
write 6145 1 $((0x40000000 + TPDO2))
write 6145 2 255

echo "Node ${NODE_ID}: back to Operational"
cansend "${CAN_IFACE}" 000#01$(printf "%02X" ${NODE_ID})

echo "Node ${NODE_ID}: done"
echo
echo "Reminder: the new mapping needs to be saved to flash if you want it"
echo "to persist across power cycles. Save via 0x1010:01 = 0x65766173 ('save')"
echo "in MotionLab, or:"
echo "  ros2 service call /node_${NODE_ID}/sdo_write canopen_interfaces/srv/COWrite \\"
echo "    \"{index: 4112, subindex: 1, data: 1702257011}\""
