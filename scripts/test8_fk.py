"""Teste 8: cinematica direta do controlador para a Pose A (e B) do teste 5."""
import math

import yaml
from xarm.wrapper import XArmAPI

cfg = yaml.safe_load(open("config/robot.yaml"))
arm = XArmAPI(cfg["ip"], check_joint_limit=False)

# angulos do /joint_states no teste 5, em radianos
poses = {
    "A": [0, 0, 0, 0, 0, 0],
    "B": [0.5, 0.3, 0.8, 0.0, 0.5, 0.0],
}
for nome, q_rad in poses.items():
    q_deg = [math.degrees(q) for q in q_rad]
    code, pose = arm.get_forward_kinematics(q_deg, input_is_radian=False,
                                            return_is_radian=False)
    print(f"Pose {nome}: codigo {code}")
    print("   x, y, z (mm):", [round(v, 3) for v in pose[:3]])
    print("   roll, pitch, yaw (graus):", [round(v, 3) for v in pose[3:]])
arm.disconnect()
