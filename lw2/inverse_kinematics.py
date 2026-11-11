"""Labwork 2 - algebraic inverse kinematics of the UFACTORY Lite 6, as a ROS 2 node.

THIS IS THE ONLY FILE YOU EDIT IN LABWORK 2.

Implement the following, replacing every ``raise NotImplementedError``:

    Part B - the maths (no ROS needed)
        inverse_kinematics(T_d)                    every distinct solution for a pose
        select_solution(solutions, q_current)      the solution that is executed

    Part B - the node
        IkNode                                     reads a desired pose, publishes
                                                   the selected joint angles

Run the node with ``ros2 run lite6_ik ik_node``.

``transform_from_pose``, ``JOINT_LIMITS``, the constants ``TOL_SAME`` and
``TOL_WRIST`` and ``main`` below are PROVIDED: already implemented, do not edit
them.

Keep the function and class names, arguments and return types exactly as they
are: the checker node (``ik_check``), the study program (``ik_study``), the
tests in ``test/`` and the instructor's grader all use them by name.

Conventions - those of Labwork 1:

    * Frame {0} coincides with the ROS frame ``link_base``; frame {6} with
      ``link_eef``. The desired pose T_d is the pose of {6} in {0}.
    * Joint angles in RADIANS, in the controller's convention, as published on
      /joint_states (q = 0 is the controller's zero configuration).
    * Lengths in METRES.

Allowed imports: ``numpy`` and ``math`` for the maths, ``forward_kinematics``
from your own ``lite6_kinematics.forward_kinematics`` (already imported below),
and ``rclpy``, ``sensor_msgs`` and ``geometry_msgs`` for the node (already
imported below). Nothing else - no MoveIt, no kinematics library, no URDF
parser, no tf. ``test/test_inverse_kinematics.py`` checks the imports.
"""

import math

import numpy as np

from lite6_kinematics.forward_kinematics import forward_kinematics

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:         # the maths and its unit tests also run without ROS
    rclpy = None
    Node = object

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'


# =============================================================================
# Part B - the maths
# =============================================================================
# Lengths of the Lite 6 (m): the same values as the modified DH table of
# Labwork 1 (forward_kinematics.py).
#   d1: base -> shoulder (Z1)   a2: shoulder -> elbow (X2)   a3: elbow offset (X3)
#   d4: elbow -> wrist (Z4)     d6: wrist -> link_eef (Z6)
D1 = 0.2435
A2 = 0.2002
A3 = 0.0870
D4 = 0.22761
D6 = 0.0625

L_FOREARM = math.hypot(A3, D4)      # elbow -> wrist centre, straight line
GAMMA = math.atan2(D4, A3)          # constant angle of that line in the elbow frame
REACH_TOL = 1e-9                    # rounding tolerance of the reach condition


def _wrap(a):
    """Angle in (-pi, pi]."""
    return -((-a + math.pi) % (2.0 * math.pi) - math.pi)


def _same(a, b):
    """True if no angle of a and b differs by more than TOL_SAME, modulo 2 pi."""
    return all(abs(_wrap(x - y)) <= TOL_SAME for x, y in zip(a, b))


def _R03(q1, q2, q3):
    """Rotation of {3} in {0} (from the DH table: depends on q1 and q2 - q3)."""
    c1, s1 = math.cos(q1), math.sin(q1)
    c23, s23 = math.cos(q2 - q3), math.sin(q2 - q3)
    return np.array([[c1 * c23, c1 * s23, s1],
                     [s1 * c23, s1 * s23, -c1],
                     [-s23, c23, 0.0]])


def inverse_kinematics(T_d) -> list:
    """Every distinct joint vector that puts the end-effector at T_d.

    The solution is closed-form, from the expressions of Part A: no iterative
    or optimisation method.

    Args:
        T_d: (4, 4) desired pose of {6} in {0}; translation in metres.

    Returns:
        A list of (6,) float arrays, joint angles in radians, order
        joint1..joint6, one array per distinct solution (in any order). The
        empty list if T_d is out of the robot's reach.

    Conventions of Part A:
        * every angle is in (-pi, pi]; the joint ranges JOINT_LIMITS are NOT
          applied here: a solution outside them is still returned;
        * two solutions in which no angle differs by more than TOL_SAME
          (angles compared modulo 2 pi) are the same solution: only one of
          them is returned;
        * if the angle between the axes of joints 4 and 6 is smaller than
          TOL_WRIST, those axes are aligned and every solution returned has
          q4 = 0.
    """
    T_d = np.asarray(T_d, dtype=float)
    R_d, p_d = T_d[:3, :3], T_d[:3, 3]

    # 1. Wrist centre (common origin of {4} and {5}): p_w = p_d - d6 z6.
    x, y, z = p_d - D6 * R_d[:, 2]

    # Position of the wrist centre from the DH table:
    #   x = r cos q1,  y = r sin q1
    #   r = a2 sin q2 + a3 cos(q2 - q3) - d4 sin(q2 - q3)
    #   s = z - d1 = a2 cos q2 - a3 sin(q2 - q3) - d4 cos(q2 - q3)
    s = z - D1
    solutions = []
    for sign in (1.0, -1.0):                    # 2. arm in front of / behind the base
        r = sign * math.hypot(x, y)
        q1 = math.atan2(sign * y, sign * x)     # x = y = 0: shoulder singularity, q1 = 0 (or pi)

        # 3. Shoulder: (r - a2 sin q2)^2 + (s - a2 cos q2)^2 = a3^2 + d4^2
        #    =>  r sin q2 + s cos q2 = K  =>  rho cos(q2 - beta) = K
        rho = math.hypot(r, s)
        if rho < 1e-12:
            continue
        K = (r * r + s * s + A2 * A2 - L_FOREARM ** 2) / (2.0 * A2)
        c = K / rho
        if abs(c) > 1.0 + REACH_TOL:            # REACH: the pose is out of reach
            return []
        c = max(-1.0, min(1.0, c))
        beta = math.atan2(r, s)
        for elbow in (1.0, -1.0):               # 4. elbow up / elbow down
            q2 = beta + elbow * math.acos(c)

            # 5. Elbow: a3 cos(phi) - d4 sin(phi) = r - a2 sin q2,
            #           a3 sin(phi) + d4 cos(phi) = a2 cos q2 - s,  phi = q2 - q3
            u = r - A2 * math.sin(q2)
            w = A2 * math.cos(q2) - s
            phi = math.atan2(w, u) - GAMMA
            q3 = q2 - phi

            # 6. Wrist: R36 = R03^T R_d, with
            #   R36[0,2] = -s5 c4,  R36[2,2] = -s5 s4,  R36[1,2] = -c5,
            #   R36[1,0] = -s5 c6,  R36[1,1] =  s5 s6
            M = _R03(q1, q2, q3).T @ R_d
            c5 = -M[1, 2]
            s5_abs = math.hypot(M[0, 2], M[2, 2])
            wrist_angle = math.atan2(s5_abs, c5)    # angle between the axes of joints 4 and 6
            if wrist_angle < TOL_WRIST:             # aligned: q5 = 0, only q4 + q6 is fixed
                wrists = [(0.0, 0.0, math.atan2(M[2, 0], M[0, 0]))]
            elif wrist_angle > math.pi - TOL_WRIST:  # opposite: q5 = pi, only q6 - q4 is fixed
                wrists = [(0.0, math.pi, math.atan2(M[2, 0], -M[0, 0]))]
            else:                                   # two wrists: q5 > 0 and q5 < 0
                wrists = []
                for s5 in (s5_abs, -s5_abs):
                    q4 = math.atan2(-M[2, 2] / s5, -M[0, 2] / s5)
                    q5 = math.atan2(s5, c5)
                    q6 = math.atan2(M[1, 1] / s5, -M[1, 0] / s5)
                    wrists.append((q4, q5, q6))

            for q4, q5, q6 in wrists:
                q = np.array([_wrap(v) for v in (q1, q2, q3, q4, q5, q6)])
                if not any(_same(q, other) for other in solutions):
                    solutions.append(q)
    return solutions


def select_solution(solutions, q_current):
    """The solution that is executed, chosen among `solutions`.

    Args:
        solutions: list of (6,) joint vectors (rad), as inverse_kinematics()
                   returns them. Their angles may be any real numbers: this
                   function must not assume they are in (-pi, pi].
        q_current: six joint angles (rad), the current configuration of the arm.

    Returns:
        A (6,) float array, the selected joint vector, or None if no solution
        is executable (no solutions at all, or all discarded). The inputs are
        not modified.

    The rule is the one of Part A: in each solution, every angle q_i is
    replaced by the value q_i + 2 pi k (k an integer) that lies inside the
    range of joint i (JOINT_LIMITS) and is closest to q_current[i]; a solution
    for which no such value exists is discarded. Among the solutions that
    remain, the one at the smallest euclidean distance from q_current is
    selected, and the returned vector is that solution with the replaced angles.
    """
    q_current = np.asarray(q_current, dtype=float).reshape(6)
    two_pi = 2.0 * math.pi
    best, best_dist = None, math.inf
    for sol in solutions:
        sol = np.asarray(sol, dtype=float).reshape(6)
        q = np.empty(6)
        ok = True
        for i in range(6):
            low, high = JOINT_LIMITS[i]
            k0 = round((q_current[i] - sol[i]) / two_pi)    # multiple of 2 pi closest to q_current
            candidates = [sol[i] + two_pi * k for k in (k0 - 2, k0 - 1, k0, k0 + 1, k0 + 2)
                          if low <= sol[i] + two_pi * k <= high]
            if not candidates:                              # no multiple of 2 pi inside the range
                ok = False
                break
            q[i] = min(candidates, key=lambda v: abs(v - q_current[i]))
        if not ok:
            continue
        dist = float(np.linalg.norm(q - q_current))
        if dist < best_dist:
            best, best_dist = q, dist
    return best


# =============================================================================
# Part B - the node
# =============================================================================
class IkNode(Node):
    """Solves the inverse kinematics of every desired pose it receives.

    Subscribes   target_topic         geometry_msgs/PoseStamped  (/lite6_ik/target)
                 joint_states_topic   sensor_msgs/JointState     (/joint_states)
    Publishes    solution_topic       sensor_msgs/JointState     (/lite6_ik/solution)

    The target is the pose of link_eef in link_base (header.frame_id is
    BASE_FRAME). For each target received, the node computes the solutions and
    selects one from the latest joint angles received. If there is a selected
    solution it publishes a JointState:
        name             JOINT_NAMES
        position         the selected solution, in the order of JOINT_NAMES
        header.stamp     the stamp of the TARGET it answers (copy it:
                         ik_check matches the solution to the target by it)
    Otherwise it publishes nothing and logs why:
        unreachable      the pose has no solution at all
        out_of_range     it has solutions, but all of them were discarded

    The topic names are constructor arguments so that the tests can run the
    node on private topics: use them, do not write the names again.
    """

    def __init__(self, target_topic='/lite6_ik/target', joint_states_topic='/joint_states',
                 solution_topic='/lite6_ik/solution'):
        super().__init__('ik_node')
        self.q_latest = None                                 # latest joint angles received
        self.pub = self.create_publisher(JointState, solution_topic, 10)
        self.create_subscription(JointState, joint_states_topic, self.on_joint_state, 10)
        self.create_subscription(PoseStamped, target_topic, self.on_target, 10)

    def on_joint_state(self, msg):
        """Called for every JointState: keep the latest joint angles."""
        # TODO (LW2, Part B)
        #   Pick the angles of JOINT_NAMES BY NAME from msg.name / msg.position:
        #   the order is not promised and the message may carry other joints.
        #   Store them; do nothing else here (this runs inside the spin, and
        #   joint states arrive hundreds of times per second).
        index = {name: i for i, name in enumerate(msg.name)}
        if all(name in index for name in JOINT_NAMES):
            self.q_latest = np.array([msg.position[index[name]] for name in JOINT_NAMES], dtype=float)

    def on_target(self, msg):
        """Called for every desired pose: solve it and publish the solution."""
        # TODO (LW2, Part B)
        #   1. if no joint states have arrived yet, log a warning and return
        #   2. T_d = transform_from_pose(msg)
        #   3. compute the solutions and select one from the latest angles,
        #      with the two functions above
        #   4. if there is a selected solution publish the JointState described
        #      in the class docstring; otherwise log unreachable or
        #      out_of_range (and do not publish)
        if self.q_latest is None:
            self.get_logger().warn('no joint states received yet: target ignored')
            return
        T_d = transform_from_pose(msg)
        solutions = inverse_kinematics(T_d)
        q = select_solution(solutions, self.q_latest.copy())
        if q is None:
            reason = 'unreachable' if not solutions else 'out_of_range'
            self.get_logger().warn(f'no solution published: {reason} ({len(solutions)} solutions)')
            return
        sol = JointState()
        sol.header.stamp = msg.header.stamp                  # stamp of the target it answers
        sol.name = list(JOINT_NAMES)
        sol.position = [float(v) for v in q]
        self.pub.publish(sol)
        self.get_logger().info(f'{len(solutions)} solutions, selected: '
                               f'{np.round(np.degrees(q), 2).tolist()} deg')


# =============================================================================
# PROVIDED - DO NOT EDIT. Already implemented; nothing below is yours to change.
# =============================================================================
TOL_SAME = 1e-6     # rad - two solutions closer than this in every angle are the same
TOL_WRIST = 1e-6    # rad - angle between the axes of joints 4 and 6 below which they are aligned

# Joint ranges (rad) of the xarm_description model of the Lite 6, one row per
# joint, [lower, upper].
JOINT_LIMITS = np.array([
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.61799, 2.61799],
    [-0.061087, math.pi * 0.99],
    [-math.pi * 0.99, math.pi * 0.99],
    [-2.1642, 2.1642],
    [-math.pi * 0.99, math.pi * 0.99],
])


def transform_from_pose(msg) -> np.ndarray:
    """(4, 4) transform of a geometry_msgs/PoseStamped.

    The quaternion is normalised first, so a slightly off unit quaternion still
    gives a rotation matrix.
    """
    p, o = msg.pose.position, msg.pose.orientation
    n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
    x, y, z, w = o.x / n, o.y / n, o.z / n, o.w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def main(args=None):
    """Entry point of ``ros2 run lite6_ik ik_node``."""
    if rclpy is None:
        raise SystemExit('ROS 2 is not available here: run this inside the container.')
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = IkNode()
        node.get_logger().info('ik_node running: desired poses in, joint angles out')
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'\n{e}\nImplement it in lite6_ik/inverse_kinematics.py first.')
        status = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status


if __name__ == '__main__':
    raise SystemExit(main())
