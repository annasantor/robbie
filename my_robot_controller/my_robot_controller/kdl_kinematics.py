"""
kdl_kinematics.py

Provides forward kinematics and Jacobian computation for the arm
using PyKDL and urdf_parser_py — no kdl_parser_py dependency.

Chain: base_link → link_5 (end effector parent link per SRDF)
Joints: shoulder_roll, shoulder_pitch, elbow_roll,
        elbow_pitch, wrist_roll, wrist_pitch
"""

import numpy as np
import PyKDL
from urdf_parser_py.urdf import URDF


class KDLKinematics:
    """
    Wraps PyKDL chain FK and Jacobian solvers.
    Builds KDL chain manually from urdf_parser_py URDF.
    Initialized once from the robot_description string.
    """

    JOINT_NAMES = [
        'shoulder_roll',
        'shoulder_pitch',
        'elbow_roll',
        'elbow_pitch',
        'wrist_roll',
        'wrist_pitch',
    ]

    BASE_LINK = 'base_link'
    TIP_LINK  = 'link_5'

    def __init__(self, robot_description: str):
        """
        Args:
            robot_description: URDF XML string from /robot_description parameter
        """
        self._robot = URDF.from_xml_string(robot_description)
        self._chain = PyKDL.Chain()
        self._build_chain(self.BASE_LINK, self.TIP_LINK)
        self._n_joints = self._chain.getNrOfJoints()

        if self._n_joints != 6:
            raise RuntimeError(
                f'Expected 6 joints in chain {self.BASE_LINK}→{self.TIP_LINK}, '
                f'got {self._n_joints}. Check link names in SRDF.'
            )

        self._fk_solver  = PyKDL.ChainFkSolverPos_recursive(self._chain)
        self._jac_solver = PyKDL.ChainJntToJacSolver(self._chain)

    def _build_chain(self, base: str, tip: str):
        """
        Walk the URDF from base to tip and build a KDL chain.
        """
        # Build child_link → joint map
        link_to_joint = {}
        for joint in self._robot.joints:
            link_to_joint[joint.child] = joint

        # Trace path from tip back to base
        path = []
        current = tip
        while current != base:
            if current not in link_to_joint:
                raise RuntimeError(
                    f'Could not trace chain from {base} to {tip}. '
                    f'Link {current} has no parent joint.'
                )
            joint = link_to_joint[current]
            path.append(joint)
            current = joint.parent
        path.reverse()

        # Build KDL segments
        for joint in path:
            # Origin transform
            if joint.origin is not None:
                xyz = joint.origin.xyz or [0.0, 0.0, 0.0]
                rpy = joint.origin.rpy or [0.0, 0.0, 0.0]
                frame = PyKDL.Frame(
                    PyKDL.Rotation.RPY(rpy[0], rpy[1], rpy[2]),
                    PyKDL.Vector(xyz[0], xyz[1], xyz[2])
                )
            else:
                frame = PyKDL.Frame.Identity()

            # Joint type — use constructor 4:
            # PyKDL.Joint(name, origin, axis, type)
            if joint.joint_type in ('revolute', 'continuous'):
                axis = joint.axis or [0.0, 0.0, 1.0]
                kdl_joint = PyKDL.Joint(
                    joint.name,
                    PyKDL.Vector(0.0, 0.0, 0.0),   # joint origin (at segment frame)
                    PyKDL.Vector(axis[0], axis[1], axis[2]),
                    PyKDL.Joint.RotAxis
                )
            elif joint.joint_type == 'prismatic':
                axis = joint.axis or [0.0, 0.0, 1.0]
                kdl_joint = PyKDL.Joint(
                    joint.name,
                    PyKDL.Vector(0.0, 0.0, 0.0),
                    PyKDL.Vector(axis[0], axis[1], axis[2]),
                    PyKDL.Joint.TransAxis
                )
            else:
                # fixed joint
                kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

            segment = PyKDL.Segment(joint.child, kdl_joint, frame)
            self._chain.addSegment(segment)

    def fk(self, q: np.ndarray) -> np.ndarray:
        """
        Forward kinematics: joint angles → end-effector position.

        Args:
            q: joint angles in radians, shape (6,), in JOINT_NAMES order

        Returns:
            p_ee: end-effector position [x, y, z] in base_link frame
        """
        if len(q) != self._n_joints:
            raise ValueError(
                f'Expected {self._n_joints} joint angles, got {len(q)}'
            )

        kdl_q = PyKDL.JntArray(self._n_joints)
        for i, qi in enumerate(q):
            kdl_q[i] = float(qi)

        frame = PyKDL.Frame()
        ret = self._fk_solver.JntToCart(kdl_q, frame)
        if ret < 0:
            raise RuntimeError(f'FK solver failed with code {ret}')

        return np.array([frame.p.x(), frame.p.y(), frame.p.z()])

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Geometric Jacobian at current joint configuration.

        Args:
            q: joint angles in radians, shape (6,), in JOINT_NAMES order

        Returns:
            J: Jacobian matrix, shape (6, 6)
        """
        if len(q) != self._n_joints:
            raise ValueError(
                f'Expected {self._n_joints} joint angles, got {len(q)}'
            )

        kdl_q = PyKDL.JntArray(self._n_joints)
        for i, qi in enumerate(q):
            kdl_q[i] = float(qi)

        kdl_jac = PyKDL.Jacobian(self._n_joints)
        ret = self._jac_solver.JntToJac(kdl_q, kdl_jac)
        if ret < 0:
            raise RuntimeError(f'Jacobian solver failed with code {ret}')

        J = np.zeros((6, self._n_joints))
        for i in range(6):
            for j in range(self._n_joints):
                J[i, j] = kdl_jac[i, j]

        return J

    def jacobian_linear(self, q: np.ndarray) -> np.ndarray:
        """
        Linear (translational) part of Jacobian only.

        Returns:
            Jv: shape (3, 6)
        """
        return self.jacobian(q)[:3, :]