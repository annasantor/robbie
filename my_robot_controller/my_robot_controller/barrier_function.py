"""
barrier_function.py

Control Barrier Function for obstacle avoidance.

Safe set definition (keep OUTSIDE obstacle sphere):
    C = { p_ee : h(p_ee) >= 0 }
    h(p_ee) = ||p_ee - p_obstacle|| - r_safe

Supports multiple obstacles. Each obstacle has its own
BarrierFunction instance. The CBF filter enforces all
constraints simultaneously via a multi-constraint QP.

Current obstacles:
    1. TV    - static, defined in yaml
    2. Human - static sphere, defined in yaml
               (dynamic tracking identified as future work)
"""

import numpy as np


class BarrierFunction:
    """
    CBF for keeping end-effector OUTSIDE an obstacle sphere.

    Safe set C = { p_ee : h(p_ee) >= 0 }
    h(p_ee) = ||p_ee - p_obstacle|| - r_safe

    h >= 0  →  end-effector is outside the sphere (safe)
    h <  0  →  end-effector is inside the sphere (unsafe)

    Gradient is the unit vector pointing away from obstacle —
    nonzero everywhere except p_ee == p_obstacle (unreachable
    in practice if r_safe > 0).
    """

    def __init__(self, name: str, p_obstacle: list,
                 r_safe: float, gamma: float = 1.0):
        """
        Args:
            name:       obstacle name for logging (e.g. 'tv', 'human')
            p_obstacle: obstacle center [x, y, z] in base_link frame
            r_safe:     safety radius in meters (keep-away distance)
            gamma:      CBF decay rate
                        larger = more aggressive correction near boundary
                        smaller = smoother but triggers earlier
        """
        self.name       = name
        self.p_obstacle = np.array(p_obstacle, dtype=float)
        self.r_safe     = float(r_safe)
        self.gamma      = float(gamma)

    def h(self, p_ee: np.ndarray) -> float:
        """
        Barrier function value.
        h >= 0  →  safe (outside sphere)
        h <  0  →  unsafe (inside sphere)

        Args:
            p_ee: end-effector position [x, y, z], shape (3,)

        Returns:
            scalar barrier value
        """
        return float(np.linalg.norm(p_ee - self.p_obstacle) - self.r_safe)

    def grad_h(self, p_ee: np.ndarray) -> np.ndarray:
        """
        Gradient of h with respect to p_ee.
        Unit vector pointing away from obstacle center.

        Args:
            p_ee: end-effector position [x, y, z], shape (3,)

        Returns:
            grad: shape (3,) unit vector pointing away from obstacle
        """
        diff = p_ee - self.p_obstacle
        norm = np.linalg.norm(diff)
        if norm < 1e-6:
            raise RuntimeError(
                f'End-effector is at obstacle [{self.name}] center — '
                f'singular point. Increase r_safe or reposition obstacle.'
            )
        return diff / norm

    def is_safe(self, p_ee: np.ndarray) -> bool:
        """
        Returns True if end-effector is outside the obstacle sphere.

        Args:
            p_ee: end-effector position [x, y, z], shape (3,)
        """
        return self.h(p_ee) >= 0.0

    def cbf_condition(
        self,
        p_ee:  np.ndarray,
        q_dot: np.ndarray,
        Jv:    np.ndarray,
    ) -> float:
        """
        Evaluate the CBF condition:
            grad_h @ Jv @ q_dot + gamma * h >= 0

        Positive → trajectory is safe w.r.t. this obstacle
        Negative → CBF filter must intervene

        Args:
            p_ee:  end-effector position [x, y, z], shape (3,)
            q_dot: joint velocities, shape (6,)
            Jv:    linear Jacobian, shape (3, 6)

        Returns:
            scalar — positive means safe, negative means violation
        """
        h_val = self.h(p_ee)
        dh    = self.grad_h(p_ee)
        p_dot = Jv @ q_dot           # end-effector velocity (3,)
        Lfh   = float(dh @ p_dot)    # Lie derivative
        return Lfh + self.gamma * h_val

    def lgh(self, p_ee: np.ndarray, Jv: np.ndarray) -> np.ndarray:
        """
        Lgh = grad_h @ Jv — control influence on CBF condition.
        Used directly by the QP filter.

        Args:
            p_ee: end-effector position [x, y, z], shape (3,)
            Jv:   linear Jacobian, shape (3, 6)

        Returns:
            Lgh: shape (6,) — one row of the QP constraint matrix
        """
        return self.grad_h(p_ee) @ Jv

    def verify_regularity(self, p_ee: np.ndarray, tol: float = 1e-6) -> None:
        """
        Verify CBF regularity condition at boundary:
        grad_h must be nonzero when h ≈ 0.
        Call during setup to sanity-check parameters.

        Args:
            p_ee: end-effector position near boundary
            tol:  tolerance for boundary and gradient checks
        """
        if abs(self.h(p_ee)) < tol:
            grad_norm = np.linalg.norm(self.grad_h(p_ee))
            if grad_norm < tol:
                raise RuntimeError(
                    f'CBF regularity condition violated at boundary '
                    f'for obstacle [{self.name}]: grad_h is zero.'
                )