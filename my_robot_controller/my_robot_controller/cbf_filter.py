"""
cbf_filter.py

CBF safety filter using analytical QP solution.

For N obstacle constraints, the QP is:
    min  ||u - u_nom||^2
    s.t. Lgh_i @ u >= -gamma_i * h_i - Lfh_i   for i = 1..N

With N constraints we use cvxpy/OSQP for correctness and generality.
For N=1 an analytical solution exists — kept as fallback.

Obstacles:
    1. TV    — static sphere
    2. Human — static sphere (dynamic tracking: future work)
"""

import numpy as np
import cvxpy as cp
from my_robot_controller.barrier_function import BarrierFunction


class CBFFilter:
    """
    Applies CBF safety filter to a nominal control input.

    Takes u_nom (PID output, joint position increments),
    checks all CBF constraints, and returns the minimum-norm
    safe correction u_safe.

    If all constraints are already satisfied, u_safe = u_nom.
    """

    def __init__(self, barriers: list[BarrierFunction]):
        """
        Args:
            barriers: list of BarrierFunction instances (one per obstacle)
        """
        self._barriers  = barriers
        self._n_joints  = 6
        self._n_constraints = len(barriers)

    def filter(
        self,
        u_nom:  np.ndarray,
        p_ee:   np.ndarray,
        q_dot:  np.ndarray,
        Jv:     np.ndarray,
    ) -> tuple[np.ndarray, list[float], bool]:
        """
        Apply CBF filter to nominal control input.

        Args:
            u_nom:  nominal joint position commands, shape (6,)
                    output of PID controller
            p_ee:   current end-effector position [x,y,z], shape (3,)
            q_dot:  current joint velocities, shape (6,)
            Jv:     linear Jacobian, shape (3, 6)

        Returns:
            u_safe:   filtered safe joint commands, shape (6,)
            h_values: list of barrier values [h1, h2, ...]
                      for logging and visualization
            modified: True if CBF had to modify u_nom
        """
        # Evaluate all barrier values
        h_values = [b.h(p_ee) for b in self._barriers]

        # Check if all constraints already satisfied
        all_safe = True
        for i, b in enumerate(self._barriers):
            cond = b.cbf_condition(p_ee, q_dot, Jv)
            if cond < 0:
                all_safe = False
                break

        if all_safe:
            return u_nom.copy(), h_values, False

        # -- QP: min ||u - u_nom||^2 s.t. CBF constraints --
        u = cp.Variable(self._n_joints)

        # Build constraint list
        constraints = []
        for b in self._barriers:
            h_val = b.h(p_ee)
            dh    = b.grad_h(p_ee)
            p_dot = Jv @ q_dot
            Lfh   = float(dh @ p_dot)
            Lgh   = dh @ Jv            # shape (6,)

            # CBF constraint: Lfh + Lgh @ u >= -gamma * h
            constraints.append(
                Lfh + Lgh @ u >= -b.gamma * h_val
            )

        objective = cp.Minimize(cp.sum_squares(u - u_nom))
        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.OSQP, warm_starting=True)

        if prob.status in ('optimal', 'optimal_inaccurate') and u.value is not None:
            return u.value, h_values, True
        else:
            # QP failed — return nominal and log warning
            # Caller should log this
            return u_nom.copy(), h_values, False

    def all_safe(self, p_ee: np.ndarray) -> bool:
        """
        Quick safety check without computing filter.
        Returns True if p_ee satisfies all barrier constraints.
        """
        return all(b.is_safe(p_ee) for b in self._barriers)

    @property
    def barrier_names(self) -> list[str]:
        return [b.name for b in self._barriers]
