import numpy as np

from inverted_pendulum_control.model import linear_model, discretize
from inverted_pendulum_control.mpc import MPCController


DT = 0.02


def make_controller(N=23):
    A, B = linear_model()
    Ad, Bd = discretize(A, B, DT)
    Q = np.diag([10.0, 1.0, 100.0, 1.0])
    R = np.array([[0.1]])
    return Ad, Bd, MPCController(Ad, Bd, Q, R, N)


def test_zero_state_gives_zero_force():
    _, _, mpc = make_controller()
    assert abs(mpc.solve(np.zeros((4, 1)))) < 1e-6


def test_closed_loop_stabilizes_tilted_pendulum():
    Ad, Bd, mpc = make_controller()
    x = np.array([[0.0], [0.0], [0.1], [0.0]])
    for _ in range(int(10.0 / DT)):
        u = mpc.solve(x)
        x = Ad @ x + Bd * u
    assert np.all(np.abs(x) < 1e-3)
