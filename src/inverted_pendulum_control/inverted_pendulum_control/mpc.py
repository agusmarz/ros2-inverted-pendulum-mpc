import casadi as ca
import numpy as np

from scipy.linalg import solve_discrete_are


def stage_cost(x, u, Q, R):
    return ca.mtimes([x.T, Q, x]) + ca.mtimes([u.T, R, u])


def terminal_cost(x, P):
    return ca.mtimes([x.T, P, x])


class MPCController:
    """MPC lineal para un modelo en tiempo discreto x[k+1] = Ad x[k] + Bd u[k].

    El problema de optimización se arma una sola vez; cada llamada a solve()
    solo actualiza el parámetro del estado inicial y lo vuelve a resolver
    con Ipopt.
    """

    def __init__(self, Ad, Bd, Q, R, N):
        self.N = N
        nx = Ad.shape[0]
        nu = Bd.shape[1]

        # Costo terminal a partir de la ecuación de Riccati en tiempo discreto
        P = solve_discrete_are(Ad, Bd, Q, R)

        self.opti = ca.Opti()
        self.x = self.opti.variable(nx, N + 1)
        self.u = self.opti.variable(nu, N)
        self.x_init = self.opti.parameter(nx, 1)

        cost = 0
        for k in range(N):
            cost += stage_cost(self.x[:, k], self.u[:, k], Q, R)
        cost += terminal_cost(self.x[:, N], P)
        self.opti.minimize(cost)

        self.opti.subject_to(self.x[:, 0] == self.x_init)
        for k in range(N):
            self.opti.subject_to(self.x[:, k + 1] == Ad @ self.x[:, k] + Bd @ self.u[:, k])

        # Ipopt es un solver de optimización no lineal (punto interior). Con el
        # modelo lineal actual el problema es un QP, pero Ipopt permite pasar
        # más adelante a restricciones o a un modelo no lineal sin cambiar de
        # solver. print_level 0 y sb 'yes' apagan el log y el banner de Ipopt.
        self.opti.solver('ipopt', {
            'print_time': False,
            'ipopt': {
                'print_level': 0,
                'sb': 'yes',
                'tol': 1e-8,
            },
        })

        self._x_guess = np.zeros((nx, N + 1))
        self._u_guess = np.zeros((nu, N))

    def solve(self, state):
        """Devuelve la primera acción de control óptima para el estado dado."""
        self.opti.set_value(self.x_init, state)
        self.opti.set_initial(self.x, self._x_guess)
        self.opti.set_initial(self.u, self._u_guess)

        solution = self.opti.solve()

        # Incluí este warm start porque encontré que es una mejor implementación
        # de MPC: la solución de este paso, corrida un paso hacia adelante, se
        # usa como punto de partida del próximo solve, que así arranca cerca
        # del óptimo en lugar de arrancar desde cero.
        x_sol = np.reshape(solution.value(self.x), self._x_guess.shape)
        u_sol = np.reshape(solution.value(self.u), self._u_guess.shape)
        self._x_guess = np.hstack([x_sol[:, 1:], x_sol[:, -1:]])
        self._u_guess = np.hstack([u_sol[:, 1:], u_sol[:, -1:]])

        return float(u_sol[0, 0])
