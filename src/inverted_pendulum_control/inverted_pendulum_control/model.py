import numpy as np


def linear_model(M=3.0, m=1.0, L=0.5, r=0.01, g=9.81):
    """Modelo en tiempo continuo linealizado alrededor del equilibrio vertical.

    Estado: [x, x_dot, theta, theta_dot]. Entrada: fuerza sobre el carro.

    Es un copy paste del modelo que estaba en control_node.py del repo
    original (el del LQR); solo se movió a una función para poder usarlo
    desde el nodo y desde los tests.
    """
    # Distancia entre el pivote y el centro de masa
    l = L / 2.0

    # Inercia del péndulo respecto de su centro de masa
    I = (m / 12.0) * (3.0 * r**2 + L**2)

    # Inercia total del péndulo respecto del pivote
    J = I + m * l**2

    # Denominador común (determinante) que aparece al resolver las ecuaciones de movimiento en forma matricial
    delta = (M + m) * J - (m * l)**2

    A = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -(m**2 * g * l**2) / delta, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, m * g * l * (M + m) / delta, 0.0]
    ])

    B = np.array([
        [0.0],
        [J / delta],
        [0.0],
        [-m * l / delta]
    ])

    return A, B


def discretize(A, B, dt):
    """Discretización por Euler explícito: x[k+1] = Ad x[k] + Bd u[k].

    Se aproxima la derivada como (x[k+1] - x[k]) / dt, lo que da
    Ad = I + A dt y Bd = B dt. Es más rústico que la retención de orden
    cero (exponencial de matriz), pero con dt chico la diferencia es
    pequeña.
    """
    Ad = np.eye(A.shape[0]) + A * dt
    Bd = B * dt
    return Ad, Bd
