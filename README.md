# ROS 2 Inverted Pendulum: MPC en lugar de LQR

Este repo documenta las modificaciones que le hice a
[gmsanchez/ros2-inverted-pendulum-simulation-and-lqr-control](https://github.com/gmsanchez/ros2-inverted-pendulum-simulation-and-lqr-control)
(rama `jazzy`, ROS 2 Jazzy + Gazebo Harmonic). Es un carro sobre un riel con un péndulo invertido.

El proyecto original es de Kaan Aslım (licencia MIT, ver `LICENSE`), y Guido Sanchez lo migró a Jazzy. Este repo conserva todo ese historial de git; mis cambios son los últimos commits. En `src/` están los cuatro paquetes ROS 2. `docs/`, `images/` y `videos/` son la documentación del LQR original.

## Cambio principal

El controlador original era un **LQR** en tiempo continuo: resolvía la ecuación de Riccati continua al arrancar el nodo y aplicaba `u = -K x` en el callback de `/joint_states`.

Lo reemplacé por un **MPC lineal** en tiempo discreto:

- **Modelo**: el mismo modelo linealizado alrededor del equilibrio vertical, con estado `[x, x_dot, theta, theta_dot]` y como entrada la fuerza sobre el carro. Lo discreticé con Euler explícito: `Ad = I + A·dt`, `Bd = B·dt`.
- **Problema de optimización**: horizonte `N = 23` con `dt = 0.02 s`. El costo de etapa es `xᵀQx + uᵀRu` y el costo terminal `xᵀPx`, donde `P` sale de la ecuación de Riccati **discreta**.
- **Solver**: CasADi (`Opti()`) con Ipopt. Con el modelo lineal el problema es un QP, pero Ipopt deja la puerta abierta a restricciones o a un modelo no lineal. El primer solve tarda más (~175 ms) porque Ipopt se inicializa; después ronda los 3–4 ms. El problema se arma una sola vez y en cada paso solo se actualiza el estado inicial. Uso warm start con la solución anterior desplazada un paso.
- **Lazo de control**: ahora corre en un timer a `dt` fijo, en vez de ejecutarse en cada mensaje de `/joint_states`. El callback solo guarda el último estado medido.

## Archivos modificados / agregados

Rutas relativas a `src/inverted_pendulum_control/` salvo la última.

| Archivo | Cambio |
|---|---|
| `inverted_pendulum_control/model.py` | **Nuevo.** `linear_model()` (matrices A, B, copiadas del nodo LQR original) y `discretize()` (Euler explícito). |
| `inverted_pendulum_control/mpc.py` | **Nuevo.** Clase `MPCController` (CasADi + Ipopt). |
| `inverted_pendulum_control/control_node.py` | Saqué el LQR, uso `MPCController` con timer y parámetros ROS. |
| `config/mpc_params.yaml` | **Nuevo.** Parámetros `dt`, `horizon`, `Q`, `R`. |
| `inverted_pendulum_control/disturbance_test.py` | Espera a que el bridge esté suscrito antes de aplicar la perturbación (antes a veces se perdía el mensaje). |
| `test/test_mpc.py` | **Nuevo.** Tests: con estado cero da fuerza cero, y estabiliza el péndulo arrancando con una inclinación de 0.1 rad. |
| `setup.py` | Instala `config/*.yaml`. |
| `package.xml` | Nota: `casadi` se instala con pip (no está en rosdep). |
| `inverted_pendulum_bringup/launch/inverted_pendulum.launch.xml` | Le pasa `mpc_params.yaml` al nodo de control. |

## Parámetros (`config/mpc_params.yaml`)

```yaml
inverted_pendulum_controller:
  ros__parameters:
    dt: 0.02
    horizon: 23
    Q: [10.0, 1.0, 100.0, 1.0]
    R: 0.1
```

Este archivo es la única fuente de los parámetros: el nodo los declara sin valores por defecto, así que falla al arrancar si no se los pasa. El launch del bringup ya lo carga; con `ros2 run` hay que agregar `--ros-args --params-file <ruta>/mpc_params.yaml`. Si editás el yaml, recompilá `inverted_pendulum_control` para que se instale la versión nueva.

`Q` y `R` son los mismos pesos que usaba el LQR original, así que los dos controladores se pueden comparar directamente.

## Cómo correrlo

### 1. Instalar y compilar (una sola vez)

```bash
pip install casadi
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/agusmarz/ros2-inverted-pendulum-mpc.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Si casadi está instalado en un entorno virtual (por ejemplo `~/entorno_casadi`), los nodos de ROS no lo ven solos: hay que agregarlo al `PYTHONPATH` en cada terminal, antes de lanzar nada:

```bash
export PYTHONPATH=$HOME/entorno_casadi/lib/python3.12/site-packages:$PYTHONPATH
```

### 2. Abrir la simulación (terminal 1)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch inverted_pendulum_bringup inverted_pendulum.launch.xml
```

Se abren Gazebo y RViz con el péndulo vertical, y en la terminal tiene que aparecer `MPC started: dt=0.02, horizon=23`.

### 3. Romper el equilibrio (terminal 2)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run inverted_pendulum_control disturbance_test
```

Aplica un torque de 0.5 N·m sobre el péndulo durante 0.05 s. El péndulo se inclina unos 2°, el carro se desplaza unos 13 cm para atajarlo y en pocos segundos vuelve todo al equilibrio. El torque y la duración se pueden cambiar:

```bash
ros2 run inverted_pendulum_control disturbance_test --ros-args -p torque:=1.0 -p duration:=0.05
```

Con perturbaciones muy grandes (por ejemplo 3 N·m durante 0.1 s) el carro llega al tope del riel (0.5 m) y el péndulo se cae: el MPC todavía no tiene restricciones que lo tengan en cuenta.

### 4. Verificar que todo funcione (terminal 2)

```bash
# Tests unitarios del MPC (tienen que pasar los 2)
cd ~/ros2_ws/src/ros2-inverted-pendulum-mpc/src/inverted_pendulum_control
python3 -m pytest test/test_mpc.py

# Con la simulación abierta:
ros2 topic hz /joint_states       # ~1000 Hz (Gazebo publica en cada paso de 1 ms)
ros2 topic hz /cart_force_cmd     # ~50 Hz (el MPC corre cada dt = 0.02 s)
ros2 param get /inverted_pendulum_controller horizon   # 23, leído de mpc_params.yaml
ros2 topic echo /joint_states --field position         # [x, theta] del carro y el péndulo
```

## Pendiente

- Agregar restricciones de fuerza y de recorrido del carro (hoy el MPC no tiene restricciones).
- Comparar LQR y MPC ante perturbaciones (`disturbance_test.py`).
