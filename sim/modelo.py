# Modelo del motor DC (#4753, codo R1 del SCARA), discretización y diseño de los observadores.
# Genera firmware/include/observer_params.h con las matrices que usa la Pico.
#   Estados: x = [i_a (A); w (rad/s, eje del motor)]   Entrada: u = V_a (V)   Medición: y = w
# Uso: python3 sim/modelo.py
import os
import numpy as np
from scipy.signal import cont2discrete

TS = 0.01  # tiempo de muestreo (s), el mismo del lazo de la Pico

# --- Parámetros del motor ---------------------------------------------------------------
# Identificación del Lab 1 (Initial_model_Lab4.m): Ra=72.7, La=0.0662, Kv=0.0106, J=8.55e-7, beta=1.59e-6
# Ajuste inicial con el datasheet (#4753 a 12 V: 0.2 A sin carga, 200 rpm salida = 1047 rad/s motor, stall 5.5 A).
# J y La ajustados con sim/ajuste.py a los escalones de 4, 6 y 8 V medidos en el codo (incluyen la inercia del brazo).
V_NOM, I_NOM, W_NOM = 12.0, 0.2, 200 * 50 * 2 * np.pi / 60
Ra = 12.0 / 5.5
Kv = (V_NOM - Ra * I_NOM) / W_NOM       # V/(rad/s) = N*m/A
beta = Kv * I_NOM / W_NOM               # N*m*s
J = 2.806e-6                             # kg*m^2 (ajustado; el Lab 1 daba 8.55e-7 sin el brazo)
La = 5.56e-4                             # H (ajustado; tau_e = La/Ra ~ 0.25 ms)

# --- Diseño de los observadores -----------------------------------------------------------
POLOS_LUENBERGER = [0.0, 0.2]           # z: 0 = polo eléctrico de la planta (tau_e << Ts); 0.2 más rápido que el mecánico (0.81)
R_KALMAN = 35.0                          # varianza de w medida en estado estable (26 a 45 (rad/s)^2 en los escalones)
Q_KALMAN = np.diag([0.01**2, 10.0**2])   # incertidumbre del modelo: 10 mA en i, 10 rad/s en w
P0_KALMAN = np.diag([0.1, 100.0])


def continuo(Ra=Ra, La=La, Kv=Kv, J=J, beta=beta):
    Ac = np.array([[-Ra / La, -Kv / La],
                   [Kv / J, -beta / J]])
    Bc = np.array([[1.0 / La], [0.0]])
    return Ac, Bc


def discreto(Ac, Bc, ts=TS):
    Ad, Bd, *_ = cont2discrete((Ac, Bc, np.eye(2), np.zeros((2, 1))), ts, method='zoh')
    return Ad, Bd


C = np.array([[0.0, 1.0]])


def luenberger(Ad, polos=POLOS_LUENBERGER):
    """Fórmula de Ackermann: L = phi(A) * obs^-1 * [0; 1]."""
    obs = np.vstack([C, C @ Ad])
    assert np.linalg.matrix_rank(obs) == 2, 'el sistema no es observable'
    a = np.poly(polos)                                   # z^2 + a1 z + a2
    phi = Ad @ Ad + a[1] * Ad + a[2] * np.eye(2)
    return phi @ np.linalg.inv(obs) @ np.array([[0.0], [1.0]])


def escribir_header(Ad, Bd, L, ruta):
    f = lambda m: ', '.join(f'{v:.9e}f' for v in np.asarray(m).ravel())
    texto = f"""#pragma once
// Generado por sim/modelo.py. No editar a mano: cambia los parámetros en modelo.py y vuelve a ejecutarlo.
// Estados x = [i (A); w (rad/s motor)], entrada u (V), medición y = w. Ts = {TS} s
constexpr float TS = {TS}f;
constexpr float AD[4] = {{{f(Ad)}}};   // fila por fila
constexpr float BD[2] = {{{f(Bd)}}};
constexpr float LG[2] = {{{f(L)}}};    // ganancia de Luenberger
constexpr float QK[4] = {{{f(Q_KALMAN)}}};
constexpr float RK = {R_KALMAN:.9e}f;
constexpr float P0K[4] = {{{f(P0_KALMAN)}}};
"""
    with open(ruta, 'w') as fh:
        fh.write(texto)


if __name__ == '__main__':
    Ac, Bc = continuo()
    Ad, Bd = discreto(Ac, Bc)
    L = luenberger(Ad)
    x_ss = np.linalg.solve(Ac, -Bc * V_NOM).ravel()
    print(f'Ra={Ra:.3f} ohm  La={La:.2e} H  Kv={Kv:.5f}  J={J:.3e}  beta={beta:.3e}')
    print(f'Estado estable a 12 V: i={x_ss[0]:.3f} A  w={x_ss[1]:.1f} rad/s ({x_ss[1]*60/2/np.pi/50:.0f} rpm salida)')
    print('Polos de la planta (z):', np.round(np.linalg.eigvals(Ad), 4))
    print('Ad =\n', Ad, '\nBd =\n', Bd, '\nL =\n', L)
    print('Polos de A-LC:', np.round(np.linalg.eigvals(Ad - L @ C), 4))
    ruta = os.path.join(os.path.dirname(__file__), '..', 'firmware', 'include', 'observer_params.h')
    escribir_header(Ad, Bd, L, ruta)
    print('Escrito', os.path.normpath(ruta))
