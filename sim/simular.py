# Corre los observadores de Luenberger y Kalman sobre los datos medidos (mismo algoritmo que la Pico)
# para afinarlos antes de cargarlos. Grafica velocidad medida vs. estimada y corriente estimada.
# Uso: python3 sim/simular.py
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo as m
from ajuste import cargar, DATOS

C = m.C


def luenberger(Ad, Bd, L, u, y):
    """Predictor de la guía: x(k) = (A - L C) x(k-1) + L y(k-1) + B u(k-1)."""
    x = np.zeros((len(u), 2))
    for k in range(1, len(u)):
        x[k] = (Ad - L @ C) @ x[k - 1] + L.ravel() * y[k - 1] + Bd.ravel() * u[k]
    return x


def kalman(Ad, Bd, Q, R, P0, u, y):
    x = np.zeros((len(u), 2))
    P = P0.copy()
    for k in range(1, len(u)):
        xm = Ad @ x[k - 1] + Bd.ravel() * u[k]                 # predicción
        Pm = Ad @ P @ Ad.T + Q
        K = Pm @ C.T / (C @ Pm @ C.T + R)                      # corrección
        x[k] = xm + K.ravel() * (y[k] - (C @ xm)[0])
        P = (np.eye(2) - K @ C) @ Pm
    return x, K.ravel()


if __name__ == '__main__':
    Ad, Bd = m.discreto(*m.continuo())
    L = m.luenberger(Ad)
    exps = cargar()
    fig, ax = plt.subplots(len(exps), 2, figsize=(11, 3.2 * len(exps)), squeeze=False)
    for f, (nombre, t, u, w) in enumerate(exps):
        xl = luenberger(Ad, Bd, L, u, w)
        xk, Kss = kalman(Ad, Bd, m.Q_KALMAN, m.R_KALMAN, m.P0_KALMAN, u, w)
        on = u > 0
        print(f'{nombre}: error w  Luenberger RMS {np.sqrt(np.mean((xl[:,1]-w)**2)):5.1f}  '
              f'Kalman RMS {np.sqrt(np.mean((xk[:,1]-w)**2)):5.1f} rad/s | i estable L {xl[on,0][-10:].mean():.3f} A  '
              f'K {xk[on,0][-10:].mean():.3f} A')
        ax[f, 0].plot(t, w, label='medida', lw=1)
        ax[f, 0].plot(t, xl[:, 1], '--', label='Luenberger')
        ax[f, 0].plot(t, xk[:, 1], '-.', label='Kalman')
        ax[f, 0].set_title(nombre); ax[f, 0].set_ylabel('w (rad/s)'); ax[f, 0].grid(); ax[f, 0].legend(fontsize=8)
        ax[f, 1].plot(t, xl[:, 0], '--', label='Luenberger')
        ax[f, 1].plot(t, xk[:, 0], '-.', label='Kalman')
        ax[f, 1].set_ylabel('i estimada (A)'); ax[f, 1].grid(); ax[f, 1].legend(fontsize=8)
    print('L =', L.ravel(), ' polos A-LC:', np.round(np.linalg.eigvals(Ad - L @ C), 3), ' K (estable) =', Kss)
    ax[-1, 0].set_xlabel('t (s)'); ax[-1, 1].set_xlabel('t (s)')
    fig.tight_layout()
    png = os.path.normpath(os.path.join(DATOS, 'observadores_sim.png'))
    fig.savefig(png, dpi=100)
    print('Grafica:', png)
