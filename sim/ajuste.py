# Ajuste del modelo paramétrico a los datos medidos (punto 1 y 2 de la guía).
# Kv y beta quedan fijos por la condición de la guía (12 V -> 0.2 A y velocidad nominal) y se ajustan
# J y La para que la dinámica reproduzca los escalones de datos/step_*.csv. Grafica modelo vs. datos.
# Uso: python3 sim/ajuste.py
import glob
import os
import sys
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import modelo as m

AQUI = os.path.dirname(os.path.abspath(__file__))
DATOS = os.path.join(AQUI, '..', 'datos')


def cargar():
    exps = []
    for ruta in sorted(glob.glob(os.path.join(DATOS, 'step_*.csv'))):
        d = np.genfromtxt(ruta, delimiter=',', names=True)
        exps.append((os.path.basename(ruta), d['t'], d['u'], d['w']))
    return exps


def simular(Ad, Bd, u):
    """u[k] es el voltaje aplicado durante el intervalo anterior a la muestra k (columna u del CSV)."""
    x = np.zeros(2)
    out = np.zeros((len(u), 2))
    for k in range(1, len(u)):
        x = Ad @ x + Bd.ravel() * u[k]
        out[k] = x
    return out


def modelo_d(p):
    J, La = p
    Ac, Bc = m.continuo(Ra=m.Ra, La=La, Kv=m.Kv, J=J, beta=m.beta)
    return m.discreto(Ac, Bc)


def residuo(p, exps):
    Ad, Bd = modelo_d(p)
    return np.concatenate([simular(Ad, Bd, u)[:, 1] - w for _, _, u, w in exps])


if __name__ == '__main__':
    exps = cargar()
    if not exps:
        sys.exit('No hay datos en datos/step_*.csv')
    p0 = np.array([m.J, m.La])
    sol = least_squares(lambda q: residuo(q * p0, exps), np.ones(2), bounds=(0.01, 1000))
    J, La = sol.x * p0
    Kv, beta = m.Kv, m.beta
    rms = np.sqrt(np.mean(sol.fun ** 2))
    Ac, Bc = m.continuo(Ra=m.Ra, La=La, Kv=Kv, J=J, beta=beta)
    x12 = np.linalg.solve(Ac, -Bc * 12).ravel()
    tau_m = -1 / max(np.linalg.eigvals(Ac).real)
    print('Modelo ajustado a %d experimentos (error RMS %.1f rad/s):' % (len(exps), rms))
    print(f'  Ra={m.Ra:.3f}  La={La:.2e}  Kv={Kv:.5f}  J={J:.3e}  beta={beta:.3e}   tau_mec={tau_m*1000:.0f} ms')
    print(f'  A 12 V: i={x12[0]:.3f} A  w={x12[1]:.0f} rad/s ({x12[1]*60/2/np.pi/50:.0f} rpm salida)')

    fig, ax = plt.subplots(len(exps), 2, figsize=(11, 3.2 * len(exps)), squeeze=False)
    Ad0, Bd0 = m.discreto(*m.continuo())
    Ad1, Bd1 = modelo_d(sol.x * p0)
    for f, (nombre, t, u, w) in enumerate(exps):
        s0, s1 = simular(Ad0, Bd0, u), simular(Ad1, Bd1, u)
        ax[f, 0].plot(t, w, label='medida')
        ax[f, 0].plot(t, s0[:, 1], ':', label='modelo inicial')
        ax[f, 0].plot(t, s1[:, 1], '--', label='modelo ajustado')
        ax[f, 0].set_title(nombre); ax[f, 0].set_ylabel('w (rad/s)'); ax[f, 0].grid(); ax[f, 0].legend(fontsize=8)
        ax[f, 1].plot(t, s0[:, 0], ':', label='modelo inicial')
        ax[f, 1].plot(t, s1[:, 0], '--', label='modelo ajustado')
        ax[f, 1].set_ylabel('i (A)'); ax[f, 1].grid(); ax[f, 1].legend(fontsize=8)
    ax[-1, 0].set_xlabel('t (s)'); ax[-1, 1].set_xlabel('t (s)')
    fig.tight_layout()
    png = os.path.normpath(os.path.join(DATOS, 'ajuste_modelo.png'))
    fig.savefig(png, dpi=100)
    print('Grafica:', png)
