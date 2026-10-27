# Genera las figuras del informe (tamaño de una columna) a partir de los datos medidos.
# Uso: python3 informe/figuras.py
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, '..', 'sim'))
import modelo as m
import pid_discreto as pid
import simular as s
from ajuste import simular as sim_modelo

DATOS = os.path.join(AQUI, '..', 'datos')
FIG = os.path.join(AQUI, 'figuras')
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({'font.size': 8, 'lines.linewidth': 1.1, 'legend.fontsize': 7, 'figure.dpi': 200})
ANCHO = 3.4  # pulgadas, una columna


def cargar(nombre):
    return np.genfromtxt(os.path.join(DATOS, nombre), delimiter=',', names=True)


def guardar(fig, nombre):
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, nombre), bbox_inches='tight')
    plt.close(fig)
    print('figura', nombre)


def fig_ajuste():
    """Modelo inicial vs. ajustado vs. datos (escalones 4, 6 y 8 V)."""
    Ac0, Bc0 = m.continuo(Ra=72.74, La=0.0662, Kv=0.01059, J=8.55e-7, beta=1.59e-6)   # Lab 1
    Ad0, Bd0 = m.discreto(Ac0, Bc0)
    Ad1, Bd1 = m.discreto(*m.continuo())
    fig, ax = plt.subplots(2, 1, figsize=(ANCHO, 3.6), sharex=True)
    for nombre, c in [('step_4V_1.0s_181355.csv', 'C0'), ('step_6V_0.8s_181417.csv', 'C1'), ('step_8V_0.6s_181421.csv', 'C2')]:
        d = cargar(nombre)
        v = d['u'].max()
        ax[0].plot(d['t'], d['w'], c, lw=0.8, label=f'measured {v:.0f} V')
        ax[0].plot(d['t'], sim_modelo(Ad1, Bd1, d['u'])[:, 1], c + '--', lw=1.1)
        ax[0].plot(d['t'], sim_modelo(Ad0, Bd0, d['u'])[:, 1], c + ':', lw=1.0)
        ax[1].plot(d['t'], sim_modelo(Ad1, Bd1, d['u'])[:, 0], c + '--', label=f'tuned {v:.0f} V')
    ax[0].plot([], [], 'k--', label='tuned model'); ax[0].plot([], [], 'k:', label='Lab 1 model')
    ax[0].set_ylabel(r'$\omega$ (rad/s)'); ax[0].grid(alpha=.3); ax[0].legend(ncol=2)
    ax[1].set_ylabel(r'model $i_a$ (A)'); ax[1].set_xlabel('t (s)'); ax[1].grid(alpha=.3); ax[1].legend()
    guardar(fig, 'ajuste.pdf')


def fig_nominal():
    """Respuesta del modelo ajustado a 12 V (continuo vs. discreto)."""
    from scipy.signal import lsim
    Ac, Bc = m.continuo()
    t = np.arange(0, 0.3, 1e-4)
    _, y, _ = lsim((Ac, Bc, np.eye(2), np.zeros((2, 1))), 12 * np.ones_like(t), t)
    Ad, Bd = m.discreto(Ac, Bc)
    td = np.arange(0, 0.3, m.TS)
    xd = sim_modelo(Ad, Bd, 12 * np.ones_like(td))
    fig, ax = plt.subplots(2, 1, figsize=(ANCHO, 3.0), sharex=True)
    ax[0].plot(t, y[:, 0], label='continuous'); ax[0].step(td, xd[:, 0], 'r', where='post', label='discrete (ZOH)')
    ax[0].axhline(0.2, color='k', ls=':', lw=.8); ax[0].set_ylabel(r'$i_a$ (A)'); ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(t, y[:, 1]); ax[1].step(td, xd[:, 1], 'r', where='post')
    ax[1].axhline(m.W_NOM, color='k', ls=':', lw=.8); ax[1].set_ylabel(r'$\omega$ (rad/s)'); ax[1].set_xlabel('t (s)'); ax[1].grid(alpha=.3)
    guardar(fig, 'nominal.pdf')


def fig_polos():
    """Corriente estimada por Luenberger con los polos del script original y con los finales."""
    Ad, Bd = m.discreto(*m.continuo())
    d = cargar('step_6V_0.8s_181417.csv')
    xk, _ = s.kalman(Ad, Bd, m.Q_KALMAN, m.R_KALMAN, m.P0_KALMAN, d['u'], d['w'])
    fig, ax = plt.subplots(figsize=(ANCHO, 2.0))
    for polos, est in [([0.3, 0.4], ':'), ([0.0, 0.2], '--')]:
        xl = s.luenberger(Ad, Bd, m.luenberger(Ad, polos), d['u'], d['w'])
        ax.plot(d['t'], xl[:, 0], est, label=f'Luenberger poles {polos}')
    ax.plot(d['t'], xk[:, 0], 'C2-.', label='Kalman')
    ax.set_ylabel(r'$\hat{i}_a$ (A)'); ax.set_xlabel('t (s)'); ax.set_ylim(-4, 3); ax.grid(alpha=.3); ax.legend()
    guardar(fig, 'polos.pdf')


def fig_real(nombre, salida, titulo=None):
    d = cargar(nombre)
    fig, ax = plt.subplots(3, 1, figsize=(ANCHO, 4.0), sharex=True, gridspec_kw={'height_ratios': [3, 3, 1.4]})
    ax[0].plot(d['t'], d['w'], 'C0', lw=0.9, label='measured')
    ax[0].plot(d['t'], d['wL'], 'C1--', label='Luenberger')
    ax[0].plot(d['t'], d['wK'], 'C2-.', label='Kalman')
    ax[0].set_ylabel(r'$\omega$ (rad/s)'); ax[0].grid(alpha=.3)
    ax[0].legend(ncol=3, loc='lower center', bbox_to_anchor=(0.5, 1.0), frameon=False)
    ax[1].plot(d['t'], d['iL'], 'C1--', label='Luenberger')
    ax[1].plot(d['t'], d['iK'], 'C2-.', label='Kalman')
    ax[1].set_ylabel(r'$\hat{i}_a$ (A)'); ax[1].grid(alpha=.3)
    ax[2].plot(d['t'], d['u'], 'k', lw=0.9)
    ax[2].set_ylabel('u (V)'); ax[2].set_xlabel('t (s)'); ax[2].grid(alpha=.3)
    guardar(fig, salida)


def fig_sim():
    Ad, Bd = m.discreto(*m.continuo())
    d = cargar('step_8V_0.6s_181421.csv')
    xl = s.luenberger(Ad, Bd, m.luenberger(Ad), d['u'], d['w'])
    xk, _ = s.kalman(Ad, Bd, m.Q_KALMAN, m.R_KALMAN, m.P0_KALMAN, d['u'], d['w'])
    fig, ax = plt.subplots(2, 1, figsize=(ANCHO, 3.2), sharex=True)
    ax[0].plot(d['t'], d['w'], 'C0', lw=0.9, label='measured')
    ax[0].plot(d['t'], xl[:, 1], 'C1--', label='Luenberger')
    ax[0].plot(d['t'], xk[:, 1], 'C2-.', label='Kalman')
    ax[0].set_ylabel(r'$\omega$ (rad/s)'); ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(d['t'], xl[:, 0], 'C1--', label='Luenberger'); ax[1].plot(d['t'], xk[:, 0], 'C2-.', label='Kalman')
    ax[1].set_ylabel(r'$\hat{i}_a$ (A)'); ax[1].set_xlabel('t (s)'); ax[1].grid(alpha=.3); ax[1].legend()
    guardar(fig, 'simulacion.pdf')


def fig_pid():
    """Discrete PID design vs. the real elbow: 0.5 to 2.0 rad step with Vmax = 6 V."""
    d = cargar('pos_223846.csv')
    i0 = np.argmax((d['ref'] == 2.0) & (d['t'] > 45))
    sl = slice(i0, i0 + 200)
    t = d['t'][sl] - d['t'][i0]
    planta3, num, den, _ = pid.planta()
    polos = [0.92 + 0.0753j, 0.92 - 0.0753j] + pid.POLOS_EXTRA
    q = pid.disenar(num, den, polos)
    ts, th, uu = pid.simular(planta3, q, ref=2.0, ang0=d['art'][i0], n=200)
    fig, ax = plt.subplots(2, 1, figsize=(ANCHO, 3.0), sharex=True, gridspec_kw={'height_ratios': [3, 2]})
    ax[0].plot(t, d['ref'][sl], 'k:', lw=0.9, label='reference')
    ax[0].plot(t, d['rf'][sl], 'C3--', lw=0.9, label='filtered ref.')
    ax[0].plot(ts, th, 'C1', label='simulated')
    ax[0].plot(t, d['art'][sl], 'C0', lw=0.9, label='measured')
    ax[0].set_ylabel(r'$\theta$ (rad)'); ax[0].grid(alpha=.3); ax[0].legend(ncol=2, fontsize=6.5)
    ax[1].plot(ts, uu, 'C1', label='simulated')
    ax[1].plot(t, d['u'][sl], 'C0', lw=0.9, label='measured')
    ax[1].set_ylabel('u (V)'); ax[1].set_xlabel('t (s)'); ax[1].grid(alpha=.3); ax[1].legend(fontsize=6.5)
    guardar(fig, 'pid_escalon.pdf')


def fig_pos_real():
    """Position control run on the elbow with live reference changes."""
    d = cargar('pos_223846.csv')
    sl = (d['t'] >= 33.0) & (d['t'] <= 48.5)
    t = d['t'][sl] - 33.0
    fig, ax = plt.subplots(3, 1, figsize=(ANCHO, 4.0), sharex=True, gridspec_kw={'height_ratios': [3, 2.4, 1.6]})
    ax[0].plot(t, d['ref'][sl], 'k:', lw=0.9, label='reference')
    ax[0].plot(t, d['art'][sl], 'C0', lw=0.9, label='measured')
    ax[0].set_ylabel(r'$\theta$ (rad)'); ax[0].grid(alpha=.3); ax[0].legend(ncol=2, loc='lower center', bbox_to_anchor=(0.5, 1.0), frameon=False)
    ax[1].plot(t, d['iL'][sl], 'C1--', label='Luenberger')
    ax[1].plot(t, d['iK'][sl], 'C2-.', label='Kalman')
    ax[1].set_ylabel(r'$\hat{i}_a$ (A)'); ax[1].grid(alpha=.3); ax[1].legend(ncol=2, fontsize=6.5)
    ax[2].plot(t, d['u'][sl], 'k', lw=0.9)
    ax[2].set_ylabel('u (V)'); ax[2].set_xlabel('t (s)'); ax[2].grid(alpha=.3)
    guardar(fig, 'pos_real.pdf')


def fig_traza():
    """Norm of the diagonal of the Kalman P on the Pico (k = -1 is P0)."""
    d = cargar('pos_223846.csv')
    p0 = np.sqrt(m.P0_KALMAN[0, 0] ** 2 + m.P0_KALMAN[1, 1] ** 2)
    k = np.arange(-1, 10)
    tr = np.concatenate([[p0], d['trP'][:10]])
    fig, ax = plt.subplots(figsize=(ANCHO, 1.7))
    ax.plot(k, tr, 'C2o-', ms=3)
    ax.axhline(d['trP'][-1], color='k', ls=':', lw=.8)
    ax.text(9.3, d['trP'][-1] + 4, f"{d['trP'][-1]:.2f}", ha='right', fontsize=7)
    ax.set_xlabel('sample $k$'); ax.set_ylabel(r'$\sqrt{P_{11}^2+P_{22}^2}$'); ax.grid(alpha=.3)
    guardar(fig, 'traza_p.pdf')


if __name__ == '__main__':
    fig_nominal()
    fig_ajuste()
    fig_sim()
    fig_polos()
    fig_real('step_5V_1.0s_181654.csv', 'real_escalon.pdf')
    fig_real('perfil_191152.csv', 'real_perfil.pdf')
    fig_real('vaiven_191730.csv', 'real_perturbacion.pdf')
    fig_pid()
    fig_pos_real()
    fig_traza()
