# Diseño del PID discreto de posición del codo por ubicación de polos en z.
# Genera firmware/include/pid_params.h con los coeficientes que usa la Pico.
#   Planta: G(z) = theta_codo(z) / U(z), modelo del motor de modelo.py + integrador, ZOH con Ts = 10 ms.
#   PID con derivada filtrada:  C(z) = (q0 z^2 + q1 z + q2) / ((z - 1)(z + g))
#   Ecuación en diferencias:    u(k) = (1-g) u(k-1) + g u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2)
#   Prefiltro de referencia:    F(z) = ((1-a) z / (z - a))^2, para que un cambio grande de referencia
#                               no sature el lazo (con q0 grande, u saltaría de +Vmax a -Vmax).
# Uso: python3 sim/pid_discreto.py
import os
import numpy as np
from scipy.signal import cont2discrete, ss2tf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from modelo import continuo, TS

RED_ARTIC = 150.0          # vueltas del motor por vuelta del codo (50:1 y polea 3:1)
V_MAX = 6.0                # saturación por defecto en el firmware

# --- Especificaciones (lazo cerrado) ----------------------------------------------------
ZETA = 0.7                 # sobrepaso ~4.6 %
TS_2 = 0.5                 # tiempo de establecimiento al 2 % (s), con escalón pequeño (sin saturar)
POLOS_EXTRA = [0.6, 0.6]   # polos no dominantes, más rápidos que el par dominante
ALFA_REF = 0.93            # polo doble del prefiltro (tau = -Ts/ln(a) ~ 0.14 s)


def planta():
    """G(z) de V a ángulo del codo. Devuelve el modelo de 3 estados [i, w, theta] y la G(z) reducida."""
    Ac, Bc = continuo()
    A = np.zeros((3, 3)); A[:2, :2] = Ac; A[2, 1] = 1.0 / RED_ARTIC
    B = np.vstack([Bc, [[0.0]]]); C = np.array([[0.0, 0.0, 1.0]])
    Ad, Bd, Cd, Dd, _ = cont2discrete((A, B, C, np.zeros((1, 1))), TS, method='zoh')
    num, den = ss2tf(Ad, Bd, Cd, Dd)
    num = np.trim_zeros(num[0], 'f')
    # El polo eléctrico queda en z ~ 0 (tau_e = 0.25 ms << Ts) y se cancela con el cero en z ~ 0
    polos, ceros = np.roots(den), np.roots(num)
    ip, ic = np.argmin(abs(polos)), np.argmin(abs(ceros))
    p_e, c_e = polos[ip], ceros[ic]
    num_r = np.real(np.poly(np.delete(ceros, ic)) * num[0])
    den_r = np.real(np.poly(np.delete(polos, ip)))
    return (Ad, Bd, Cd), num_r, den_r, (p_e, c_e)


def disenar(num, den, polos_z):
    """Iguala el polinomio característico al deseado (sistema lineal en g, q0, q1, q2).
    den = (z-1)(z-a), num = b1 z + b0.  Car(z) = (z-1)(z+g)·den + num·(q0 z^2 + q1 z + q2)."""
    deseado = np.real(np.poly(polos_z))                # grado 4, mónico
    z1 = np.polymul([1, -1], den)                       # (z-1)·den, grado 3
    # Car = z·z1 + g·z1 + q0·z^2·num + q1·z·num + q2·num   (lineal en g, q0, q1, q2)
    cols = [np.polymul([1], z1), np.polymul([1, 0, 0], num), np.polymul([1, 0], num), num]
    M = np.zeros((5, 4))
    for j, c in enumerate(cols):
        M[5 - len(c):, j] = c
    rhs = deseado - np.polymul([1, 0], z1)
    sol = np.linalg.lstsq(M[1:], rhs[1:], rcond=None)[0]   # el término z^4 ya coincide
    g, q0, q1, q2 = sol
    return q0, q1, q2, g


def simular(planta3, q, ref=1.0, ang0=0.25, v_max=V_MAX, n=150, alfa=ALFA_REF):
    """Lazo cerrado sobre el modelo de 3 estados, con saturación (igual que el firmware)."""
    Ad, Bd, Cd = planta3
    q0, q1, q2, g = q
    x = np.array([0.0, 0.0, ang0])
    e1 = e2 = u1 = u2 = 0.0
    r1 = r2 = ang0
    t, th, uu = [], [], []
    for k in range(n):
        th_k = (Cd @ x).item()
        r1 = alfa * r1 + (1 - alfa) * ref          # prefiltro de 2.º orden
        r2 = alfa * r2 + (1 - alfa) * r1
        e = r2 - th_k
        u = (1 - g) * u1 + g * u2 + q0 * e + q1 * e1 + q2 * e2
        u = float(np.clip(u, -v_max, v_max))
        t.append(k * TS); th.append(th_k); uu.append(u)
        x = Ad @ x + Bd.ravel() * u
        e2, e1, u2, u1 = e1, e, u1, u
    return np.array(t), np.array(th), np.array(uu)


def escribir_header(q, polos_z, ruta):
    q0, q1, q2, g = q
    p = ', '.join(f'{z:.4f}' for z in polos_z)
    texto = f"""#pragma once
// Generado por sim/pid_discreto.py. No editar a mano: cambia las especificaciones allí y vuelve a ejecutarlo.
// PID discreto de posición del codo:  C(z) = (q0 z^2 + q1 z + q2) / ((z - 1)(z + g)),  Ts = {TS} s
// u(k) = (1-g) u(k-1) + g u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2)     (e en rad del codo, u en V)
// Polos de lazo cerrado en z: {p}
constexpr float PID_Q0 = {q0:.9e}f;
constexpr float PID_Q1 = {q1:.9e}f;
constexpr float PID_Q2 = {q2:.9e}f;
constexpr float PID_G = {g:.9e}f;
// Prefiltro de la referencia: r(k) = a r(k-1) + (1-a) ref, dos veces en cascada
constexpr float PID_ALFA_REF = {ALFA_REF:.9e}f;
"""
    with open(ruta, 'w') as fh:
        fh.write(texto)


if __name__ == '__main__':
    planta3, num, den, (p_e, c_e) = planta()
    print(f'Polo eléctrico z={p_e.real:.2e} cancelado con el cero z={c_e.real:.2e}')
    print('G(z) =', np.round(num, 9), '/', np.round(den, 6), ' polos', np.round(np.roots(den), 4),
          ' cero', np.round(np.roots(num), 4))

    sigma = 4.0 / TS_2
    wd = sigma * np.sqrt(1 - ZETA**2) / ZETA
    zd = np.exp(-sigma * TS) * np.exp(1j * wd * TS)
    polos_z = [zd, np.conj(zd)] + POLOS_EXTRA
    q = disenar(num, den, polos_z)
    print(f'Polos dominantes z = {abs(zd):.4f} ∠ ±{np.angle(zd):.4f} rad   (zeta={ZETA}, ts={TS_2} s)')
    print('q0 = %.4f  q1 = %.4f  q2 = %.4f  g = %.4f' % q)

    # Comprobación: polos del lazo cerrado con la planta reducida
    car = np.polyadd(np.polymul(np.polymul([1, -1], [1, q[3]]), den), np.polymul(num, q[:3]))
    print('Polos de lazo cerrado:', np.round(np.roots(car), 4))

    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for ref, v in [(0.35, V_MAX), (1.25, V_MAX), (2.45, V_MAX)]:
        t, th, uu = simular(planta3, q, ref=ref, v_max=v, n=200)
        mp = (th.max() - ref) / (ref - 0.25) * 100
        ts = t[np.where(abs(th - ref) > 0.02 * (ref - 0.25))[0][-1] + 1]
        print(f'Escalón 0.25 -> {ref} rad (Vmax {v} V): sobrepaso {mp:.1f} %, ts(2%) {ts:.2f} s')
        ax[0].plot(t, th, label=f'0.25 → {ref} rad, Vmax {v} V')
        ax[1].plot(t, uu)
    ax[0].set_ylabel('θ codo (rad)'); ax[0].legend(); ax[0].grid()
    ax[1].set_ylabel('u (V)'); ax[1].set_xlabel('t (s)'); ax[1].grid()
    fig.tight_layout()
    aqui = os.path.dirname(os.path.abspath(__file__))
    png = os.path.normpath(os.path.join(aqui, '..', 'datos', 'pid_discreto_sim.png'))
    fig.savefig(png, dpi=110)
    print('Gráfica:', png)

    ruta = os.path.normpath(os.path.join(aqui, '..', 'firmware', 'include', 'pid_params.h'))
    escribir_header(q, polos_z, ruta)
    print('Escrito', ruta)
