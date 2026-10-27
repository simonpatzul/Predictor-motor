#pragma once
// Generado por sim/pid_discreto.py. No editar a mano: cambia las especificaciones allí y vuelve a ejecutarlo.
// PID discreto de posición del codo:  C(z) = (q0 z^2 + q1 z + q2) / ((z - 1)(z + g)),  Ts = 0.01 s
// u(k) = (1-g) u(k-1) + g u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2)     (e en rad del codo, u en V)
// Polos de lazo cerrado en z: 0.9200+0.0753j, 0.9200-0.0753j, 0.6000, 0.6000
constexpr float PID_Q0 = 1.397109147e+02f;
constexpr float PID_Q1 = -2.479733630e+02f;
constexpr float PID_Q2 = 1.100321509e+02f;
constexpr float PID_G = -3.027328121e-01f;
// Prefiltro de la referencia: r(k) = a r(k-1) + (1-a) ref, dos veces en cascada
constexpr float PID_ALFA_REF = 9.300000000e-01f;
