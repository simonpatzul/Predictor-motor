#pragma once
// Generado por sim/modelo.py. No editar a mano: cambia los parámetros en modelo.py y vuelve a ejecutarlo.
// Estados x = [i (A); w (rad/s motor)], entrada u (V), medición y = w. Ts = 0.01 s
constexpr float TS = 0.01f;
constexpr float AD[4] = {-4.188924749e-03f, -4.154922111e-03f, 8.232846377e-01f, 8.166018130e-01f};   // fila por fila
constexpr float BD[2] = {3.793218370e-01f, 1.599078962e+01f};
constexpr float LG[2] = {-3.115995837e-03f, 6.124128882e-01f};    // ganancia de Luenberger
constexpr float QK[4] = {1.000000000e-04f, 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+02f};
constexpr float RK = 3.500000000e+01f;
constexpr float P0K[4] = {1.000000000e-01f, 0.000000000e+00f, 0.000000000e+00f, 1.000000000e+02f};
