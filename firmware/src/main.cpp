// Lab 4 - Observadores de estado: Luenberger y Kalman sobre el motor del codo (R1) del SCARA.
// La Pico aplica voltaje al motor, mide la velocidad con el encoder y estima corriente y velocidad
// con los dos observadores cada TS. Todo sale por USB como CSV.
//
// El hombro (R2) se sostiene activo en su ángulo de encendido (sostener_hombro) y P queda frenado.
// Comandos por el puerto serie (terminar con Enter):
//   home            busca el final de carrera del codo y pone el cero ahí
//   step V T        aplica V voltios (0 a 12) durante T segundos, se corta solo antes del final del recorrido
//   perfil V1 T1 V2 T2 ...   hasta 8 tramos de voltaje (-12 a 12 V) seguidos, p. ej. perfil 6 0.4 3 0.3 -6 0.4 -3 0.3
//   vaiven V T A B  va y viene a ±V durante T s, invirtiendo al llegar a A y B rad (prueba de perturbación)
//   pos REF T [VMAX [Q0 Q1 Q2 G]]  PID discreto de posición del codo (ver posicion())
//   stop            deshabilita el motor
//   estado          muestra el ángulo del codo, el final de carrera y si ya se hizo home
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <initializer_list>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "encoder.h"
#include "motor_driver.h"
#include "limit_switch.h"
#include "observer_params.h"
#include "pid_params.h"

// --- Hardware del codo (R1) en la baquela, igual que en el firmware del SCARA -------------
constexpr uint ENC_A = 11, ENC_B = 10;
constexpr float ENC_SIGN = -1.0f;              // así voltaje positivo => ángulo que aumenta
constexpr float CUENTAS_MOTOR = 64.0f;         // cuentas por vuelta del eje del motor (16 CPR x 4)
constexpr float RED_ARTIC = 50.0f * 3.0f;      // vueltas del motor por vuelta del codo (caja 50:1 y polea 3:1)
constexpr uint PWM_L = 4, PWM_R = 5, EN_L = 7, EN_R = 6;
constexpr uint LIMITE = 19;
constexpr float V_FUENTE = 12.0f;

// Recorrido seguro del codo medido desde su final de carrera (rad de la articulación).
// El codo llega hasta ~3.7 rad; se deja margen para frenar.
constexpr float ART_INICIO = 0.25f;
constexpr float ART_MAX = 2.6f;                // pasado ~2.85 rad el codo se traba al invertir
constexpr float ART_MIN = 0.08f;               // no acercarse más al final de carrera con voltaje negativo
constexpr int MAX_TRAMOS = 8;
constexpr int MUESTRAS_ATASCO = 15;            // 0.15 s con voltaje y sin moverse => se corta (motor atascado)
constexpr int MUESTRAS_ATASCO_BAJO = 50;       // con |u| <= 4 V se permite 0.5 s (corriente de bloqueo <= 1.8 A), para pruebas de carga a mano
constexpr float V_HOME = 3.0f;

Encoder encoder(pio0, 0, ENC_A, ENC_B);
MotorDriver driver(PWM_L, PWM_R, EN_L, EN_R, 1.0f);
// Hombro (R2) y prismático (P): driver habilitado con 0 V = freno (el BTS7960 cortocircuita el motor),
// para que la reacción del codo no haga girar el hombro y el brazo choque.
MotorDriver freno_hombro(0, 1, 3, 2, 1.0f);
MotorDriver freno_p(14, 15, 17, 16, 1.0f);
LimitSwitch limite(LIMITE, true);
bool home_ok = false;

float ang_motor() { return ENC_SIGN * encoder.count() * 2.0f * float(M_PI) / CUENTAS_MOTOR; }   // rad eje motor
float ang_artic() { return ang_motor() / RED_ARTIC; }                                            // rad del codo

// --- Sostén activo del hombro (R2) ----------------------------------------------------------
// El freno solo no basta: la reacción del codo lo hace girar. Un PD en un timer de 10 ms lo
// mantiene en el ángulo que tenía al encender la Pico, mientras el codo hace cualquier comando.
// Mismo motor, encoder y polea que el codo (pines y signo como en el firmware del SCARA).
Encoder encoder_hombro(pio0, 1, 12, 13);
constexpr float KP_HOMBRO = 30.0f;             // V/rad
constexpr float KD_HOMBRO = 1.0f;              // V/(rad/s)
constexpr float V_HOMBRO = 4.0f;               // tope (corriente de bloqueo <= 1.8 A)
constexpr float TOL_HOMBRO = 0.003f;           // dentro de esta banda (rad) queda solo frenado
constexpr float FALLA_HOMBRO = 0.15f;          // si se aleja más que esto algo anda mal: se suelta el lazo y queda frenado
volatile bool hombro_activo = true;
volatile float hombro_err = 0.0f;
repeating_timer_t timer_hombro;

float ang_hombro() { return ENC_SIGN * encoder_hombro.count() * 2.0f * float(M_PI) / CUENTAS_MOTOR / RED_ARTIC; }

bool sostener_hombro(repeating_timer_t*) {
    static float ang_ant = 0.0f;
    const float ang = ang_hombro();
    const float w = (ang - ang_ant) / TS;
    ang_ant = ang;
    const float e = -ang;                       // objetivo: 0 = donde estaba al encender
    hombro_err = e;
    if (fabsf(e) > FALLA_HOMBRO) hombro_activo = false;
    float u = 0.0f;
    if (hombro_activo && fabsf(e) > TOL_HOMBRO) {
        u = KP_HOMBRO * e - KD_HOMBRO * w;
        if (u > V_HOMBRO) u = V_HOMBRO;
        if (u < -V_HOMBRO) u = -V_HOMBRO;
    }
    freno_hombro.set(u / V_FUENTE);
    return true;
}

void aplicar(float v) { driver.set(v / V_FUENTE); }

// --- Observadores (2 estados, 1 medición) --------------------------------------------------
struct Luenberger {
    float x[2] = {0, 0};
    // Predictor: x(k) = (A - L C) x(k-1) + L y(k-1) + B u(k-1), con C = [0 1]
    void paso(float u_ant, float y_ant) {
        const float e = y_ant - x[1];
        const float x0 = AD[0] * x[0] + AD[1] * x[1] + BD[0] * u_ant + LG[0] * e;
        const float x1 = AD[2] * x[0] + AD[3] * x[1] + BD[1] * u_ant + LG[1] * e;
        x[0] = x0; x[1] = x1;
    }
};

struct Kalman {
    float x[2] = {0, 0};
    float P[4] = {P0K[0], P0K[1], P0K[2], P0K[3]};
    void paso(float u_ant, float y) {
        // Predicción: x- = A x + B u ;  P- = A P A' + Q
        const float x0 = AD[0] * x[0] + AD[1] * x[1] + BD[0] * u_ant;
        const float x1 = AD[2] * x[0] + AD[3] * x[1] + BD[1] * u_ant;
        float AP[4] = {AD[0] * P[0] + AD[1] * P[2], AD[0] * P[1] + AD[1] * P[3],
                       AD[2] * P[0] + AD[3] * P[2], AD[2] * P[1] + AD[3] * P[3]};
        float Pm[4] = {AP[0] * AD[0] + AP[1] * AD[1] + QK[0], AP[0] * AD[2] + AP[1] * AD[3] + QK[1],
                       AP[2] * AD[0] + AP[3] * AD[1] + QK[2], AP[2] * AD[2] + AP[3] * AD[3] + QK[3]};
        // Corrección con C = [0 1]: K = P- C' / (C P- C' + R)
        const float s = Pm[3] + RK;
        const float K0 = Pm[1] / s, K1 = Pm[3] / s;
        const float e = y - x1;
        x[0] = x0 + K0 * e;
        x[1] = x1 + K1 * e;
        // P+ = (I - K C) P-
        P[0] = Pm[0] - K0 * Pm[2]; P[1] = Pm[1] - K0 * Pm[3];
        P[2] = Pm[2] - K1 * Pm[2]; P[3] = Pm[3] - K1 * Pm[3];
    }
};

// Mueve el codo con voltaje constante hasta que la condición se cumpla o pase el tiempo
template <typename Cond>
bool mover_hasta(float v, Cond listo, uint32_t timeout_ms) {
    driver.enable(true);
    aplicar(v);
    const absolute_time_t fin = make_timeout_time_ms(timeout_ms);
    while (!listo()) {
        if (time_reached(fin)) { aplicar(0); return false; }
        sleep_ms(1);
    }
    aplicar(0);
    return true;
}

void home() {
    printf("# home: buscando el final de carrera del codo...\n");
    if (!limite.pressed() && !mover_hasta(-V_HOME, [] { return limite.pressed(); }, 15000)) {
        driver.enable(false);
        printf("# FALLA: no se encontro el final de carrera\n");
        return;
    }
    sleep_ms(300);
    encoder.reset();
    if (!mover_hasta(V_HOME, [] { return ang_artic() >= ART_INICIO; }, 5000)) {
        driver.enable(false);
        printf("# FALLA: no se pudo salir del final de carrera\n");
        return;
    }
    sleep_ms(300);
    home_ok = true;
    printf("# home OK, codo en %.3f rad\n", ang_artic());
}

void volver_al_inicio() {
    sleep_ms(300);
    if (ang_artic() > ART_INICIO)
        mover_hasta(-V_HOME, [] { return ang_artic() <= ART_INICIO || limite.pressed(); }, 8000);
    else
        mover_hasta(V_HOME, [] { return ang_artic() >= ART_INICIO; }, 8000);
    sleep_ms(200);
}

// Aplica n tramos de voltaje v[j] durante t[j] segundos y registra todo cada TS.
// Se corta solo (voltaje a 0) si el codo se acerca a cualquiera de los dos extremos del recorrido.
void perfil(const float* v, const float* t, int n, float vaiven_a = -1, float vaiven_b = -1) {
    if (!home_ok) { printf("# primero haz home\n"); return; }
    const bool vaiven = vaiven_a >= 0;
    float signo = 1.0f;
    int fin_tramo[MAX_TRAMOS];
    int total = 0;
    for (int j = 0; j < n; ++j) { total += int(t[j] / TS + 0.5f); fin_tramo[j] = total; }
    const int n_on = total;
    const int n_off = int(0.4f / TS);                 // 0.4 s más con el motor sin voltaje
    Luenberger lu; Kalman ka;
    float u_ant = 0, y_ant = 0, ang_ant = ang_motor();
    bool cortado = false, atascado = false;
    int quieto = 0;

    driver.enable(true);
    printf("t,u,w,wL,iL,wK,iK,art\n");
    absolute_time_t prox = get_absolute_time();
    for (int k = 0; k < n_on + n_off; ++k) {
        // 1) Medir
        const float ang = ang_motor();
        const float w = (ang - ang_ant) / TS;
        ang_ant = ang;
        // 2) Observadores (con la entrada y la medición del instante anterior, como en la guía)
        lu.paso(u_ant, y_ant);
        ka.paso(u_ant, w);
        // 3) Actuar
        const float art = ang / RED_ARTIC;
        int j = 0;
        while (j < n - 1 && k >= fin_tramo[j]) ++j;
        if (vaiven) {                                 // invierte al llegar a los extremos A y B
            if (art >= vaiven_b) signo = -1.0f;
            if (art <= vaiven_a) signo = 1.0f;
        }
        float vj = vaiven ? signo * fabsf(v[0]) : v[j];
        if (vj > V_FUENTE) vj = V_FUENTE;
        if (vj < -V_FUENTE) vj = -V_FUENTE;
        if ((vj > 0 && art >= ART_MAX) || (vj < 0 && art <= ART_MIN)) cortado = true;
        quieto = (fabsf(u_ant) > 2.0f && fabsf(w) < 5.0f) ? quieto + 1 : 0;
        if (quieto >= (fabsf(u_ant) <= 4.0f ? MUESTRAS_ATASCO_BAJO : MUESTRAS_ATASCO)) { cortado = true; atascado = true; }
        const float u = (k < n_on && !cortado) ? vj : 0.0f;
        aplicar(u);
        printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.4f,%.3f\n", k * TS, u_ant, w, lu.x[1], lu.x[0], ka.x[1], ka.x[0], art);
        u_ant = u; y_ant = w;
        prox = delayed_by_us(prox, uint64_t(TS * 1e6f));
        sleep_until(prox);
    }
    aplicar(0);
    printf("# fin%s\n", atascado ? " (se corto: motor atascado)" : cortado ? " (se corto por fin de recorrido)" : "");
    volver_al_inicio();
}

// --- Control de posición del codo: PID discreto ---------------------------------------------
// pos REF T [VMAX [Q0 Q1 Q2 G]]: lleva el codo a REF rad durante T s (máx. 120) y registra todo cada TS.
// Mientras corre acepta por el serie:  r REF  (nueva referencia)  |  x  (termina).
// Ley de control (diseñada en z con sim/pid_discreto.py, coeficientes en pid_params.h):
//   u(k) = (1-g) u(k-1) + g u(k-2) + q0 e(k) + q1 e(k-1) + q2 e(k-2)
// u(k-1), u(k-2) son los voltajes ya saturados, así la parte integral no se acumula al saturar.
// e(k) = rf(k) - theta(k), con rf la referencia pasada por el prefiltro F(z) = ((1-a) z / (z - a))^2.
// Al terminar se queda donde está.
constexpr float REF_MIN = 0.15f, REF_MAX = 2.5f;

float limitar(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

void posicion(float ref, float t_max, float v_max, float q0, float q1, float q2, float g) {
    if (!home_ok) { printf("# primero haz home\n"); return; }
    ref = limitar(ref, REF_MIN, REF_MAX);
    t_max = limitar(t_max, 0.1f, 120.0f);
    v_max = limitar(v_max, 0.5f, V_FUENTE);
    const int n_max = int(t_max / TS + 0.5f);
    Luenberger lu; Kalman ka;
    float u_ant = 0, u_ant2 = 0, y_ant = 0, ang_ant = ang_motor(), e1 = 0, e2 = 0;
    float rf1 = ang_artic(), rf = rf1;              // estados del prefiltro, arrancan en el ángulo actual
    bool cortado = false, atascado = false, abortado = false;
    int quieto = 0;
    char cmd[32]; int nc = 0;

    driver.enable(true);
    printf("# pos: ref %.3f rad, q0 %.4f q1 %.4f q2 %.4f g %.4f vmax %.1f\n", ref, q0, q1, q2, g, v_max);
    printf("t,u,w,wL,iL,wK,iK,art,ref,rf,trP\n");
    absolute_time_t prox = get_absolute_time();
    for (int k = 0; k < n_max && !abortado; ++k) {
        // Comandos que llegan durante el lazo (sin bloquear)
        for (int c; (c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT;) {
            if (c != '\n' && c != '\r') { if (nc < int(sizeof(cmd)) - 1) cmd[nc++] = char(c); continue; }
            cmd[nc] = '\0'; nc = 0;
            float r;
            if (sscanf(cmd, "r %f", &r) == 1) ref = limitar(r, REF_MIN, REF_MAX);
            else if (cmd[0] == 'x' || strcmp(cmd, "stop") == 0) abortado = true;
        }
        // 1) Medir
        const float ang = ang_motor();
        const float w = (ang - ang_ant) / TS;
        ang_ant = ang;
        // 2) Observadores
        lu.paso(u_ant, y_ant);
        ka.paso(u_ant, w);
        // 3) PID discreto sobre el ángulo del codo
        const float art = ang / RED_ARTIC;
        rf1 = PID_ALFA_REF * rf1 + (1.0f - PID_ALFA_REF) * ref;
        rf = PID_ALFA_REF * rf + (1.0f - PID_ALFA_REF) * rf1;
        const float e = rf - art;
        float u = (1.0f - g) * u_ant + g * u_ant2 + q0 * e + q1 * e1 + q2 * e2;
        u = limitar(u, -v_max, v_max);
        e2 = e1; e1 = e;
        if ((u > 0 && art >= ART_MAX) || (u < 0 && art <= ART_MIN)) cortado = true;
        // Atasco: con voltaje alto se corta rápido; con voltaje bajo solo si el error todavía es grande
        const bool sin_mover = fabsf(u_ant) > 2.0f && fabsf(w) < 5.0f;
        if (sin_mover && (fabsf(u_ant) > 4.0f || fabsf(e) > 0.05f)) ++quieto; else quieto = 0;
        if (quieto >= (fabsf(u_ant) <= 4.0f ? MUESTRAS_ATASCO_BAJO : MUESTRAS_ATASCO)) { cortado = true; atascado = true; }
        if (cortado) { u = 0.0f; abortado = true; }
        aplicar(u);
        // "Traza" de P del Kalman como la pidió el profe: raíz de la suma de los cuadrados de la diagonal
        const float tr_p = sqrtf(ka.P[0] * ka.P[0] + ka.P[3] * ka.P[3]);
        printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f\n", k * TS, u_ant, w, lu.x[1], lu.x[0], ka.x[1], ka.x[0], art, ref, rf, tr_p);
        u_ant2 = u_ant; u_ant = u; y_ant = w;
        prox = delayed_by_us(prox, uint64_t(TS * 1e6f));
        sleep_until(prox);
    }
    aplicar(0);
    printf("# fin%s\n", atascado ? " (se corto: motor atascado)" : cortado ? " (se corto por fin de recorrido)" : "");
}

int main() {
    stdio_init_all();
    encoder.init();
    driver.init();
    limite.init();
    for (MotorDriver* f : {&freno_hombro, &freno_p}) { f->init(); f->set(0.0f); f->enable(true); }
    encoder_hombro.init();
    add_repeating_timer_ms(-10, sostener_hombro, nullptr, &timer_hombro);
    char linea[128];
    int n = 0;
    while (true) {
        const int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c != '\n' && c != '\r') {
            if (n < int(sizeof(linea)) - 1) linea[n++] = char(c);
            continue;
        }
        linea[n] = '\0';
        n = 0;
        float v, t, vs_[1], ts_[1], a_, b_;
        if (strcmp(linea, "home") == 0) home();
        else if (sscanf(linea, "step %f %f", &v, &t) == 2) { if (v < 0) v = 0; perfil(&v, &t, 1); }
        else if (sscanf(linea, "vaiven %f %f %f %f", &vs_[0], &ts_[0], &a_, &b_) == 4) {
            if (a_ < ART_INICIO) a_ = ART_INICIO;
            if (b_ > ART_MAX) b_ = ART_MAX;
            if (b_ > a_) perfil(vs_, ts_, 1, a_, b_);
        }
        else if (strncmp(linea, "perfil", 6) == 0) {
            float vs[MAX_TRAMOS], ts[MAX_TRAMOS];
            int nt = 0, usados = 0;
            const char* p = linea + 6;
            while (nt < MAX_TRAMOS && sscanf(p, "%f %f%n", &vs[nt], &ts[nt], &usados) == 2) { p += usados; ++nt; }
            if (nt > 0) perfil(vs, ts, nt);
            else printf("# uso: perfil V1 T1 V2 T2 ...\n");
        }
        else if (strncmp(linea, "pos ", 4) == 0) {
            float ref, tp, vm = 6.0f, q0 = PID_Q0, q1 = PID_Q1, q2 = PID_Q2, g = PID_G;
            if (sscanf(linea, "pos %f %f %f %f %f %f %f", &ref, &tp, &vm, &q0, &q1, &q2, &g) >= 2) posicion(ref, tp, vm, q0, q1, q2, g);
            else printf("# uso: pos REF T [VMAX [Q0 Q1 Q2 G]]\n");
        }
        else if (strcmp(linea, "estado") == 0)
            printf("# estado: codo %.3f rad, final de carrera %d, home %d, hombro %.4f rad%s\n", ang_artic(), limite.pressed(), home_ok,
                   ang_hombro(), hombro_activo ? " (sostenido)" : " (SUELTO: se alejo mas de 0.15 rad, solo frenado)");
        else if (strcmp(linea, "stop") == 0) { aplicar(0); driver.enable(false); printf("# motor deshabilitado\n"); }
        else if (linea[0]) printf("# comandos: home | step V T | perfil V1 T1 V2 T2 ... | vaiven V T A B | pos REF T [VMAX [Q0 Q1 Q2 G]] | stop | estado\n");
    }
}
