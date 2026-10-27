# Lab 4 – Observadores de estado (Luenberger y Kalman)

Control Digital – Universidad EIA. Estimación de corriente y velocidad de un motor Pololu 37D #4753 (50:1)
midiendo solo la velocidad con el encoder. Implementado en una Raspberry Pi Pico sobre el codo (R1) del SCARA.

- `sim/modelo.py` – modelo del motor, discretización (Ts = 10 ms), Luenberger (Ackermann) y Kalman. Genera `firmware/include/observer_params.h`.
- `firmware/` – firmware de la Pico: aplica voltaje, mide la velocidad y corre los dos observadores cada 10 ms.
- `captura/captura.py` – manda comandos a la Pico, guarda el CSV en `datos/` y grafica.
- `matlab/` – scripts originales del laboratorio.

## Uso
```bash
python3 sim/modelo.py                                   # recalcula matrices y el .h
cmake -S firmware -B firmware/build && cmake --build firmware/build -j   # compila (PICO_SDK_PATH definido)
python3 captura/captura.py home                         # home del codo
python3 captura/captura.py step 6 1.0                   # escalón de 6 V durante 1 s
python3 captura/captura.py perfil 4 0.3 8 0.15 3 0.25 -6 0.3 -3 0.3   # cambios de referencia
captura/perturbacion.sh                                 # va y viene a ±3.5 V para frenarlo con la mano
python3 sim/pid_discreto.py                             # diseña el PID discreto en z y genera pid_params.h
python3 captura/interfaz.py                             # interfaz web en vivo (http://localhost:8000): setpoint de ángulo + PID discreto
python3 sim/ajuste.py                                   # ajuste del modelo con los escalones medidos
python3 sim/simular.py                                  # observadores sobre los datos medidos
```
Firmware ya compilado: `firmware/lab4_fw.uf2` (copiar a la Pico en modo BOOTSEL).
Informe: `informe/informe.pdf`.
Columnas del CSV: `t, u, w (medida), wL, iL (Luenberger), wK, iK (Kalman), art (ángulo del codo)`.
El comando `pos REF T [VMAX [Q0 Q1 Q2 G]]` agrega las columnas `ref`, `rf` (referencia filtrada) y `trP` (√(P11² + P22²) del Kalman); durante la corrida acepta `r REF` (nueva referencia) y `x` (termina).
Para volver al SCARA: cargar `scara_fw.uf2` del repo Scara-Robotica.
