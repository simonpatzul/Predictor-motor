# Captura un experimento de la Pico (firmware Lab 4), lo guarda en datos/ y lo grafica.
# Uso:
#   python3 captura/captura.py home            hace el home del codo
#   python3 captura/captura.py step 6 1.0      escalón de 6 V durante 1 s
#   python3 captura/captura.py perfil 6 0.4 3 0.3 -6 0.4 -3 0.3   tramos de voltaje (V, s) seguidos
import os
import sys
import time
import serial
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PUERTO = '/dev/ttyACM0'
CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'datos')


def enviar(cmd, timeout=30):
    s = serial.Serial(PUERTO, 115200, timeout=0.5)
    time.sleep(0.2)
    s.reset_input_buffer()
    s.write((cmd + '\n').encode())
    lineas, t0 = [], time.time()
    while time.time() - t0 < timeout:
        l = s.readline().decode(errors='ignore').strip()
        if not l:
            continue
        lineas.append(l)
        if l.startswith('#'):
            print(l)
            if any(k in l for k in ("# fin", "home OK", "FALLA", "comandos", "primero", "estado:", "deshabilitado")):
                break
    s.close()
    return lineas


def graficar(ruta_csv):
    import csv
    with open(ruta_csv) as fh:
        filas = list(csv.DictReader(fh))
    col = lambda k: [float(f[k]) for f in filas]
    t = col('t')
    fig, ax = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(t, col('w'), label='medida', lw=1)
    ax[0].plot(t, col('wL'), '--', label='Luenberger')
    ax[0].plot(t, col('wK'), '-.', label='Kalman')
    ax[0].set_ylabel('w motor (rad/s)'); ax[0].legend(); ax[0].grid()
    ax[1].plot(t, col('iL'), '--', label='Luenberger')
    ax[1].plot(t, col('iK'), '-.', label='Kalman')
    ax[1].set_ylabel('corriente estimada (A)'); ax[1].legend(); ax[1].grid()
    ax[2].plot(t, col('u'), label='u (V)')
    ax[2].set_ylabel('voltaje (V)'); ax[2].set_xlabel('t (s)'); ax[2].grid()
    fig.tight_layout()
    png = ruta_csv.replace('.csv', '.png')
    fig.savefig(png, dpi=110)
    print('Grafica:', os.path.normpath(png))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__ if __doc__ else 'uso: captura.py home | step V T')
        sys.exit(1)
    cmd = ' '.join(sys.argv[1:])
    lineas = enviar(cmd, timeout=60)
    datos = [l for l in lineas if not l.startswith('#')]
    if sys.argv[1] in ('step', 'perfil', 'vaiven') and len(datos) > 1:
        os.makedirs(CARPETA, exist_ok=True)
        if sys.argv[1] == 'step':
            nombre = f"step_{sys.argv[2]}V_{sys.argv[3]}s_{time.strftime('%H%M%S')}.csv"
        else:
            nombre = f"{sys.argv[1]}_{time.strftime('%H%M%S')}.csv"
        ruta = os.path.join(CARPETA, nombre)
        with open(ruta, 'w') as fh:
            fh.write('\n'.join(datos) + '\n')
        print('Datos:', os.path.normpath(ruta), f'({len(datos) - 1} muestras)')
        graficar(ruta)
