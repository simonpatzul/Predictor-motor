# Interfaz web en vivo para el firmware del Lab 4: manda comandos a la Pico y grafica lo que llega.
# Uso:
#   python3 captura/interfaz.py              abre http://localhost:8000
#   python3 captura/interfaz.py /dev/ttyACM1 8080
# Cada corrida (pos, step, perfil, vaiven) se guarda en datos/ como CSV, igual que captura.py.
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

PUERTO = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
HTTP = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
AQUI = os.path.dirname(os.path.abspath(__file__))
CARPETA = os.path.join(AQUI, '..', 'datos')
PAGINA = os.path.join(AQUI, 'interfaz.html')
PID_H = os.path.join(AQUI, '..', 'firmware', 'include', 'pid_params.h')

clientes = []                # una cola por navegador conectado
clientes_lock = threading.Lock()
puerto = {'s': None, 'ultimo_cmd': ''}
puerto_lock = threading.Lock()


def difundir(linea):
    with clientes_lock:
        for q in clientes:
            q.put(linea)


def guardar(filas):
    if len(filas) < 2:
        return
    os.makedirs(CARPETA, exist_ok=True)
    tipo = (puerto['ultimo_cmd'].split() or ['corrida'])[0]
    ruta = os.path.normpath(os.path.join(CARPETA, f"{tipo}_{time.strftime('%H%M%S')}.csv"))
    with open(ruta, 'w') as fh:
        fh.write('\n'.join(filas) + '\n')
    difundir(f'# guardado: {os.path.relpath(ruta, os.path.join(AQUI, ".."))} ({len(filas) - 1} muestras)')
    print('Datos:', ruta)


def lector():
    """Hilo que mantiene abierto el puerto, lee líneas y las reparte a los navegadores."""
    filas = None
    while True:
        try:
            s = serial.Serial(PUERTO, 115200, timeout=0.2)
        except serial.SerialException:
            time.sleep(1.0)
            continue
        with puerto_lock:
            puerto['s'] = s
        difundir('# conectado ' + PUERTO)
        print('Conectado a', PUERTO)
        try:
            while True:
                l = s.readline().decode(errors='ignore').strip()
                if not l:
                    continue
                if l.startswith('t,'):
                    filas = [l]
                elif l.startswith('# fin'):
                    if filas:
                        guardar(filas)
                    filas = None
                elif filas is not None and not l.startswith('#'):
                    filas.append(l)
                difundir(l)
        except (serial.SerialException, OSError):
            pass
        with puerto_lock:
            puerto['s'] = None
        try:
            s.close()
        except Exception:
            pass
        difundir('# desconectado')
        print('Se perdió', PUERTO)
        time.sleep(1.0)


def leer_pid():
    """Coeficientes del PID discreto que generó sim/pid_discreto.py (los mismos que trae el firmware)."""
    pid = {}
    try:
        with open(PID_H) as fh:
            for l in fh:
                p = l.split()
                if len(p) >= 5 and p[0] == 'constexpr' and p[2].startswith('PID_'):
                    pid[p[2][4:].lower()] = float(p[4].rstrip(';').rstrip('f'))
    except OSError:
        pass
    return pid


def enviar(cmd):
    with puerto_lock:
        s = puerto['s']
        if s is None:
            return False
        if not cmd.startswith('r ') and cmd != 'x':
            puerto['ultimo_cmd'] = cmd
        s.write((cmd + '\n').encode())
    return True


class Manejador(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def responder(self, codigo, tipo, cuerpo):
        self.send_response(codigo)
        self.send_header('Content-Type', tipo)
        self.send_header('Content-Length', str(len(cuerpo)))
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            with open(PAGINA, 'rb') as fh:
                self.responder(200, 'text/html; charset=utf-8', fh.read())
        elif self.path == '/estado':
            cuerpo = json.dumps({'conectado': puerto['s'] is not None, 'puerto': PUERTO}).encode()
            self.responder(200, 'application/json', cuerpo)
        elif self.path == '/pid':
            self.responder(200, 'application/json', json.dumps(leer_pid()).encode())
        elif self.path == '/eventos':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            q = queue.Queue()
            with clientes_lock:
                clientes.append(q)
            try:
                self.wfile.write(f"data: # {'conectado' if puerto['s'] else 'desconectado'} {PUERTO}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        lineas = [q.get(timeout=5)]
                    except queue.Empty:
                        self.wfile.write(b': ping\n\n')     # mantiene viva la conexión
                        self.wfile.flush()
                        continue
                    while not q.empty() and len(lineas) < 50:   # agrupa para no saturar al navegador
                        lineas.append(q.get_nowait())
                    self.wfile.write(('data: ' + '\ndata: '.join(lineas) + '\n\n').encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with clientes_lock:
                    clientes.remove(q)
        else:
            self.responder(404, 'text/plain', b'no existe')

    def do_POST(self):
        if self.path != '/cmd':
            return self.responder(404, 'text/plain', b'no existe')
        n = int(self.headers.get('Content-Length', 0))
        cmd = self.rfile.read(n).decode(errors='ignore').strip()[:120]
        ok = bool(cmd) and enviar(cmd)
        self.responder(200 if ok else 503, 'text/plain', b'ok' if ok else b'sin conexion con la Pico')


if __name__ == '__main__':
    threading.Thread(target=lector, daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', HTTP), Manejador)
    srv.daemon_threads = True
    print(f'Interfaz en http://localhost:{HTTP}  (puerto serie {PUERTO})')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
