#!/bin/bash
# Prueba de perturbación: el codo va y viene a ±3.5 V durante 6 s, invirtiendo al llegar a 0.5 y 1.4 rad.
# Durante la prueba, frena el brazo con la mano unas veces (sin detenerlo más de medio segundo).
# Uso: captura/perturbacion.sh
cd "$(dirname "$0")/.."
echo "Empieza en 3 s: frena el brazo con la mano durante el movimiento..."
sleep 3
python3 captura/captura.py vaiven 3.5 6 0.5 1.4
