"""
probar_robot.py — Prueba la conexión con RobotStudio SIN cámara.

El robot dibuja un cuadrado y luego un círculo en el plano Y-Z alrededor de HOME.
Si esto funciona, la parte de comunicación está bien y solo queda la webcam.

  python probar_robot.py
  python probar_robot.py --lado 150 --radio 100
"""
import argparse
import math
import time

from robot_cliente import RobotStudio


def esperar(robot, obj, t_max=5.0):
    robot.mover(*obj)
    t0 = time.time()
    while robot.enviado != robot.objetivo and time.time() - t0 < t_max:
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--puerto", type=int, default=1025)
    ap.add_argument("--lado", type=float, default=200)
    ap.add_argument("--radio", type=float, default=120)
    a = ap.parse_args()

    robot = RobotStudio(a.ip, a.puerto, umbral_mm=0.5)
    print("Esperando a que el programa RAPID acepte la conexión…")
    while not robot.conectado:
        time.sleep(0.2)

    m = a.lado / 2
    print("Cuadrado")
    for p in [(0, -m, -m), (0, m, -m), (0, m, m), (0, -m, m), (0, -m, -m)]:
        esperar(robot, p)
        print(f"  Y {p[1]:+.0f}  Z {p[2]:+.0f}   ({robot.latencia_ms:.0f} ms)")
    print("Círculo")
    for i in range(41):
        ang = 2 * math.pi * i / 40
        esperar(robot, (0, a.radio * math.cos(ang), a.radio * math.sin(ang)))
    print("HOME")
    robot.home()
    time.sleep(2)
    robot.cerrar()
    print("Prueba terminada.")


if __name__ == "__main__":
    main()
