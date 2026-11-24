"""
seguir_mano_robot.py — El robot de RobotStudio sigue tu mano (webcam + Faster R-CNN + Kalman).

Cadena:  webcam -> Faster R-CNN (mano) 1 de cada N fotogramas -> filtro de Kalman en todos
         -> centro de la mano (píxeles) -> suavizado -> desplazamiento en mm (plano Y-Z del robot)
         -> TCP -> módulo RAPID SeguirMano.mod -> MoveL en el controlador virtual

Es el mismo detector y el mismo Kalman del proyecto de conteo de dedos; aquí no se cuentan
dedos: lo que se usa es la POSICIÓN de la mano.

Uso:
  1) En RobotStudio: cargar SeguirMano.mod, PP a Main y Play (ver README.md).
  2) python seguir_mano_robot.py
     python seguir_mano_robot.py --sin-robot          (solo ver el seguimiento)
     python seguir_mano_robot.py --rango-y 400 --rango-z 250 --invertir-y

Teclas:
  q salir | p pausar/reanudar el robot | c calibrar (la posición actual de la mano = centro)
  h robot a HOME | r ver/ocultar la rejilla
"""
import argparse
import os
import time
from collections import deque

import cv2
import numpy as np
import torch

from modelos import crear_detector
from robot_cliente import RobotStudio

AQUI = os.path.dirname(os.path.abspath(__file__))


# ----------------------------------------------------------------------------- Kalman
class SeguidorKalman:
    """Velocidad constante sobre la caja: estado (cx, cy, w, h, vx, vy, vw, vh)."""

    def __init__(self):
        kf = cv2.KalmanFilter(8, 4)
        kf.transitionMatrix = np.eye(8, dtype=np.float32)
        for i in range(4):
            kf.transitionMatrix[i, i + 4] = 1.0
        kf.measurementMatrix = np.eye(4, 8, dtype=np.float32)
        kf.processNoiseCov = np.diag([1, 1, 1, 1, 10, 10, 10, 10]).astype(np.float32) * 1e-1
        kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 5.0
        self.kf, self.activo, self.sin_detectar = kf, False, 0

    @staticmethod
    def _a_med(c):
        x1, y1, x2, y2 = c
        return np.array([[(x1 + x2) / 2], [(y1 + y2) / 2], [x2 - x1], [y2 - y1]], np.float32)

    def iniciar(self, caja):
        self.kf.statePost = np.vstack([self._a_med(caja), np.zeros((4, 1), np.float32)])
        self.kf.errorCovPost = np.eye(8, dtype=np.float32)
        self.activo, self.sin_detectar = True, 0

    def predecir(self):
        cx, cy, w, h = self.kf.predict()[:4, 0]
        return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])

    def corregir(self, caja):
        self.kf.correct(self._a_med(caja))
        self.sin_detectar = 0


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


class SeguidorMano:
    def __init__(self, args):
        self.args = args
        self.dev = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        ck = torch.load(args.detector, map_location="cpu", weights_only=False)
        self.detector = crear_detector(ck.get("num_clases", 2), preentrenado=False,
                                       min_size=ck.get("min_size", 384), max_size=ck.get("max_size", 640))
        self.detector.load_state_dict(ck["state_dict"])
        self.detector.eval().to(self.dev)
        self.kalman = SeguidorKalman()
        self.n_frame = 0
        self.t_det = deque(maxlen=30)

    @torch.no_grad()
    def detectar(self, frame):
        t0 = time.time()
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        x = torch.from_numpy(rgb).permute(2, 0, 1).float().div(255).to(self.dev)
        o = self.detector([x])[0]
        self.t_det.append(time.time() - t0)
        k = o["scores"] >= self.args.umbral_det
        return o["boxes"][k].cpu().numpy(), o["scores"][k].cpu().numpy()

    def actualizar(self, frame):
        """Detección cada N fotogramas + predicción de Kalman en todos (asociación por IoU)."""
        pred = self.kalman.predecir() if self.kalman.activo else None
        if self.n_frame % self.args.cada == 0:
            cajas, scores = self.detectar(frame)
            if len(cajas):
                j = int(np.argmax(scores))
                if pred is not None:
                    jj = int(np.argmax([iou(pred, c) for c in cajas]))
                    if iou(pred, cajas[jj]) >= 0.1:
                        j = jj
                if not self.kalman.activo:
                    self.kalman.iniciar(cajas[j]); pred = cajas[j]
                else:
                    self.kalman.corregir(cajas[j])
            elif self.kalman.activo:
                self.kalman.sin_detectar += 1
                if self.kalman.sin_detectar > self.args.perdida:
                    self.kalman.activo, pred = False, None
        self.n_frame += 1
        return pred


# ----------------------------------------------------------------------------- mapeo imagen -> robot
class Mapeo:
    """
    Centro de la mano (u, v) en [-0.5, 0.5] respecto al centro de calibración ->
    desplazamiento (dx, dy, dz) en mm respecto a HOME. Plano vertical Y-Z del robot:
      mano a la derecha  -> +Y (o -Y con --invertir-y)
      mano arriba        -> +Z
    """

    def __init__(self, args):
        self.args = args
        self.centro = (0.5, 0.5)
        self.suave = None

    def calibrar(self, u, v):
        self.centro = (u, v)
        self.suave = None

    def __call__(self, u, v):
        a = self.args
        du, dv = u - self.centro[0], v - self.centro[1]
        dy = du * a.rango_y * (-1 if a.invertir_y else 1)
        dz = -dv * a.rango_z
        dy = float(np.clip(dy, -a.lim_y, a.lim_y))
        dz = float(np.clip(dz, -a.lim_z, a.lim_z))
        obj = np.array([0.0, dy, dz])
        # suavizado exponencial (además del Kalman): quita el temblor de la mano
        self.suave = obj if self.suave is None else a.alfa * obj + (1 - a.alfa) * self.suave
        return self.suave


# ----------------------------------------------------------------------------- dibujo
def dibujar(frame, caja, centro_cal, obj, robot, pausado, fps, rejilla, sistema):
    H, W = frame.shape[:2]
    if rejilla:
        for i in range(1, 4):
            cv2.line(frame, (W * i // 4, 0), (W * i // 4, H), (60, 60, 60), 1)
            cv2.line(frame, (0, H * i // 4), (W, H * i // 4), (60, 60, 60), 1)
    cxc, cyc = int(centro_cal[0] * W), int(centro_cal[1] * H)
    cv2.drawMarker(frame, (cxc, cyc), (255, 200, 0), cv2.MARKER_CROSS, 30, 2)
    if caja is not None:
        x1, y1, x2, y2 = map(int, caja)
        color = (0, 220, 0) if sistema.kalman.sin_detectar == 0 else (0, 200, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(frame, (cxc, cyc), (cx, cy), (0, 0, 255), 1)
    # panel
    cv2.rectangle(frame, (0, 0), (360, 118), (20, 20, 20), -1)
    det_ms = 1000 * np.mean(sistema.t_det) if sistema.t_det else 0
    estado = ("CONECTADO" if robot.conectado else "esperando a RobotStudio...") if robot.usar else "sin robot"
    lineas = [f"{fps:4.1f} FPS | det {det_ms:3.0f} ms | Kalman {'ON' if sistema.kalman.activo else '--'}",
              f"robot: {estado}" + (f" | {robot.latencia_ms:3.0f} ms" if robot.conectado else ""),
              "objetivo:  Y {:+6.0f} mm   Z {:+6.0f} mm".format(obj[1], obj[2]) if obj is not None else "objetivo: (sin mano)",
              "PAUSADO (p)" if pausado else "q salir  p pausa  c calibrar  h home"]
    for i, t in enumerate(lineas):
        col = (0, 140, 255) if (i == 3 and pausado) else (235, 235, 235)
        cv2.putText(frame, t, (8, 24 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description="El robot de RobotStudio sigue la mano (Faster R-CNN + Kalman)")
    ap.add_argument("--detector", default=os.path.join(AQUI, "detector_mano.pth"))
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--puerto", type=int, default=1025)
    ap.add_argument("--sin-robot", action="store_true", help="solo seguimiento en pantalla")
    ap.add_argument("--camara", type=int, default=0)
    ap.add_argument("--video", default=None, help="usar un vídeo en lugar de la webcam")
    ap.add_argument("--guardar", default=None, help="grabar la vista de la cámara en .mp4 (para el vídeo)")
    ap.add_argument("--cada", type=int, default=3, help="detectar la mano 1 de cada N fotogramas")
    ap.add_argument("--perdida", type=int, default=5)
    ap.add_argument("--umbral-det", type=float, default=0.6)
    ap.add_argument("--rango-y", type=float, default=500, help="mm que recorre el robot de borde a borde de la imagen")
    ap.add_argument("--rango-z", type=float, default=350)
    ap.add_argument("--lim-y", type=float, default=250, help="límite de seguridad en Y (mm desde HOME)")
    ap.add_argument("--lim-z", type=float, default=175)
    ap.add_argument("--alfa", type=float, default=0.5, help="suavizado (1 = sin suavizar)")
    ap.add_argument("--hilos", type=int, default=2,
                    help="hilos de CPU para la red (deja CPU libre para RobotStudio)")
    ap.add_argument("--invertir-y", action="store_true")
    args = ap.parse_args()

    # La red usaría todos los núcleos y RobotStudio simularía más lento que el tiempo real.
    torch.set_num_threads(max(1, args.hilos))
    sistema = SeguidorMano(args)
    robot = RobotStudio(args.ip, args.puerto, usar=not args.sin_robot)
    mapeo = Mapeo(args)

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.camara, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(args.camara)
    for _ in range(10):  # algunas webcams fallan la primera lectura
        if cap.isOpened() and cap.read()[0]:
            break
        time.sleep(0.2)
    if not cap.isOpened():
        raise SystemExit("No se puede abrir la cámara/vídeo")

    escritor, pausado, rejilla = None, False, True
    marcas = deque()
    print("Listo. Pon la mano delante de la cámara. Teclas: q p c h r")
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if not args.video:
            frame = cv2.flip(frame, 1)  # espejo: mueves la mano a la derecha y la ves a la derecha
        H, W = frame.shape[:2]

        caja = sistema.actualizar(frame)
        obj = None
        if caja is not None:
            u, v = (caja[0] + caja[2]) / 2 / W, (caja[1] + caja[3]) / 2 / H
            obj = mapeo(u, v)
            if not pausado:
                robot.mover(*obj)

        t = time.time(); marcas.append(t)
        while marcas and t - marcas[0] > 1.0:
            marcas.popleft()
        dibujar(frame, caja, mapeo.centro, obj, robot, pausado, len(marcas), rejilla, sistema)

        if args.guardar:
            if escritor is None:
                escritor = cv2.VideoWriter(args.guardar, cv2.VideoWriter_fourcc(*"mp4v"), 15, (W, H))
            escritor.write(frame)
        cv2.imshow("Seguir mano -> RobotStudio", frame)
        tecla = cv2.waitKey(1) & 0xFF
        if 65 <= tecla <= 90:
            tecla = ord(chr(tecla).lower())  # funciona aunque esté Bloq Mayús
        if tecla == ord("q"):
            break
        elif tecla == ord("p"):
            pausado = not pausado
        elif tecla == ord("c") and caja is not None:
            mapeo.calibrar((caja[0] + caja[2]) / 2 / W, (caja[1] + caja[3]) / 2 / H)
            print("Calibrado: esta posición de la mano es ahora HOME del robot")
        elif tecla == ord("h"):
            robot.home()
        elif tecla == ord("r"):
            rejilla = not rejilla

    cap.release()
    if escritor:
        escritor.release()
    robot.cerrar()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
