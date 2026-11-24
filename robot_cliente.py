"""
robot_cliente.py — Cliente TCP para el módulo RAPID SeguirMano.mod (RobotStudio).

Protocolo (texto, una petición y una respuesta):
    Python -> robot :  "M;dx;dy;dz;"   desplazamiento en mm respecto a la posición HOME del robot
                       "H;"            volver a HOME
    robot  -> Python:  "OK"            cuando ya ha aceptado el movimiento

Se usa petición-respuesta: Python solo envía un objetivo nuevo cuando el robot ha contestado
al anterior. Así no se acumulan mensajes en el socket y el robot siempre va al punto MÁS
RECIENTE de la mano (si no, el retardo crecería sin parar).
"""
import socket
import threading
import time


class RobotStudio:
    def __init__(self, ip="127.0.0.1", puerto=1025, usar=True, umbral_mm=3.0):
        self.ip, self.puerto, self.usar, self.umbral = ip, puerto, usar, umbral_mm
        self.objetivo = None          # último objetivo pedido (dx, dy, dz)
        self.enviado = None           # último objetivo que aceptó el robot
        self.pedir_home = False
        self.conectado = False
        self.latencia_ms = 0.0
        self._sock = None
        self._evento = threading.Event()
        self._parar = False
        if usar:
            threading.Thread(target=self._bucle, daemon=True).start()

    # ------------------------------------------------------------------ API
    def mover(self, dx, dy, dz=0.0):
        self.objetivo = (float(dx), float(dy), float(dz))
        self._evento.set()

    def home(self):
        self.pedir_home = True
        self._evento.set()

    def cerrar(self):
        self._parar = True
        self._evento.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass

    # ------------------------------------------------------------------ hilo
    def _conectar(self):
        while not self._parar:
            try:
                s = socket.create_connection((self.ip, self.puerto), timeout=2)
                s.settimeout(10)
                self._sock, self.conectado = s, True
                print(f"[robot] Conectado a RobotStudio en {self.ip}:{self.puerto}")
                return
            except OSError:
                self.conectado = False
                time.sleep(1.0)   # el programa RAPID aún no está esperando: reintentar

    def _peticion(self, texto):
        t0 = time.time()
        self._sock.sendall(texto.encode("ascii"))
        resp = b""
        while b"OK" not in resp:
            trozo = self._sock.recv(64)
            if not trozo:
                raise ConnectionError("RobotStudio cerró la conexión")
            resp += trozo
        self.latencia_ms = (time.time() - t0) * 1000

    def _cambio_suficiente(self, obj):
        if self.enviado is None:
            return True
        return max(abs(a - b) for a, b in zip(obj, self.enviado)) >= self.umbral

    def _bucle(self):
        self._conectar()
        while not self._parar:
            self._evento.wait(0.05)
            self._evento.clear()
            try:
                if self.pedir_home:
                    self.pedir_home = False
                    self._peticion("H;")
                    self.enviado = (0.0, 0.0, 0.0)
                    continue
                obj = self.objetivo
                if obj is not None and self._cambio_suficiente(obj):
                    self._peticion("M;{:.1f};{:.1f};{:.1f};".format(*obj))
                    self.enviado = obj
            except OSError as e:
                print("[robot] Conexión perdida:", e, "-> reconectando…")
                self.conectado = False
                try:
                    self._sock.close()
                except OSError:
                    pass
                self.enviado = None
                self._conectar()
