# Robot que sigue tu mano · Faster R-CNN + Kalman + RobotStudio

📺 **Vídeo paso a paso:** https://youtu.be/grM44AcW-7k  
📱 **Short:** https://youtube.com/shorts/D45FG7RmtRk  
Canal **KG Control** · del PLC al robot inteligente.

> **Sobre `detector_mano.pth` (76 MB):** no está en el repositorio porque supera el límite de subida de GitHub por web.
> Puedes entrenarlo tú con `02_detector_mano.ipynb` en Google Colab (GPU T4, unos 20–30 min) y copiarlo junto a los scripts.

| Archivo | Qué es |
|---|---|
| `02_detector_mano.ipynb` | Entrenamiento del detector Faster R-CNN con HaGRID (Colab) |
| `modelos.py` | Definición del detector (y de los clasificadores del proyecto original) |
| `seguir_mano_robot.py` | Programa principal: webcam → detector → Kalman → mm → robot |
| `robot_cliente.py` | Cliente TCP con pregunta-respuesta |
| `probar_robot.py` | Prueba sin cámara: cuadrado y círculo |
| `SeguirMano.mod` | Módulo RAPID para el controlador (servidor TCP + bucle de seguimiento) |
| `1_probar_robot.bat`, `2_seguir_mano.bat` | Lanzadores para Windows |

---

# El robot de RobotStudio sigue tu mano

Usa el mismo detector **Faster R-CNN** y el mismo **filtro de Kalman** del proyecto de conteo de dedos. Aquí no se cuentan dedos: lo que importa es **dónde está la mano**. Python envía esa posición por TCP a un programa RAPID, y el robot del controlador virtual la sigue en el plano vertical (Y-Z).

```
webcam -> Faster R-CNN (1 de cada 3 fotogramas) -> Kalman (todos) -> centro de la mano
       -> suavizado -> mm respecto a HOME -> TCP "M;dx;dy;dz;" -> RAPID SeguirMano.mod -> MoveL
```

## Archivos

| Archivo | Qué hace |
|---|---|
| `seguir_mano_robot.py` | Programa principal: webcam, detector, Kalman, mapeo a mm y envío al robot |
| `robot_cliente.py` | Cliente TCP. Funciona por petición-respuesta: siempre manda la posición más reciente, sin cola de mensajes |
| `probar_robot.py` | Prueba sin cámara: el robot dibuja un cuadrado y un círculo |
| `SeguirMano.mod` | Servidor TCP en el robot. Recibe los objetivos y hace `MoveL` |
| `modelos.py`, `detector_mano.pth` | El detector del proyecto (Faster R-CNN MobileNetV3-FPN ajustado con HaGRID) |

## Paso 1 · RobotStudio

1. Crea una estación con un robot pequeño (por ejemplo, un IRB 1200 o un IRB 120) y su controlador virtual: **Inicio › Sistema de robot › Desde diseño**.
   - Con **RobotWare 6 (IRC5)**, al crear el sistema añade la opción **616-1 PC Interface**, porque sin ella no hay sockets.
   - Con **RobotWare 7 (OmniCore)**, si `SocketCreate` da error, añade la opción **Socket Messaging** en el editor de opciones.
2. La postura HOME está definida en el `.mod` (`jHome`, pensada para el GoFa CRB 15000). Al pulsar Play, el robot va primero a esa postura. Para usar la posición en la que esté el robot, pon `USAR_JHOME := FALSE`.
3. Ve a **RAPID › Cargar módulo** y elige `SeguirMano.mod`. Si ya hay un `main` en otro módulo, bórralo o renómbralo.
4. Pulsa **Aplicar**, luego **PP a Main** y después **Iniciar** (Play). En el FlexPendant debe aparecer «Esperando a Python en el puerto 1025».
5. Para grabar el vídeo, quita **RAPID › Puntero de programa › Seguir puntero de programa**. Así la vista 3D no salta al editor en cada movimiento.

## Paso 2 · Probar la conexión (sin cámara)

Haz doble clic en **`1_probar_robot.bat`** (o ejecuta `python probar_robot.py`).

El robot tiene que dibujar un cuadrado de 200 mm y un círculo, y volver a HOME.

## Paso 3 · Seguir la mano

Haz doble clic en **`2_seguir_mano.bat`** (o ejecuta `python seguir_mano_robot.py`).

Pon la mano delante de la cámara. Cuando la mueves a la derecha o a la izquierda, el robot se mueve en Y; cuando la subes o la bajas, se mueve en Z.

| Tecla | Acción |
|---|---|
| `c` | Calibrar: la posición actual de la mano pasa a ser el centro (HOME del robot) |
| `p` | Pausar o reanudar el robot. La cámara sigue funcionando |
| `h` | Mandar el robot a HOME (mejor con el robot en pausa) |
| `r` | Mostrar u ocultar la rejilla |
| `q` | Salir |

Opciones útiles:

```
python seguir_mano_robot.py --sin-robot                 # solo ver el seguimiento
python seguir_mano_robot.py --invertir-y                # si el robot va al revés en horizontal
python seguir_mano_robot.py --rango-y 700 --rango-z 450 # movimientos más amplios
python seguir_mano_robot.py --alfa 0.6                  # más rápido y menos suave
python seguir_mano_robot.py --guardar camara.mp4        # grabar la vista de la cámara para el vídeo
```

## Seguridad y límites

- Python limita el desplazamiento a ±250 mm en Y y ±175 mm en Z.
- RAPID vuelve a limitarlo a ±300 mm en Y y ±200 mm en Z. Así, aunque llegue un dato erróneo, el robot no sale de esa caja alrededor de HOME.
- Si sale el error *«Position outside reach»*, coloca HOME más cerca del centro del área de trabajo o reduce `LIM_Y` y `LIM_Z` en el `.mod`.

## Por qué funciona fluido

- **Kalman:** el detector solo se ejecuta 1 de cada 3 fotogramas. Entre medias, el Kalman predice la caja, así que la posición se actualiza en todos los fotogramas.
- **Suavizado exponencial (`--alfa`):** quita el temblor natural de la mano.
- **Petición-respuesta:** Python no envía otro objetivo hasta que el robot responde «OK». No se acumulan órdenes y el retardo no crece.
- **Bucle de seguimiento en RAPID:** el robot contesta OK al momento y avanza en pasos de 40 mm (zona z10) hacia la posición más reciente: la latencia bajó de 9,7 s a unos 3 ms.
- **Umbral de 3 mm:** si la mano casi no se mueve, no se envía nada.

## Avisos normales en RobotStudio

- **50024 Fallo de trayectoria de esquina:** aparece cuando tu mano se queda quieta y el robot termina el punto antes de recibir el siguiente. Es inofensivo.
- **41581 / 41595 Error de zócalo:** aparece cuando no llega mensaje en 20 ms o al cerrar Python. El programa lo gestiona y vuelve a esperar una nueva conexión.
