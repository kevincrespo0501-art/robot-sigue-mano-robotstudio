"""
modelos.py — Definición de las redes, COMPARTIDA entre Colab y la app local.

Clasificadores (6 clases: 0..5 dedos), entrada 3x64x64 (silueta replicada en 3 canales):
  - simplecnn         : CNN pequeña entrenada desde cero (línea base, como en P2)
  - resnet18_frozen   : ResNet18 ImageNet, solo se entrena la cabeza (transfer learning)
  - resnet18_finetune : ResNet18 ImageNet, se reentrenan layer3, layer4 y la cabeza (fine-tuning)
  - mobilenetv3       : MobileNetV3-Small ImageNet con fine-tuning (ligera, convs depthwise)

Detector de mano:
  - Faster R-CNN con backbone MobileNetV3-Large + FPN (misma familia que el de P3)
"""
import torch
import torch.nn as nn
from torchvision import models

NUM_CLASES = 6
MEDIA_IMAGENET = [0.485, 0.456, 0.406]
STD_IMAGENET = [0.229, 0.224, 0.225]


class SimpleCNN(nn.Module):
    """CNN desde cero inspirada en la de P2: (Conv-BN-ReLU)x2 + MaxPool, tres bloques."""

    def __init__(self, num_clases=NUM_CLASES, p_dropout=0.5):
        super().__init__()

        def bloque(cin, cout):
            return nn.Sequential(
                nn.Conv2d(cin, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                nn.Conv2d(cout, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
            )

        self.features = nn.Sequential(bloque(3, 32), bloque(32, 64), bloque(64, 128))
        self.pool = nn.AdaptiveAvgPool2d(1)  # global average pooling: vale para 64 o 112 px
        self.clasificador = nn.Sequential(
            nn.Flatten(), nn.Dropout(p_dropout), nn.Linear(128, 128),
            nn.ReLU(inplace=True), nn.Dropout(p_dropout), nn.Linear(128, num_clases),
        )

    def forward(self, x):
        return self.clasificador(self.pool(self.features(x)))


def crear_clasificador(nombre, preentrenado=True, num_clases=NUM_CLASES):
    """Devuelve el modelo indicado con la cabeza adaptada a num_clases."""
    if nombre == "simplecnn":
        return SimpleCNN(num_clases)

    if nombre.startswith("resnet18"):
        pesos = models.ResNet18_Weights.DEFAULT if preentrenado else None
        m = models.resnet18(weights=pesos)
        m.fc = nn.Linear(m.fc.in_features, num_clases)
        for p in m.parameters():
            p.requires_grad = False
        entrenables = [m.fc]
        if nombre == "resnet18_finetune":
            entrenables += [m.layer3, m.layer4]
        for mod in entrenables:
            for p in mod.parameters():
                p.requires_grad = True
        return m

    if nombre == "mobilenetv3":
        pesos = models.MobileNet_V3_Small_Weights.DEFAULT if preentrenado else None
        m = models.mobilenet_v3_small(weights=pesos)
        m.classifier[3] = nn.Linear(m.classifier[3].in_features, num_clases)
        return m  # fine-tuning completo (es pequeña)

    raise ValueError(f"Modelo desconocido: {nombre}")


def capa_gradcam(modelo, nombre):
    """Última capa convolucional, para Grad-CAM."""
    if nombre == "simplecnn":
        return modelo.features[-1]
    if nombre.startswith("resnet18"):
        return modelo.layer4
    if nombre == "mobilenetv3":
        return modelo.features[-1]
    raise ValueError(nombre)


def crear_detector(num_clases=2, preentrenado=True, min_size=480, max_size=640):
    """
    Faster R-CNN MobileNetV3-Large FPN (torchvision) con la cabeza cambiada a
    num_clases (fondo + mano). Misma API que el fasterrcnn_resnet50_fpn_v2 de P3.
    """
    from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_fpn
    from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

    pesos = "DEFAULT" if preentrenado else None
    m = fasterrcnn_mobilenet_v3_large_fpn(weights=pesos, weights_backbone=None,
                                          min_size=min_size, max_size=max_size)
    in_feat = m.roi_heads.box_predictor.cls_score.in_features
    m.roi_heads.box_predictor = FastRCNNPredictor(in_feat, num_clases)
    return m


def silueta_a_tensor(silueta, tam=64):
    """uint8 (H,W) 0/255 -> tensor (1,3,tam,tam) normalizado como ImageNet."""
    import cv2
    if silueta.shape[0] != tam:
        silueta = cv2.resize(silueta, (tam, tam), interpolation=cv2.INTER_NEAREST)
    t = torch.from_numpy(silueta).float().div(255.0)
    t = t.unsqueeze(0).repeat(3, 1, 1)
    media = torch.tensor(MEDIA_IMAGENET).view(3, 1, 1)
    std = torch.tensor(STD_IMAGENET).view(3, 1, 1)
    return ((t - media) / std).unsqueeze(0)
