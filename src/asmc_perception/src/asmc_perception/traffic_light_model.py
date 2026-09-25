"""Inference-only ResNet18 definition for the canonical traffic-light node."""

from __future__ import annotations

import os

import torch
import torch.nn as nn
from torchvision import models, transforms


CLASS_NAMES = [
    "red",
    "yellow",
    "green",
    "red_yellow",
    "red_left",
    "green_left",
    "left",
    "unknown",
]
NUM_CLASSES = len(CLASS_NAMES)
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]
NOT_DETECTED = "not_detected"


def decide(probs, classes, min_detect=0.40):
    best_i, best_p = -1, -1.0
    for index, label in enumerate(classes):
        if label == "unknown":
            continue
        probability = float(probs[index])
        if probability > best_p:
            best_p, best_i = probability, index
    if best_i < 0 or best_p <= min_detect:
        return NOT_DETECTED, best_p
    return classes[best_i], best_p


def build_transforms(size, aspect=4.0 / 3.0):
    height = size
    width = int(round(size * aspect))
    return transforms.Compose(
        [
            transforms.Resize((height, width)),
            transforms.ToTensor(),
            transforms.Normalize(IMAGENET_MEAN, IMAGENET_STD),
        ]
    )


def build_model():
    model = models.resnet18(weights=None)
    model.fc = nn.Linear(model.fc.in_features, NUM_CLASSES)
    return model


def resolve_checkpoint(path):
    candidate = os.path.expanduser(path)
    if os.path.isfile(candidate):
        return os.path.abspath(candidate)
    raise FileNotFoundError("traffic-light checkpoint not found: %s" % path)


def load_model(checkpoint, device):
    path = resolve_checkpoint(checkpoint)
    state = torch.load(path, map_location=device, weights_only=False)
    classes = state.get("classes", CLASS_NAMES)
    size = int(state.get("args", {}).get("size", 224))
    model = build_model()
    model.load_state_dict(state["model"])
    model.to(device).eval()
    return model, build_transforms(size), classes, size, state.get("val_acc", -1)
