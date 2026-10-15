"""Semantic inference backend registry."""

PINNED_TWINLITE_COMMIT = "90f1b8695ae311d5123b05f8534b2e11e42499d2"


def create_backend(name, **kwargs):
    normalized = str(name).strip().lower()
    if normalized == "twinlite":
        from .twinlite import TwinLiteBackend

        return TwinLiteBackend(**kwargs)
    if normalized == "twinlite_tensorrt":
        from .twinlite_tensorrt import TwinLiteTensorRTBackend

        return TwinLiteTensorRTBackend(**kwargs)
    raise ValueError("unsupported semantic backend: {}".format(name))


__all__ = ["create_backend"]
