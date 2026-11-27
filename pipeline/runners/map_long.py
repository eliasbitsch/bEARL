"""Runner for Map-Long (Elias Bitsch, RGB deep dive): the VGGT-Long pipeline with MapAnything as the model.

Same container, input and output handling as pipeline/runners/vggt_long.py; only config/systems/map_long.yaml
differs (model: Mapanything, SE(3) chunk alignment as the upstream config prescribes for metric models).
Weights: $VGGT_LONG_WEIGHTS/map_anything = snapshot of huggingface.co/facebook/map-anything (CC BY-NC 4.0).
"""
from pipeline.runners.vggt_long import run  # noqa: F401
