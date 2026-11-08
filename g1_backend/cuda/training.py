"""Optional YOLO training device, preserving CPU arguments and fresh retry weights."""

from ..acceleration import disable, torch_device


def train_yolo(yolo_class, pretrained, kwargs):
    device = torch_device()
    model = yolo_class(str(pretrained))
    try:
        model.train(**kwargs, device=device)
    except (RuntimeError, MemoryError) as error:
        if device != 'cuda':
            raise
        disable('torch', error)
        device = 'cpu'
        # A failed training run may have mutated the model: restart from source weights.
        model = yolo_class(str(pretrained))
        model.train(**kwargs, device=device)
    return model, device
