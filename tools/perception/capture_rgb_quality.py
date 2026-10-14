"""Conservative checks for observed Capture render corruption patterns."""


def rgb_metrics(image):
    rgb = image.convert('RGB')
    constant = all(lo == hi for lo, hi in rgb.getextrema())
    colors = rgb.getcolors(maxcolors=256)
    binary = None
    if colors is not None:
        binary = sum(n for n, color in colors
                     if color in ((0, 0, 0), (255, 255, 255))) / float(rgb.width * rgb.height)
    return dict(constant=constant, palette_at_most_256=colors is not None,
                binary_fraction=binary, invalid_binary_palette=binary is not None and binary >= .98)


def validate_rendered_rgb(image):
    metrics = rgb_metrics(image)
    if metrics['constant']:
        raise ValueError('constant RGB texture: Capture render failure, not a negative')
    if metrics['invalid_binary_palette']:
        raise ValueError('binary low-palette RGB texture: Capture render failure, not a negative')
