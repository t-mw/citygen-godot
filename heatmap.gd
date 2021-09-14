extends Sprite

onready var noise: OpenSimplexNoise = texture.noise
var xform_inv: Transform2D

func _ready():
    var xform = get_global_transform().translated(get_rect().position)
    xform_inv = xform.affine_inverse()

func sample(global_pos: Vector2) -> float:
    # NB: in godot 4.0, x and y should be swapped i.e. this will be get_noise_2d(pos.x, pos.y)
    var local_pos = xform_inv.xform(global_pos)
    return noise.get_noise_2d(local_pos.y, local_pos.x)
