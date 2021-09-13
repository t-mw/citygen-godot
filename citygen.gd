extends Node2D

# Called when the node enters the scene tree for the first time.
func _ready():
    pass # Replace with function body.

# Called every frame. 'delta' is the elapsed time since the previous frame.
#func _process(delta):
#    pass

func _draw():
    var viewport_rect = get_viewport().get_visible_rect()
    draw_rect(viewport_rect, Color.green, true, 0.0, true)
    draw_line(Vector2(0, 0), Vector2(500, 500), Color.black, 5.0, true)
