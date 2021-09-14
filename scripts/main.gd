extends Node2D

const CityGen = preload("res://scripts/city_gen.gd")

onready var city_gen: CityGen = $"../CityGen"
onready var population_heatmap = $"../PopulationHeatmap"

var generated_segments = []
var generated_buildings = []

func _ready():
    randomize()
    run()

func _draw():
    for segment in generated_segments:
        var width = 25 if segment.metadata.highway else 5
        draw_line(segment.start, segment.end, Color8(161, 175, 165), width, true)
    for building in generated_buildings:
        var corners = building.generate_corners()
        draw_colored_polygon(corners, Color8(12, 22, 31), [], null, null, true)

func run():
    for segment in generated_segments:
        segment.free()
    for building in generated_buildings:
        building.free()

    city_gen.randomize_heatmap()
    generated_segments = city_gen.generate_segments()
    generated_buildings = city_gen.generate_buildings(generated_segments)

    # trigger redraw
    update()

func _on_GenerateButton_pressed():
    run()

func _on_HeatmapCheckbox_toggled(button_pressed):
    population_heatmap.visible = button_pressed
