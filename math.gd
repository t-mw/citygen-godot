static func random_angle(limit: float) -> float:
    # non-linear distribution
    var non_uniform_norm := pow(limit, 3)
    var val: float = 0
    while val == 0 || randf() < pow(val, 3) / non_uniform_norm:
        val = rand_range(-limit, +limit)
    return val

static func min_degree_difference(d1: float, d2: float) -> float:
    var diff := fmod(abs(d1 - d2), 180)
    return min(diff, abs(diff - 180))

static func is_point_in_segment_range(point: Vector2, segment_start: Vector2, segment_end: Vector2) -> bool:
    var vec := segment_end - segment_start
    var dot := (point - segment_start).dot(vec)
    return dot >= 0 && dot <= vec.length_squared()
