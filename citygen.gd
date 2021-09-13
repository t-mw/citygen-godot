extends Node2D

# TODO: remove r?
# TODO: use quadtree?
# TODO: cross product necessary in Segment.direction_get?

const BRANCH_ANGLE_DEVIATION := 3.0
const STRAIGHT_ANGLE_DEVIATION := 15.0
const DEFAULT_SEGMENT_WIDTH := 5
const HIGHWAY_SEGMENT_WIDTH := 20
const DEFAULT_SEGMENT_LENGTH := 300
const HIGHWAY_SEGMENT_LENGTH := 400
const DEFAULT_BRANCH_PROBABILITY := 0.4
const HIGHWAY_BRANCH_PROBABILITY := 0.05
const HIGHWAY_BRANCH_POPULATION_THRESHOLD := 0.1
const NORMAL_BRANCH_POPULATION_THRESHOLD := 0.1
const NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY := 5
const SEGMENT_COUNT_LIMIT := 2000

func random_branch_angle() -> float:
    return Math.random_angle(BRANCH_ANGLE_DEVIATION)

func random_straight_angle() -> float:
    return Math.random_angle(STRAIGHT_ANGLE_DEVIATION)

# Called when the node enters the scene tree for the first time.
func _ready():
    randomize()
    generate()

func _draw():
    var segments = generate()

    var viewport := get_viewport()
    var camera := $Camera2D
    var viewport_rect := viewport.get_visible_rect()
    var global_to_viewport: Transform2D = viewport.global_canvas_transform * camera.get_canvas_transform()
    var viewport_to_global: Transform2D = global_to_viewport.affine_inverse()
    var viewport_rect_global: Rect2 = viewport_to_global.xform(viewport_rect)

    draw_rect(viewport_rect_global, Color.gray, true)
    for segment in segments:
        var width = HIGHWAY_SEGMENT_WIDTH if segment.metadata.highway else DEFAULT_SEGMENT_WIDTH
        draw_line(segment.r_start, segment.r_end, Color.black, width, true)

func generate():
    var segments := []
    var priority_q := []

    var root_metadata := SegmentMetadata.new();
    root_metadata.highway = true;
    var root_segment := Segment.new(Vector2(0, 0), Vector2(HIGHWAY_SEGMENT_LENGTH, 0), 0, root_metadata)

    var opposite_direction := root_segment.clone()
    var new_end := Vector2(root_segment.r_start.x - HIGHWAY_SEGMENT_LENGTH, opposite_direction.r_end.y);
    opposite_direction.set_r_end(new_end)
    opposite_direction.links_b.append(root_segment)
    root_segment.links_b.append(opposite_direction)
    priority_q.append(root_segment)
    priority_q.append(opposite_direction)

    while len(priority_q) > 0 && len(segments) < SEGMENT_COUNT_LIMIT:
        # pop smallest r(ti, ri, qi) from Q (i.e., smallest ‘t’)
        var min_t = null
        var min_t_i: int = 0
        for i in len(priority_q):
            var segment: Segment = priority_q[i]
            if min_t == null || segment.t < min_t:
                min_t = segment.t
                min_t_i = i

        var min_segment: Segment = priority_q[min_t_i]
        priority_q.remove(min_t_i)

        var accepted := local_constraints(min_segment, segments)
        if accepted:
            min_segment.setup_branch_links()
            segments.append(min_segment)
            for new_segment in global_goals_generate(min_segment):
                new_segment.t = min_segment.t + 1 + new_segment.t
                priority_q.append(new_segment)

    return segments

func local_constraints(segment: Segment, segments: Array) -> bool:
    return true

func global_goals_generate(previous_segment: Segment) -> Array:
    var new_branches = []
    if !previous_segment.metadata.severed:
        var template = GlobalGoalsTemplate.new(previous_segment)

        var continue_straight = template.segment_continue(previous_segment.direction)
        var straight_pop = 10 # TODO: heatmap.popOnRoad(continueStraight.r)

        if previous_segment.metadata.highway:
            var random_straight = template.segment_continue(previous_segment.direction + random_straight_angle())
            var random_pop = 10 # TODO: heatmap.popOnRoad(randomStraight.r)
            var road_pop = null
            if random_pop > straight_pop:
                new_branches.append(random_straight)
                road_pop = random_pop
            else:
                new_branches.append(continue_straight)
                road_pop = straight_pop
            if road_pop > HIGHWAY_BRANCH_POPULATION_THRESHOLD:
                if randf() < HIGHWAY_BRANCH_PROBABILITY:
                    var left_highway_branch = template.segment_continue(previous_segment.direction - 90 + random_branch_angle())
                    new_branches.append(left_highway_branch)
                elif randf() < HIGHWAY_BRANCH_PROBABILITY:
                    var right_highway_branch = template.segment_continue(previous_segment.direction + 90 + random_branch_angle())
                    new_branches.append(right_highway_branch)
        elif straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD:
            new_branches.append(continue_straight)
        if straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD:
            if randf() < DEFAULT_BRANCH_PROBABILITY:
                var left_branch = template.segment_branch(previous_segment.direction - 90 + random_branch_angle())
                new_branches.append(left_branch)
            elif randf() < DEFAULT_BRANCH_PROBABILITY:
                var right_branch = template.segment_branch(previous_segment.direction + 90 + random_branch_angle())
                new_branches.append(right_branch)

    for branch in new_branches:
        branch.previous_segment_to_link = previous_segment

    return new_branches

class GlobalGoalsTemplate:
    var previous_segment: Segment

    func _init(_previous_segment: Segment):
        self.previous_segment = _previous_segment

    func segment(direction: float, length: float, t: int, metadata: SegmentMetadata) -> Segment:
        return Segment.new_using_direction(self.previous_segment.end, direction, length, t, metadata)

    # used for highways or going straight on a normal branch
    func segment_continue(direction: float) -> Segment:
        return self.segment(direction, self.previous_segment.length, 0, self.previous_segment.metadata)

    # used for branches extending from highways i.e. not highways themselves
    func segment_branch(direction: float) -> Segment:
        var t := NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY if self.previous_segment.metadata.highway else 0
        return self.segment(direction, DEFAULT_SEGMENT_LENGTH, t, SegmentMetadata.new())

class Segment:
    var start: Vector2
    var end: Vector2

    # time-step delay before this segment is evaluated
    var t: int

    # meta-information relevant to global goals
    var metadata: SegmentMetadata

    # representation of road
    var r_start: Vector2 setget set_r_start
    var r_end: Vector2 setget set_r_end
    var r_revision: int = 0

    # links backwards and forwards
    var links_b: Array = [] # [Segment]
    var links_f: Array = [] # [Segment]

    var previous_segment_to_link = null

    var direction: float setget ,get_direction
    var direction_revision: int = -1
    var length: float setget ,get_length
    var length_revision: int = -1

    func set_r_start(v: Vector2):
        r_start = v
        # TODO: obj.collider.updateCollisionProperties({start: @start})
        self.r_revision += 1

    func set_r_end(v: Vector2):
        r_end = v
        # TODO: obj.collider.updateCollisionProperties({end: @end})
        self.r_revision += 1

    func get_direction() -> float:
        if self.direction_revision != self.r_revision:
            self.direction_revision = self.r_revision
            var vec = self.r_end - self.r_start
            direction = -1 * sign(Vector2(0, 1).cross(vec)) * Math.angle_between(Vector2(0, 1), vec)
        return direction

    func get_length() -> float:
        if self.length_revision != self.r_revision:
            self.length_revision = self.r_revision
            length = (self.r_end - self.r_start).length()
        return length

    func _init(_start: Vector2, _end: Vector2, _t: int, _metadata: SegmentMetadata):
        self.start = _start
        self.end = _end
        self.t = _t
        self.metadata = _metadata
        self.r_start = _start
        self.r_end = _end

    static func new_using_direction(_start: Vector2, _direction: float, _length: float, _t: int, _metadata: SegmentMetadata) -> Segment:
        var new_end := Vector2(_start.x + _length * sin(deg2rad(_direction)), _start.y + _length * cos(deg2rad(_direction)))
        return Segment.new(_start, new_end, _t, _metadata)

    func intersection_with(other: Segment):
        return Geometry.segment_intersects_segment_2d(self.r_start, self.r_end, other.r_start, other.r_end)

    func links_for_end_containing(segment: Segment):
        if self.links_b.has(segment):
            return self.links_b
        elif self.links_f.has(segment):
            return self.links_f
        else:
            return null

    func setup_branch_links():
        if self.previous_segment_to_link == null:
            return
        # setup links between each current branch and each existing branch stemming from the previous segment
        for link in self.previous_segment_to_link.links_f:
            self.links_b.append(link)
            link.links_for_end_containing(self.previous_segment_to_link).append(self)
        self.previous_segment_to_link.links_f.append(self)
        self.links_b.append(self.previous_segment_to_link)

    func clone() -> Segment:
        return Segment.new(self.r_start, self.r_end, self.t, self.metadata.clone())

class SegmentMetadata:
    var highway: bool = false
    var severed: bool = false

    func clone() -> SegmentMetadata:
        var clone = SegmentMetadata.new()
        clone.highway = self.highway
        clone.severed = self.severed
        return clone

class Math:
    static func random_angle(limit: float) -> float:
        # non-linear distribution
        var non_uniform_norm := pow(limit, 3)
        var val: float = 0
        while val == 0 || randf() < pow(val, 3) / non_uniform_norm:
            val = rand_range(-limit, +limit)
        return val

    static func angle_between(v1: Vector2, v2: Vector2) -> float:
        var angle_rad = acos((v1.x * v2.x + v1.y * v2.y) / (v1.length() * v2.length()))
        return rad2deg(angle_rad)
