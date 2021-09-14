extends Node2D

const BRANCH_ANGLE_DEVIATION := 3.0 # degrees
const STRAIGHT_ANGLE_DEVIATION := 15.0 # degrees
const MINIMUM_INTERSECTION_DEVIATION := 30 # degrees
const DEFAULT_SEGMENT_WIDTH := 5 # pixels
const HIGHWAY_SEGMENT_WIDTH := 20 # pixels
const DEFAULT_SEGMENT_LENGTH := 300 # world units
const HIGHWAY_SEGMENT_LENGTH := 400 # world units
const DEFAULT_BRANCH_PROBABILITY := 0.4
const HIGHWAY_BRANCH_PROBABILITY := 0.05
const HIGHWAY_BRANCH_POPULATION_THRESHOLD := 0.1
const NORMAL_BRANCH_POPULATION_THRESHOLD := 0.1
const NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY := 5
const SEGMENT_COUNT_LIMIT := 2000
const ROAD_SNAP_DISTANCE := 50 # world units

const BUILDING_COUNT_PER_SEGMENT := 10
const MAX_BUILDING_DISTANCE_FROM_SEGMENT := 400.0 # world units
const BUILDING_PLACEMENT_LOOP_LIMIT := 3

func random_branch_angle() -> float:
    return Math.random_angle(BRANCH_ANGLE_DEVIATION)

func random_straight_angle() -> float:
    return Math.random_angle(STRAIGHT_ANGLE_DEVIATION)

const Building = preload("res://building.gd")
const Heatmap = preload("res://heatmap.gd")
const Math = preload("res://math.gd")
const segment_mod = preload("res://segment.gd")
const Segment = segment_mod.Segment
const SegmentMetadata = segment_mod.SegmentMetadata

onready var population_heatmap: Heatmap = $"../PopulationHeatmap"
onready var physics_space := get_world_2d().direct_space_state
onready var physics_space_rid := get_world_2d().space

var generated_segments = []
var generated_buildings = []

func _ready():
    randomize()

    var noise = population_heatmap.noise
    noise.seed = randi()
    noise.octaves = 4
    noise.period = 10.0
    noise.persistence = 0.2

    generated_segments = generate_segments()
    generated_buildings = generate_buildings(generated_segments)

func _draw():
    for segment in generated_segments:
        var width = HIGHWAY_SEGMENT_WIDTH if segment.metadata.highway else DEFAULT_SEGMENT_WIDTH
        draw_line(segment.start, segment.end, Color.black, width, true)
    for building in generated_buildings:
        draw_colored_polygon(building.generate_corners(), Color.black, [], null, null, true)

func generate_segments() -> Array:
    var segments := []
    var priority_q := []

    var root_metadata := SegmentMetadata.new();
    root_metadata.highway = true;
    var root_segment := Segment.new(Vector2(0, 0), Vector2(HIGHWAY_SEGMENT_LENGTH, 0), 0, root_metadata)

    var opposite_direction := root_segment.clone()
    var new_end := Vector2(root_segment.start.x - HIGHWAY_SEGMENT_LENGTH, opposite_direction.end.y);
    opposite_direction.end = new_end
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
            min_segment.attach_to_physics_space(physics_space_rid)
            segments.append(min_segment)
            for new_segment in global_goals_generate(min_segment):
                new_segment.t = min_segment.t + 1 + new_segment.t
                priority_q.append(new_segment)

    return segments

func sample_population(start: Vector2, end: Vector2) -> float:
    return (population_heatmap.sample(start) + population_heatmap.sample(end)) * 0.5

func local_constraints(segment: Segment, segments: Array) -> bool:
    var action = null
    var action_priority = 0
    var previous_intersection_distance_squared = null

    # filter potential colliders with physics query
    var query = Physics2DShapeQueryParameters.new()
    query.collide_with_bodies = false
    query.collide_with_areas = true
    query.shape_rid = segment.create_physics_shape()

    var matches = []
    var query_results = physics_space.intersect_shape(query)
    for result in query_results:
        var other = result.collider as Segment
        assert(other != null)
        matches.append(other)

    segment.destroy_physics_shape()

    for other in matches:
        if segment == other:
            continue

        # intersection check
        if action_priority <= 4:
            var intersection = segment.intersection_with(other)
            if intersection != null:
                var intersection_distance_squared := segment.start.distance_squared_to(intersection)
                if previous_intersection_distance_squared == null || intersection_distance_squared < previous_intersection_distance_squared:
                    previous_intersection_distance_squared = intersection_distance_squared
                    action_priority = 4
                    action = LocalConstraintsIntersectionAction.new(other, intersection)

        # snap to crossing within radius check
        if action_priority <= 3:
            # current segment's start must have been checked to have been created.
            # other segment's start must have a corresponding end.
            if segment.end.distance_squared_to(other.end) <= ROAD_SNAP_DISTANCE * ROAD_SNAP_DISTANCE:
                action_priority = 3
                action = LocalConstraintsSnapAction.new(other, other.end)

        # intersection within radius check
        if action_priority <= 2:
            if Math.is_point_in_segment_range(segment.end, other.start, other.end):
                var intersection := Geometry.get_closest_point_to_segment_2d(segment.end, other.start, other.end)
                var distance_squared := segment.end.distance_squared_to(intersection)
                if distance_squared < ROAD_SNAP_DISTANCE * ROAD_SNAP_DISTANCE:
                    action_priority = 2
                    action = LocalConstraintsIntersectionRadiusAction.new(other, intersection)

    if action != null:
        return action.apply(segment, segments)

    return true

class LocalConstraintsIntersectionAction:
    const Segment = preload("res://segment.gd").Segment

    var other: Segment
    var intersection: Vector2

    func _init(_other: Segment, _intersection: Vector2):
        self.other = _other
        self.intersection = _intersection

    func apply(segment: Segment, segments: Array) -> bool:
        # if intersecting lines are too similar don't continue
        if Math.min_degree_difference(self.other.direction, segment.direction) < MINIMUM_INTERSECTION_DEVIATION:
            return false
        self.other.split(self.intersection, segment, segments)
        segment.end = self.intersection
        segment.metadata.severed = true
        return true

class LocalConstraintsSnapAction:
    const Segment = preload("res://segment.gd").Segment

    var other: Segment
    var point: Vector2

    func _init(_other: Segment, _point: Vector2):
        self.other = _other
        self.point = _point

    func apply(segment: Segment, _segments: Array) -> bool:
        segment.end = self.point
        segment.metadata.severed = true

        # update links of other segment corresponding to other.end
        var links := self.other.links_f if self.other.start_is_backwards() else self.other.links_b

        # check for duplicate lines, don't add if it exists
        for link in links:
            if ((link.start.is_equal_approx(segment.end) && link.end.is_equal_approx(segment.start)) ||
                (link.start.is_equal_approx(segment.start) && link.end.is_equal_approx(segment.end))):
                return false

        for link in links:
            # pick links of remaining segments at junction corresponding to other.end
            link.links_for_end_containing(self.other).append(segment)
            # add junction segments to snapped segment
            segment.links_f.append(link)

        links.append(segment)
        segment.links_f.append(self.other)

        return true

class LocalConstraintsIntersectionRadiusAction:
    const Segment = preload("res://segment.gd").Segment

    var other: Segment
    var intersection: Vector2

    func _init(_other: Segment, _intersection: Vector2):
        self.other = _other
        self.intersection = _intersection

    func apply(segment: Segment, segments: Array) -> bool:
        segment.end = self.intersection
        segment.metadata.severed = true
        # if intersecting lines are too similar don't continue
        if Math.min_degree_difference(self.other.direction, segment.direction) < MINIMUM_INTERSECTION_DEVIATION:
            return false
        self.other.split(self.intersection, segment, segments)
        return true

func global_goals_generate(previous_segment: Segment) -> Array:
    var new_branches = []
    if !previous_segment.metadata.severed:
        var template := GlobalGoalsTemplate.new(previous_segment)

        var continue_straight = template.segment_continue(previous_segment.direction)
        var straight_pop = sample_population(continue_straight.start, continue_straight.end)

        if previous_segment.metadata.highway:
            var random_straight = template.segment_continue(previous_segment.direction + random_straight_angle())
            var random_pop = sample_population(random_straight.start, random_straight.end)
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
    const segment_mod = preload("res://segment.gd")
    const Segment = segment_mod.Segment
    const SegmentMetadata = segment_mod.SegmentMetadata

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

const SEGMENT_STEP := 10
func generate_buildings(segments: Array) -> Array:
    var buildings = []

    for i in range(0, len(segments), SEGMENT_STEP):
        var segment: Segment = segments[i]

        for _b in range(0, BUILDING_COUNT_PER_SEGMENT):
            var random_angle = randf() * 360.0
            var random_radius = randf() * MAX_BUILDING_DISTANCE_FROM_SEGMENT

            var building = Building.new()
            building.center = (segment.start + segment.end) * 0.5
            building.center.x += random_radius * sin(deg2rad(random_angle))
            building.center.y += random_radius * cos(deg2rad(random_angle))
            building.direction = segment.direction
            building.aspect_ratio = rand_range(0.5, 2.0)
            building.diagonal = rand_range(80.0, 150.0)

            var permit_building = false
            var query = Physics2DShapeQueryParameters.new()
            query.collide_with_bodies = false
            query.collide_with_areas = true
            for placement_i in range(0, BUILDING_PLACEMENT_LOOP_LIMIT):
                query.shape_rid = building.create_physics_shape()
                var query_results = physics_space.collide_shape(query)

                var is_final_placement_iteration = placement_i == BUILDING_PLACEMENT_LOOP_LIMIT - 1
                if len(query_results) == 0:
                    permit_building = true
                    break
                elif is_final_placement_iteration:
                    # there is no point checking the query results if we are on the final iteration
                    # and there are still collisions
                    break

                for result in query_results:
                    # move building away from collision
                    building.center += (building.center - result)
                building.destroy_physics_shape()

            if permit_building:
                building.attach_to_physics_space(physics_space_rid)
                buildings.append(building)

    return buildings
