extends Object

var physics_shape: RID
var physics_area: RID

var center: Vector2
var direction: float
var aspect_ratio: float
var diagonal: float
var corners: PoolVector2Array

func _notification(n):
    if n == NOTIFICATION_PREDELETE:
        self.detach_from_physics_space()

func create_physics_shape() -> RID:
    if self.physics_shape.get_id() == 0:
        self.physics_shape = Physics2DServer.convex_polygon_shape_create()
        Physics2DServer.shape_set_data(self.physics_shape, PoolVector2Array(self.generate_corners()))
    return self.physics_shape

func destroy_physics_shape():
    if self.physics_shape.get_id() != 0:
        Physics2DServer.free_rid(self.physics_shape)
        self.physics_shape = RID()

func attach_to_physics_space(physics_space_rid: RID):
    assert(physics_space_rid.get_id() != 0)
    self.physics_area = Physics2DServer.area_create()
    Physics2DServer.area_attach_object_instance_id(self.physics_area, get_instance_id())
    Physics2DServer.area_set_monitorable(self.physics_area, false)
    Physics2DServer.area_add_shape(self.physics_area, self.create_physics_shape())
    Physics2DServer.area_set_space(self.physics_area, physics_space_rid)

func detach_from_physics_space():
    if self.physics_area.get_id() != 0:
        Physics2DServer.free_rid(self.physics_area)
        self.physics_area = RID()
    self.destroy_physics_shape()

func generate_corners() -> PoolVector2Array:
    if len(self.corners) == 0:
        var aspect_degrees = rad2deg(atan(self.aspect_ratio))
        self.corners = PoolVector2Array([
            Vector2(
                self.center.x + self.diagonal * sin(deg2rad(+aspect_degrees + self.direction)),
                self.center.y + self.diagonal * cos(deg2rad(+aspect_degrees + self.direction))
                ),
            Vector2(
                self.center.x + self.diagonal * sin(deg2rad(-aspect_degrees + self.direction)),
                self.center.y + self.diagonal * cos(deg2rad(-aspect_degrees + self.direction))
                ),
            Vector2(
                self.center.x + self.diagonal * sin(deg2rad(180 + aspect_degrees + self.direction)),
                self.center.y + self.diagonal * cos(deg2rad(180 + aspect_degrees + self.direction))
                ),
            Vector2(
                self.center.x + self.diagonal * sin(deg2rad(180 - aspect_degrees + self.direction)),
                self.center.y + self.diagonal * cos(deg2rad(180 - aspect_degrees + self.direction))
                )
            ])
    return self.corners
