"""Planning-scene diff builders for explicit grasp attach/detach transitions."""


def attach_collision_object(collision_object, *, link_name, touch_links):
    """Build, but do not apply, an attached-object scene diff.

    Touch links are mandatory so callers must name the validated gripper contact
    geometry instead of silently allowing collisions with the whole robot.
    """
    from moveit_msgs.msg import AttachedCollisionObject, PlanningScene

    links = tuple(dict.fromkeys(str(link) for link in touch_links if str(link)))
    if not link_name or not links:
        raise ValueError("link_name and at least one explicit touch link are required")
    attached = AttachedCollisionObject()
    attached.link_name = str(link_name)
    attached.touch_links = list(links)
    attached.object = collision_object
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    return scene


def detach_collision_object(object_id):
    """Build, but do not apply, a detach diff for a named attached object."""
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene

    if not object_id:
        raise ValueError("object_id is required")
    attached = AttachedCollisionObject()
    attached.object.id = str(object_id)
    attached.object.operation = CollisionObject.REMOVE
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [attached]
    return scene
