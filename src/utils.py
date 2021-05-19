import tf
from math import cos, radians, sin, degrees


def quat_to_euler(orientation):
    """Transform quaternion to euler angles"""
    quat = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(quat)  # roll, pitch, yaw


def polar_to_geom(ang, dist):
    """
    Converts polar coord to geometric coord. It supposes origin as (0, 0)
    :param ang: angle (degrees)
    :param dist: length
    :return: [x, y]
    """
    return [cos(radians(ang)) * dist, sin(radians(ang)) * dist]


def calc_rel_pose(pose, dist):
    """Obtain position (x, y) at a determined dist from pose"""
    _, _, yaw = quat_to_euler(pose.orientation)
    pose_end = polar_to_geom(degrees(yaw), dist)
    return pose.position.x + pose_end[0], pose.position.y + pose_end[1]
