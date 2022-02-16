
from math import atan2, cos, fabs, log, pi, sin, sqrt
from typing import Any, Optional, Sequence, Tuple, Union
from mathutils import Euler, Quaternion, Vector

def as_euler(value: Sequence[float]) -> Euler:
    return value if isinstance(value, Euler) else Euler(value)

def as_quaternion(value: Sequence[float]) -> Quaternion:
    return value if isinstance(value, Quaternion) else Quaternion(value)

def noop(value: Any) -> Any:
    return value

def axis_angle_to_euler(value: Union[Sequence[float], Tuple[Sequence[float], float]]) -> Euler:
    return quaternion_to_euler(axis_angle_to_quaternion(value))

def axis_angle_to_quaternion(value: Union[Sequence[float], Tuple[Sequence[float], float]]) -> Quaternion:
    return Quaternion(*value) if len(value) == 2 else Quaternion(value[1:], value[0])

def axis_angle_to_swing_twist(value: Union[Sequence[float], Tuple[Sequence[float], float]], axis: str, quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist(axis_angle_to_quaternion(value), axis, quaternion)

def axis_angle_to_swing_twist_x(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return axis_angle_to_swing_twist(value, 'X', quaternion=quaternion)

def axis_angle_to_swing_twist_y(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return axis_angle_to_swing_twist(value, 'Y', quaternion=quaternion)

def axis_angle_to_swing_twist_z(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return axis_angle_to_swing_twist(value, 'Z', quaternion=quaternion)

def axis_angle_to_logarithmic_map(value: Union[Sequence[float], Tuple[Sequence[float], float]]) -> Vector:
    return quaternion_to_logarithmic_map(axis_angle_to_quaternion(value))

def euler_to_axis_angle(value: Sequence[float], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    return quaternion_to_axis_angle(euler_to_quaternion(value), vectorize=vectorize)

def euler_to_quaternion(euler: Sequence[float]) -> Quaternion:
    return as_euler(euler).to_quaternion()

def euler_to_axis_angle(euler: Sequence[float], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    return quaternion_to_axis_angle(euler_to_quaternion(euler), vectorize)

def euler_to_swing_twist(value: Sequence[float], axis: str, quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist(euler_to_quaternion(value), axis, quaternion)

def euler_to_swing_twist_x(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return euler_to_swing_twist(value, 'X', quaternion=quaternion)

def euler_to_swing_twist_y(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return euler_to_swing_twist(value, 'Y', quaternion=quaternion)

def euler_to_swing_twist_z(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return euler_to_swing_twist(value, 'Z', quaternion=quaternion)

def euler_to_aim_vector_x(value: Sequence[float]) -> Vector:
    return quaternion_to_aim_vector_x(euler_to_quaternion(value))

def euler_to_aim_vector_y(value: Sequence[float]) -> Vector:
    return quaternion_to_aim_vector_y(euler_to_quaternion(value))

def euler_to_aim_vector_z(value: Sequence[float]) -> Vector:
    return quaternion_to_aim_vector_z(euler_to_quaternion(value))

def euler_to_aim_vector(value: Sequence[float], axis: str) -> Vector:
    if axis == 'X': return euler_to_aim_vector_x(value)
    if axis == 'Y': return euler_to_aim_vector_y(value)
    if axis == 'Z': return euler_to_aim_vector_z(value)
    raise ValueError()

def euler_to_logarithmic_map(value: Sequence[float]) -> Vector:
    return quaternion_to_logarithmic_map(euler_to_quaternion(value))

def quaternion_to_axis_angle(quaternion: Sequence[float], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    axis, angle = as_quaternion(quaternion).to_axis_angle()
    return Vector((angle, axis[0], axis[1], axis[2])) if vectorize else (axis, angle)

def quaternion_to_euler(quaternion: Sequence[float], order: Optional[str]='XYZ') -> Euler:
    return as_quaternion(quaternion).to_euler(order)

def quaternion_to_swing_twist(value: Sequence[float], axis: str, quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    swing, twist = as_quaternion(value).to_swing_twist(axis)
    if quaternion:
        index = 'WXYZ'.index(axis)
        swing[index] = twist
        return swing
    return swing, twist

def quaternion_to_swing_twist_x(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist(value, 'X', quaternion=quaternion)

def quaternion_to_swing_twist_y(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist(value, 'Y', quaternion=quaternion)

def quaternion_to_swing_twist_z(value: Sequence[float], quaternion: Optional[bool]=False) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist(value, 'Z', quaternion=quaternion)

def quaternion_to_aim_vector_x(value: Sequence[float]) -> Vector:
    w, x, y, z = value
    return Vector((1.0-2.0*(y*y+z*z), 2.0*(x*y+w*z), 2.0*(x*z-w*y)))

def quaternion_to_aim_vector_y(value: Sequence[float]) -> Vector:
    w, x, y, z = value
    return Vector((2.0*(x*y-w*z), 1.0-2.0*(x*x+z*z), 2.0*(y*z+w*x)))

def quaternion_to_aim_vector_z(value: Sequence[float]) -> Vector:
    w, x, y, z = value
    return Vector((2.0*(x*z+w*y), 2.0*(y*z-w*x), 1.0-2.0*(x*x+y*y)))

def quaternion_to_aim_vector(value: Sequence[float], axis: str) -> Vector:
    if axis == 'X': return quaternion_to_aim_vector_x(value)
    if axis == 'Y': return quaternion_to_aim_vector_y(value)
    if axis == 'Z': return quaternion_to_aim_vector_z(value)
    raise ValueError()

def quaternion_to_logarithmic_map(value: Sequence[float]) -> Vector:
    w, x, y, z = value
    q = Vector((w, x, y, z))
    b = sqrt(x**2 + y**2 + z**2)
    if b <= 1.0000000000000002e-14 * abs(w):
        if w < 0.0:
            if fabs(w + 1.0) > 1.0000000000000002e-14:
                q[0] = log(-w)
                q[1] = pi
                q[2] = 0.0
                q[3] = 0.0
            else:
                q[0] = 0.0
                q[1] = pi
                q[2] = 0.0
                q[3] = 0.0
        else:
            q[0] = log(w)
            q[1] = 0.0
            q[2] = 0.0
            q[3] = 0.0
    else:
        v = atan2(b, w)
        f = v / b
        q[0] = log(w * w + b * b) / 2.0
        q[1] = f * x
        q[2] = f * y
        q[3] = f * z

    return q

def swing_twist_x_to_euler(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Euler:
    return quaternion_to_euler(swing_twist_x_to_quaternion(value))

def swing_twist_x_to_axis_angle(value: Union[Tuple[Sequence[float], float], Sequence[float]], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    return quaternion_to_axis_angle(swing_twist_to_quaternion(value, 'X'), vectorize=vectorize)

def swing_twist_y_to_axis_angle(value: Union[Tuple[Sequence[float], float], Sequence[float]], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    return quaternion_to_axis_angle(swing_twist_to_quaternion(value, 'Y'), vectorize=vectorize)

def swing_twist_z_to_axis_angle(value: Union[Tuple[Sequence[float], float], Sequence[float]], vectorize: Optional[bool]=False) -> Union[Tuple[Vector, float], Vector]:
    return quaternion_to_axis_angle(swing_twist_to_quaternion(value, 'Z'), vectorize=vectorize)

def swing_twist_to_aim_vector(value: Union[Tuple[Sequence[float], float], Sequence[float]], twist_axis: str, aim_axis: str) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, twist_axis), aim_axis)

def swing_twist_x_to_aim_vector(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'X'), axis)

def swing_twist_x_to_aim_vector_x(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'X'), 'X')

def swing_twist_x_to_aim_vector_y(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'X'), 'Y')

def swing_twist_x_to_aim_vector_z(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'X'), 'Z')

def swing_twist_y_to_aim_vector(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Y'), axis)

def swing_twist_y_to_aim_vector_x(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Y'), 'X')

def swing_twist_y_to_aim_vector_y(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Y'), 'Y')

def swing_twist_y_to_aim_vector_z(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Y'), 'Z')

def swing_twist_z_to_aim_vector(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Z'), axis)

def swing_twist_z_to_aim_vector_x(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Z'), 'X')

def swing_twist_z_to_aim_vector_y(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Z'), 'Y')

def swing_twist_z_to_aim_vector_z(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_aim_vector(swing_twist_to_quaternion(value, 'Z'), 'Z')

def swing_twist_to_euler(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Quaternion:
    return quaternion_to_euler(swing_twist_to_quaternion(value, axis))

def swing_twist_x_to_euler(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_euler(value, 'X')

def swing_twist_y_to_euler(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_euler(value, 'Y')

def swing_twist_z_to_euler(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_euler(value, 'Z')

def swing_twist_to_quaternion(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Quaternion:
    index = 'WXYZ'.index(axis)
    if len(value) == 2:
        swing = as_quaternion(value[0])
        twist = Quaternion((cos(value[1] * 0.5), 0.0, 0.0, 0.0))
        twist[index] = -sin(value[1] * 0.5)
        return swing @ twist.inverted()
    else:
        swing = as_quaternion(value)
        twist = Quaternion((cos(value[index] * 0.5), 0.0, 0.0, 0.0))
        twist[index] = -sin(value[index] * 0.5)
        swing[index] = 0.0
    return swing @ twist.inverted()

def swing_twist_x_to_quaternion(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_quaternion(value, 'X')

def swing_twist_y_to_quaternion(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_quaternion(value, 'Y')

def swing_twist_z_to_quaternion(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Quaternion:
    return swing_twist_to_quaternion(value, 'Z')

def swing_twist_x_to_swing_twist_y(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_x(swing_twist_x_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_x_to_swing_twist_z(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_z(swing_twist_x_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_y_to_swing_twist_x(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_x(swing_twist_y_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_y_to_swing_twist_z(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_z(swing_twist_y_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_z_to_swing_twist_x(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_x(swing_twist_z_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_z_to_swing_twist_y(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Union[Tuple[Quaternion, float], Quaternion]:
    return quaternion_to_swing_twist_y(swing_twist_z_to_quaternion(value), quaternion=len(value) != 2)

def swing_twist_x_to_logarithmic_map(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_logarithmic_map(swing_twist_to_quaternion(value, 'X'))

def swing_twist_y_to_logarithmic_map(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_logarithmic_map(swing_twist_to_quaternion(value, 'Y'))

def swing_twist_z_to_logarithmic_map(value: Union[Tuple[Sequence[float], float], Sequence[float]]) -> Vector:
    return quaternion_to_logarithmic_map(swing_twist_to_quaternion(value, 'Z'))

def swing_twist_to_logarithmic_map(value: Union[Tuple[Sequence[float], float], Sequence[float]], axis: str) -> Vector:
    return quaternion_to_logarithmic_map(swing_twist_to_quaternion(value, axis))
