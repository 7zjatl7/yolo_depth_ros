import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation

def extrinsics_from_global_pose(position: Tuple[float, float, float], euler_angles: Tuple[float, float, float], euler_order: str = 'xyz') -> Tuple[np.ndarray, np.ndarray]:
    
    t = np.array(position)
    r = Rotation.from_euler(euler_order, euler_angles, degrees=True)
    R_matrix = r.as_matrix()
    return R_matrix, t

def transform_from_tf(translation, rotation):
    T = np.eye(4)
    t = np.array([translation[0], translation[1], translation[2]])
    T[0:3, 3] = t
    # SciPy를 사용하여 quaternion을 회전 행렬로 변환 (quaternion 형식은 [x, y, z, w])
    q = [rotation[3], rotation[0], rotation[1], rotation[2]]
    R = Rotation.from_quat(q).as_matrix()
    T[0:3, 0:3] = R
    return T

def inverse_transform(T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R.T
    T_inv[0:3, 3] = -R.T @ t
    return T_inv

def transform_point(T, point):
    p_hom = np.append(point, 1)
    p_trans = T @ p_hom
    return p_trans[0:3]

def world2cam(T_world_to_center, point_world):
    return transform_point(T_world_to_center, point_world)

def cam2world(T_world_to_center, point_center):
    T_center_to_world = inverse_transform(T_world_to_center)
    return transform_point(T_center_to_world, point_center)