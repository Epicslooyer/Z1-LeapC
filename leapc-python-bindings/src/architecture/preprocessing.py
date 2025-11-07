import numpy as np

def vec_to_np(vec):
    return np.array([vec.x, vec.y, vec.z], dtype=float)

def extract_features(hand, prev_data = None, prev_data_2 = None):

    def bone_direction(bone):
        diff = vec_to_np(bone.next_joint) - vec_to_np(bone.prev_joint)
        norm = np.linalg.norm(diff)
        return diff / norm if norm > 1e-8 else np.zeros(3)

    palm = vec_to_np(hand.palm.position)
    normal = vec_to_np(hand.palm.normal)
    direction = vec_to_np(hand.palm.direction)
    epsilon = 1e-8

    #Local frame
    right = np.cross(direction, normal)
    right_norm = np.linalg.norm(right)
    right = right / (right_norm + epsilon)
    R = np.column_stack([right, normal, direction])

    #Index finger tip
    index_finger = hand.digits[1]
    distal_bone = index_finger.bones[3]
    index_tip = vec_to_np(distal_bone.next_joint)
    tip_local = R.T @ (index_tip - palm)

    #Index finger direction
    index_direction = bone_direction(distal_bone)

    #Direction
    dist = np.linalg.norm(tip_local)
    tilt = np.dot(direction, np.array([0,1,0]))
    angle = np.dot(index_direction, direction)

    #Dynamic features, based on velocity (first order derivative) and acceleration (second order derivative)
    palm_vel, palm_accel, tip_vel, tip_accel, direction_change = [np.zeros(3) for i in range(5)]


    if prev_data is not None:
        prev_distal_bone = prev_data.digits[1].bones[3]
        prev_tip = vec_to_np(prev_distal_bone.next_joint)
        prev_palm = vec_to_np(prev_data.palm.position)
        palm_vel = (palm - prev_palm)
        tip_vel = (index_tip - prev_tip)

        prev_index_direction = bone_direction(prev_distal_bone)
        direction_change = index_direction - prev_index_direction

    if prev_data_2 is not None and prev_data is not None:
        prev_palm_2 = vec_to_np(prev_data_2.palm.position)
        palm_accel = palm - 2*prev_palm + prev_palm_2

        prev_tip_2 = vec_to_np(prev_data_2.digits[1].bones[3].next_joint)
        tip_accel = index_tip - 2*prev_tip + prev_tip_2
    
    # Scalar invariants
    stable = 1.0
    speed = np.linalg.norm(tip_vel)
    palm_speed = np.linalg.norm(palm_vel)
    angle_stability = stable - np.linalg.norm(direction_change)
    
    feature = np.concatenate([
        tip_local,
        direction, normal, right,
        palm_vel, palm_accel, tip_vel, tip_accel,
        [dist, tilt, angle, speed, palm_speed, angle_stability]
    ])
    
    return feature

def twohand_extract_feature(hand1, hand2):
    
    palm1 = vec_to_np(hand1.palm.position)
    palm2 = vec_to_np(hand2.palm.position)
    palm1vel = vec_to_np(hand1.palm.velocity)
    palm2vel = vec_to_np(hand2.palm.velocity)
    palmdist = np.linalg.norm(palm1 - palm2) / 100.0
    relvel = (palm1vel - palm2vel) / 100.0

    feature = np.concatenate([[palmdist], relvel])
    return feature
