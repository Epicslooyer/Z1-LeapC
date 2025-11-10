import numpy as np

def vec_to_np(vec):
    return np.array([vec.x, vec.y, vec.z], dtype=float)

def extract_features(hand, prev_data, prev_data_2):
    try:
        epsilon = 1e-8

        def bone_direction(bone):
            diff = vec_to_np(bone.next_joint) - vec_to_np(bone.prev_joint)
            norm = np.linalg.norm(diff)
            if norm < epsilon:
                return np.zeros(3)
            return diff / norm

        palm = vec_to_np(hand.palm.position)
        normal = vec_to_np(hand.palm.normal)
        direction = vec_to_np(hand.palm.direction)

        if np.linalg.norm(normal) < epsilon or np.linalg.norm(normal) < epsilon:
            print("Normal or direction is too small")
            return np.zeros(88) # (5 fingers * 3 tips) + (5*4*3 bone directions) + 2(3+3) +1

        #Local frame
        right = np.cross(direction, normal)
        right_norm = np.linalg.norm(right)
        right = right / (right_norm + epsilon)
        R = np.column_stack([right, normal, direction])

        fingerfeatures = []
        for digit in hand.digits:
            globaltip = vec_to_np(digit.bones[3].next_joint)
            reltip = globaltip - palm
            localtip = R.T @ reltip
            fingerfeatures.extend(localtip)

            for bone in range(4):
                globalbone = bone_direction(digit.bones[bone])
                relbone = R.T @ globalbone
                fingerfeatures.extend(relbone)
        
        palmvellocal, palmaccellocal = [np.zeros(3) for i in range(2)]

        if prev_data is not None:
            prev_palm = vec_to_np(prev_data.palm.position)
            palm_vel = (palm - prev_palm)
            palmvellocal = R.T @ palm_vel


        if prev_data_2 is not None and prev_data is not None:
            prev_palm_2 = vec_to_np(prev_data_2.palm.position)
            palm_accel = palm - 2*prev_palm + prev_palm_2
            palmaccellocal = R.T @ palm_accel
        
        # Scalar invariants
        speed = np.linalg.norm(palmvellocal)
        
        feature = np.concatenate([
            np.array(fingerfeatures),
            direction, normal, palmvellocal, palmaccellocal, [speed]
        ])
        
        return feature
    
    except Exception as e:
        print(f"Error extracting features: {e}")
        return np.zeros(88)

def twohand_extract_feature(hand1, hand2):
    
    palm1 = vec_to_np(hand1.palm.position)
    palm2 = vec_to_np(hand2.palm.position)
    palm1vel = vec_to_np(hand1.palm.velocity)
    palm2vel = vec_to_np(hand2.palm.velocity)
    palmdist = np.linalg.norm(palm1 - palm2) / 100.0
    relvel = (palm1vel - palm2vel) / 100.0

    feature = np.concatenate([[palmdist], relvel])
    return feature
