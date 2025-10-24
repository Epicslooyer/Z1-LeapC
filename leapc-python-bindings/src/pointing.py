import leap
import time
import math
import numpy as np
import cv2

def normalize(vector):
    norm = np.linalg.norm(vector)
    return vector / norm if norm > 0 else vector

def pointing(pos, dir):
    p = np.array(pos)
    d = normalize(np.array(dir))
    return lambda t: p + t * d

def intersection(p, d, point, normal):
    p = np.array(p)
    d = normalize(np.array(d))
    n = np.array(normal)
    d_dot_n = np.dot(d, n)
    if d_dot_n == 0:
        return None
    t = np.dot(n, point - p) / d_dot_n
    return p + t * d


try:
    TrackingMode = leap.TrackingMode
except AttributeError:
    try:
        from leap.enums import TrackingMode as _TrackingMode
    except (ImportError, AttributeError) as exc:  
        raise ImportError(
            "TrackingMode enum is unavailable from the loaded leap module. "
            "Install the repo's leapc-python-api package (pip install -e leapc-python-api) "
            "and ensure the correct LEAPSDK path is configured."
        ) from exc
    else:
        TrackingMode = _TrackingMode
        setattr(leap, "TrackingMode", TrackingMode)

_TRACKING_MODES = {
    TrackingMode.Desktop: "Desktop",
    TrackingMode.HMD: "HMD",
    TrackingMode.ScreenTop: "ScreenTop",
}

class DirectionVector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Canvas:
    def __init__(self):
        self.name = "Leap Motion Direction Detector"
        self.screen_size = [500, 700]
        self.hands_colour = (255, 255, 255)
        self.font_colour = (0, 255, 44)
        self.hands_format = "Skeleton"
        self.output_image = np.zeros((self.screen_size[0], self.screen_size[1], 3), np.uint8)
        self.tracking_mode = None
        self.current_direction = "No hands detected"
        self.palm_direction_text = ""

    def set_tracking_mode(self, tracking_mode):
        self.tracking_mode = tracking_mode

    def get_joint_position(self, bone):
        if bone:
            SCALE = 1.0 
            return int(bone.x * SCALE + (self.screen_size[1] / 2)), int((self.screen_size[0] / 2) - (bone.z * SCALE))
        else:
            return None

    def render_hands(self, event):
        self.output_image[:, :] = 0

        cv2.putText(
            self.output_image,
            "Leap Motion Direction Detector",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            self.font_colour,
            2,
        )

        cv2.putText(
            self.output_image,
            f"Direction: {self.current_direction}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.font_colour,
            2,
        )

        cv2.putText(
            self.output_image,
            self.palm_direction_text,
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            self.font_colour,
            1,
        )

        cv2.putText(
            self.output_image,
            f"Tracking Mode: {_TRACKING_MODES.get(self.tracking_mode, 'Unknown')}",
            (10, self.screen_size[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            self.font_colour,
            1,
        )

        if len(event.hands) == 0:
            return

        for i in range(0, len(event.hands)):
            hand = event.hands[i]
            
            palm_pos = self.get_joint_position(hand.palm.position)
            if palm_pos:
                cv2.circle(self.output_image, palm_pos, 8, self.hands_colour, -1)
                cv2.putText(self.output_image, "PALM", 
                           (palm_pos[0] + 10, palm_pos[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.hands_colour, 1)
                
                palm_dir = hand.palm.direction
                arrow_end_x = int(palm_pos[0] + palm_dir.x * 50)
                arrow_end_y = int(palm_pos[1] - palm_dir.z * 50) 
                cv2.arrowedLine(self.output_image, palm_pos, (arrow_end_x, arrow_end_y), (0, 255, 0), 3)
            
            wrist = self.get_joint_position(hand.arm.next_joint)
            elbow = self.get_joint_position(hand.arm.prev_joint)
            if wrist and elbow:
                cv2.circle(self.output_image, wrist, 3, self.hands_colour, -1)
                cv2.circle(self.output_image, elbow, 3, self.hands_colour, -1)
                if wrist and elbow:
                    cv2.line(self.output_image, wrist, elbow, self.hands_colour, 2)

            for index_digit in range(0, 5):
                digit = hand.digits[index_digit]
                for index_bone in range(0, 4):
                    bone = digit.bones[index_bone]
                    
                    bone_start = self.get_joint_position(bone.prev_joint)
                    bone_end = self.get_joint_position(bone.next_joint)

                    if bone_start:
                        cv2.circle(self.output_image, bone_start, 3, self.hands_colour, -1)

                    if bone_end:
                        cv2.circle(self.output_image, bone_end, 3, self.hands_colour, -1)

                    if bone_start and bone_end:
                        cv2.line(self.output_image, bone_start, bone_end, self.hands_colour, 2)

                    if index_digit == 1 and index_bone == 3:  
                        if bone_end:
                            cv2.circle(self.output_image, bone_end, 6, (0, 255, 0), -1)
                            cv2.putText(self.output_image, "INDEX", 
                                       (bone_end[0] + 10, bone_end[1]), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                    if index_bone == 0 and bone_start and wrist:
                        cv2.line(self.output_image, bone_start, wrist, self.hands_colour, 2)

class DirectionDetector(leap.Listener):
    def __init__(self, canvas, min=50, max=2000):
        self.canvas = canvas
        self.prev_dir = None
        self.dt = None
        self.min = min
        self.max = max
        self.alpha = 0.5
    
    def vector_length(self, v):
        v = np.array(v, dtype=float)
        n = np.linalg.norm(v)
        return (v/n) if n > 1e-8 else None
    
    def adapt_alpha(self, angle):
        a_upper, a_lower = 0.9, 0.1
        roundoff = 30
        t = min(angle / roundoff, 1)
        return a_upper * (1-t) + a_lower * t
        
    def calc_angle(self, x, y):
        dot = np.clip(np.dot(x, y), -1, 1)
        return math.degrees(math.acos(dot))
    
    def on_tracking_event(self, event):

        self.canvas.render_hands(event) 

        if len(event.hands) == 0:
            self.canvas.current_direction = "No hands detected"
            return
        
        hand = event.hands[0] 
        index = hand.digits[1]

        
        palm_pos = np.array([
            hand.palm.position.x,
            hand.palm.position.y,
            hand.palm.position.z
        ])

        tip = np.array([
            index.bones[3].next_joint.x,
            index.bones[3].next_joint.y,
            index.bones[3].next_joint.z
        ])
        base = np.array([
            index.bones[0].prev_joint.x,
            index.bones[0].prev_joint.y,
            index.bones[0].prev_joint.z
        ])

        virtual_plane = -100.0
        point_dir = normalize(tip - base)

        if self.prev_dir is not None:
            smooth_dir = normalize(self.alpha * self.prev_dir + (1 - self.alpha) * point_dir)
        else:
            smooth_dir = point_dir
        self.prev_dir = smooth_dir
        plane_point = np.array([0.0, 0.0, virtual_plane])
        plane_normal = np.array([0.0, 0.0, 1.0])
        
        target_intersection = intersection(palm_pos, point_dir, plane_point, plane_normal)

        SCALE = 1.0 
        screen_w = self.canvas.screen_size[1]
        screen_h = self.canvas.screen_size[0]
        
        palm2d = (int(palm_pos[0] * SCALE + screen_w / 2), 
                  int(screen_h / 2 - palm_pos[2] * SCALE))
        
        arrow_length = 150 
        endpointer = (
            int(palm2d[0] + point_dir[0] * arrow_length),
            int(palm2d[1] - point_dir[2] * arrow_length)
        )
        
        cv2.arrowedLine(self.canvas.output_image, palm2d, endpointer, (255, 0, 255), 3)

        if target_intersection is not None:
            inter2d = (int(target_intersection[0] * SCALE + screen_w / 2),
                       int(screen_h / 2 - target_intersection[2] * SCALE))

            cv2.circle(self.canvas.output_image, inter2d, 8, (0,0,255), -1)
            cv2.putText(self.canvas.output_image, "Target", (inter2d[0] + 5, inter2d[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        self.canvas.current_direction = self.calculate_direction(DirectionVector(point_dir[0], point_dir[1], point_dir[2]))
        self.canvas.palm_direction_text = f"Index Direction (Norm): ({point_dir[0]:.2f}, {point_dir[1]:.2f}, {point_dir[2]:.2f})"

    
    def calculate_direction(self, palm_direction):

        x = palm_direction.x
        y = palm_direction.y
        z = palm_direction.z
        
        threshold = 0.3
        
        abs_x = abs(x)
        abs_y = abs(y)
        abs_z = abs(z)
        
        if abs_x < threshold and abs_y < threshold and abs_z < threshold:
            return "Center"
        
        max_component = max(abs_x, abs_y, abs_z)
        
        if max_component == abs_x:
            return "Right" if x > 0 else "Left"
        elif max_component == abs_y:
            return "Up" if y > 0 else "Down"
        else: 
            return "Forward" if z < 0 else "Backward"

def main():
    canvas = Canvas()
    detector = DirectionDetector(canvas)
    connection = leap.Connection()
    connection.add_listener(detector)

    running = True

    with connection.open():
        connection.set_tracking_mode(TrackingMode.Desktop)

        while running:
            cv2.imshow(canvas.name, canvas.output_image)

            if cv2.waitKey(1) == ord('x'):
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()