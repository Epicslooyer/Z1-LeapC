import leap
import os
import cv2
import time
import numpy as np
from architecture.preprocessing import extract_features
from architecture.preprocessing import twohand_extract_feature
from sklearn.preprocessing import StandardScaler

GESTURE_SET = [
    "point up", "point down", "point forward", "point back",
    "point left", "point right",
    "move hand up", "move hand down",
    "pause", "clap",
    "swipe towards", "swipe back", "pull back"
]

twohand_gestures = ["clap"]

class addlistener(leap.Listener):
    def __init__(self, gesture_dir, label, sample_num):
        super().__init__()
        self.label = label
        self.twohand = label in twohand_gestures
        self.buffer = []
        self.isrecording = False
        self.frames = 0
        self.gesture_dir = gesture_dir
        self.sample_num = sample_num
    
    def startrecording(self):
        self.buffer = []
        self.frames = 0
        self.isrecording = True
        print(f"Starting recording for '{self.label}'")

    def on_tracking_event(self, event):
        if not self.isrecording:
            return
        
        if self.frames >= 90:
            self.stoprecording()
            return
        
        if self.twohand:
            if len(event.hands) != 2:
                print("Two hand gesture requires two hands")
                return
            
            hand1, hand2 = event.hands[0], event.hands[1]
            feature = twohand_extract_feature(hand1, hand2)
        
        else:
            if len(event.hands) != 1:
                print("One hand gesture requires one hand")
                return
            
            hand = event.hands[0]
            feature = extract_features(hand)
        
        self.buffer.append(feature)
        self.frames += 1
    
    def stoprecording(self):
        self.isrecording = False
        
        if len(self.buffer) < 90:
            print(f"Buffer too small to save, <90 frames")
            return False
        
        filename = f"{self.label.replace(' ', '_')}_{self.sample_num:03d}.npy"
        np.save(os.path.join(self.gesture_dir, filename), np.array(self.buffer))
        print(f"Saved gesture '{self.label}' ({len(self.buffer)} frames => {filename})")
        return True

def collect_samples(data_dir = "data"):
    os.makedirs(data_dir, exist_ok=True)

    cv2.namedWindow("Preprocessing")
    connection = leap.Connection()

    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)

        for gesture in GESTURE_SET:
            os.makedirs(os.path.join(data_dir, gesture.replace(" ", "_")), exist_ok=True)
            print(f"Preparing to record gesture: '{gesture}'")
            sample = 0

            while sample < 10:
                # self, gesture_dir, label, sample_num
                listener = addlistener(os.path.join(data_dir, gesture.replace(" ", "_")), gesture, sample)
                connection.add_listener(listener)

                img = np.zeros((100, 400), dtype = np.uint8)
                cv2.putText(img, f"Gesture: {gesture}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                cv2.putText(img, f"Sample: {sample+1}/10", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                cv2.imshow("Preprocessing", img)

                key = cv2.waitKey(0)

                if key == ord('s'):
                    listener.startrecording()

                    while listener.isrecording:
                        img = np.zeros((200, 500, 3), dtype = np.uint8)
                        cv2.putText(img, f"Recording gesture: {gesture}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255))
                        progress = listener.frames / 90.0
                        bar_width = int(480 * progress)
                        cv2.rectangle(img, (10, 80), (10 + bar_width, 120), (0, 255, 0), -1)
                        cv2.rectangle(img, (10, 80), (490, 120), (255, 255, 255), 2)
                        cv2.putText(img, f"{listener.frames}/90 frames", (180, 105),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.imshow("Preprocessing", img)
                        cv2.waitKey(50)

                    
                    if len(os.listdir(os.path.join(data_dir, gesture.replace(" ", "_")))) > sample:
                        sample += 1
                    
                
                connection.remove_listener(listener)
   
    cv2.destroyAllWindows()
    return True


if __name__ == "__main__":
    collect_samples()
