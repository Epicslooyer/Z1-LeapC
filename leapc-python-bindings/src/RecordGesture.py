import numpy as np
import time
import os
from architecture.preprocessing import extract_features

class RecordGesture:
    def __init__(self, data_dir = "data"):
        self.buffer = []
        self.prev_hand = None
        self.prev_hand_2 = None
        self.data_dir = data_dir
        os.makedirs(data_dir, exist_ok=True)
        self.start = None

    def addframe(self, hand):
        feature = extract_features(hand, self.prev_hand, self.prev_hand_2)
        self.buffer.append(feature)
        self.prev_hand_2 = self.prev_hand
        self.prev_hand = hand
    
    def clearbuffer(self):
        self.buffer.clear()
        self.prev_hand = None
        self.prev_hand_2 = None
    
    def savebuffer(self, label):

        if len(self.buffer) < 5:
            print("Buffer too small to save, <5 frames")
            return
        
        framename = f"{label}_{int(time.time())}.npy"
        np.save(os.path.join(self.data_dir, framename), np.array(self.buffer))
        print(f"Saved gesture '{label}' ({len(self.buffer)} frames => {framename}) ")
        self.clearbuffer()