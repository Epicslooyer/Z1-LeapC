import numpy as np
import matplotlib.pyplot as plt
import os
from glob import glob

def visualize_gesture(gesture_path):
    data = np.load(gesture_path, encoding="latin1")
    T, D = data.shape

    tip_local = data[:, 0:3]
    palm_vel = data[:, 12:15]
    tip_vel = data[:, 18:21]
    tilt = data[:, -5]
    tip_speed = data[:, -2]

    fig, axes = plt.subplots(3, 1, figsize=(10, 6))
    axes[0].plot(tip_local)
    axes[0].set_title("Tip local position (x,y,z)")
    axes[0].legend(['x','y','z'])

    axes[1].plot(tip_speed, label="tip speed")
    axes[1].plot(np.linalg.norm(palm_vel, axis=1), label="palm speed")
    axes[1].set_title("Speeds")
    axes[1].legend()

    axes[2].plot(tilt, label="tilt (palm vs up)")
    axes[2].set_title("Tilt angle component")
    axes[2].legend()

    plt.suptitle(f"{os.path.basename(gesture_path)}  (T={T}, D={D})")
    plt.tight_layout()
    plt.show()

def visualize_label(label, data_dir="data"):
    label_dir = os.path.join(data_dir, label)
    paths = sorted(glob(os.path.join(label_dir, f"*.npy")))
    if not paths:
        print("No gestures found for label:", label)
        print(f"Available labels: {os.listdir(data_dir)}")
        return
    plt.figure(figsize=(10, 4))
    for p in paths:
        data = np.load(p)
        tip = data[:, 0:3]
        plt.plot(tip[:,1], label=os.path.basename(p))  
    plt.title(f"Gesture trajectories (label={label})")
    plt.xlabel("Frame index")
    plt.ylabel("Tip y position")
    plt.legend(fontsize=8)
    plt.savefig(f"visualizations/{label}.png")

if __name__ == "__main__":
    os.makedirs("visualizations", exist_ok=True)
    gesture = ['clap', 'move_hand_down', 'move_hand_up', 'pause', 'point_back',
    'point_down', 'point_forward', 'point_left', 'point_right', 'point_up', 
    'pull_back', 'swipe_back', 'swipe_towards']
    gesturetemp = ['point_left', 'point_right', 'point_up', 'point_down', 'point_back', 'point_forward']
    for g in gesturetemp:
        visualize_label(g)