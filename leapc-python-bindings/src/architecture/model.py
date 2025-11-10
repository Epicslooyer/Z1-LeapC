import numpy as np
import os 
import joblib
from sklearn.preprocessing import StandardScaler
from hmmlearn.hmm import GMMHMM

#Use in conda environment with hmmlearn==0.28.0

GESTURE_SET = [
    "point up", "point down", "point forward", "point back",
    "point left", "point right",
    "move hand up", "move hand down",
    "pause", "clap",
    "swipe towards", "swipe back", "pull back", "clap"
]

os.makedirs("models", exist_ok=True)

def data_loader(labels):
    X_label = {}
    labelarr = []

    for label in labels:
        gesturename = label.replace(" ", "_")
        gesturedir = os.path.join("data", gesturename)

        samples = []
        for file in sorted(os.listdir(gesturedir)):
            if file.endswith(".npy"):
                path = os.path.join(gesturedir, file)
                sample = np.load(path)

                if sample.ndim == 2 and sample.shape[0] > 0 and sample.shape[1] > 0:
                    samples.append(sample)
                else:
                    print(f"Invalid sample found in {path}")
        
        if samples:
            X_label[gesturename] = samples
            labelarr.append(gesturename)
        
    return X_label, labelarr

def train_model():
    X_label, labelarr = data_loader(GESTURE_SET)

    if not labelarr:
        print("No data to train model")
        return
    
    groups = {}
    for label, samples in X_label.items():
        nfeatures = samples[0].shape[1]
        groups.setdefault(nfeatures, []).extend(samples)
    
    for label, samples in X_label.items():
        print(f"{label:20s} -> {samples[0].shape}")

    scalers = {}
    for nfeatures, samples in groups.items():
        flatten = np.vstack(samples)
        scaler = StandardScaler()
        scaler.fit(flatten)
        scalers[nfeatures] = scaler
        joblib.dump(scaler, os.path.join("models", f"scaler_{nfeatures}f.pkl"))

    #Scale data for training
    X_scaled = {}
    for label, samples in X_label.items():
        nfeatures = samples[0].shape[1]
        if nfeatures not in scalers:
            print(f"No scaler found for {label} with {nfeatures} features")
            continue
        scaler = scalers[nfeatures]
        X_scaled[label] = [scaler.transform(sample) for sample in samples]
        
    #Train HMM per gesture
    #Architecture: 3 hidden states, 3 guassian emissions per state, diagonal covariance matrix, 100 iterations
    models = {}
    loglikelihoods = {}
    N_STATES = 6
    N_MIXTURES = 1
    RANDOM_STATE = 42
    N_ITER = 100

    print("Labels in X_scaled:", list(X_scaled.keys()))

    for label, samples in X_scaled.items():
        nfeatures = samples[0].shape[1]
        X_concat = np.vstack(samples)
        lengths = [len(sample) for sample in samples]


        if np.any(np.isnan(X_concat)):
            print(f"NaN values found in {label} data")
        
        var = np.var(X_concat, axis=0)
        if len(np.where(var < 1e-10)[0]) > 0:
            print(f"Feature with zero variance found in {label} data")

        if label == "clap": #Ergodic model
            model = GMMHMM(n_components = N_STATES, n_mix = N_MIXTURES, covariance_type = "diag", n_iter = N_ITER, random_state = RANDOM_STATE, verbose = False, min_covar=0.1)
        else:
            #Top right topology
            startprob = np.zeros(N_STATES)
            startprob[0] = 1.0
            transmat = np.zeros((N_STATES, N_STATES))
            for i in range(N_STATES - 1):
                transmat[i, i] = 0.5
                transmat[i, i+1] = 0.5
            transmat[N_STATES - 1, N_STATES - 1] = 1.0

            model = GMMHMM(n_components = N_STATES, n_mix = N_MIXTURES, covariance_type = "diag", n_iter = N_ITER, random_state = RANDOM_STATE, verbose = False, init_params="mcw", min_covar=0.1)
            model.startprob_ = startprob
            model.transmat_ = transmat

        try:
            model.fit(X_concat, lengths)
            avgloglikelihood = np.mean([model.score(sample) for sample in samples])
            models[label] = model
            loglikelihoods[label] = avgloglikelihood
            print(f"Trained {label} model with loglikelihood: {avgloglikelihood:.4f}")
        except Exception as e:
            print(f"Error training {label} model: {e}")

    
    if models:
        modelpath = os.path.join("models", "gesture_models.pkl")
        joblib.dump(models, modelpath)
        print(f"Trained {len(models)} gesture models and saved to {modelpath}")
    else:
        print("Failed to train any gesture models")

if __name__ == "__main__":
    train_model()
