# confidence_model.py — lightweight recurrent confidence pipeline.
# If TensorFlow is installed, a true LSTM can be trained externally. On Pi 3,
# this NumPy recurrent scorer keeps the full offline pipeline runnable.
import os, numpy as np
from config import CONFIDENCE_MODEL_PATH

FEATURES=['error_rate','reaction_time','route_complexity','success','dynamic_obstacles']
class ConfidenceModel:
    def __init__(self,path=CONFIDENCE_MODEL_PATH):
        self.path=path; self.W=np.array([[-1.8,-.18,-.10,.9,-.08]],dtype=float); self.b=.0
        if os.path.exists(path):
            try:
                d=np.load(path); self.W=d['W']; self.b=float(d['b'])
            except Exception: pass
    def predict(self,sequence):
        # Recurrent state: recent sessions are weighted more heavily.
        if not sequence: return .5
        h=.0
        for row in sequence[-10:]:
            x=np.array([[float(row.get(k,0)) for k in FEATURES]])
            h=.65*h + float(np.tanh(x@self.W.T + self.b)[0,0])
        return round(float(1/(1+np.exp(-h))),2)
    def fit_output(self,sequence,targets):
        # Small ridge fit for the output weights over recurrent features.
        if not sequence or len(sequence)!=len(targets): return False
        X=[]
        for seq in sequence:
            h=0.;
            for row in seq[-10:]:
                x=np.array([float(row.get(k,0)) for k in FEATURES]); h=.65*h+float(np.tanh(x@self.W[0]+self.b))
            X.append([h,1.])
        X=np.asarray(X); y=np.asarray(targets); theta=np.linalg.solve(X.T@X+1e-3*np.eye(2),X.T@y)
        self.W[0]=theta[0]*np.array([-1.8,-.18,-.10,.9,-.08]); self.b=theta[1]
        os.makedirs(os.path.dirname(self.path),exist_ok=True); np.savez(self.path,W=self.W,b=self.b); return True
