"""Joint-space acceleration MPC prototype (NOT full nonlinear dynamics MPC)."""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import minimize

@dataclass
class MPCConfig:
    horizon: int = 12
    dt: float = .02
    acceleration_limit: float = 12.
    position_weight: float = 500.
    velocity_weight: float = 8.
    effort_weight: float = .002
    delta_weight: float = .03

class JointMPC:
    """Finite horizon constrained double-integrator MPC.

    Output accelerations must be applied via inverse dynamics and clipped
    to real actuator limits; do not infer FR5 hardware compliance.
    """
    def __init__(self,n=6,config=None):
        self.n=n
        self.cfg=config or MPCConfig()
        self.last=np.zeros((self.cfg.horizon,n))
    def solve(self,q,v,refs,velocity_refs,acceleration_refs=None):
        q=np.asarray(q,float);v=np.asarray(v,float)
        N,n=self.cfg.horizon,self.n
        refs=np.asarray(refs,float);velocity_refs=np.asarray(velocity_refs,float)
        assert q.shape==(n,) and v.shape==(n,)
        assert refs.shape==(N,n) and velocity_refs.shape==(N,n)
        if acceleration_refs is None: acceleration_refs=np.zeros((N,n))
        acceleration_refs=np.asarray(acceleration_refs,float)
        dt=self.cfg.dt
        steps=np.arange(1,N+1)
        mask=(np.arange(N)[None,:]<steps[:,None])
        A=np.maximum(steps[:,None]-np.arange(N)[None,:]-.5,0.)*mask*dt**2
        B=mask*dt
        pbase=q[None,:]+steps[:,None]*dt*v[None,:]
        vbase=np.broadcast_to(v,(N,n))
        c=self.cfg
        def objective(flat):
            u=flat.reshape(N,n)
            ep=pbase+A@u-refs
            ev=vbase+B@u-velocity_refs
            du=np.diff(np.vstack((self.last[0:1],u)),axis=0)
            value=c.position_weight*np.sum(ep**2)+c.velocity_weight*np.sum(ev**2)+c.effort_weight*np.sum((u-acceleration_refs)**2)+c.delta_weight*np.sum(du**2)
            grad=2*(c.position_weight*A.T@ep+c.velocity_weight*B.T@ev+c.effort_weight*(u-acceleration_refs))
            grad+=2*c.delta_weight*np.vstack((du[:-1]-du[1:],du[-1:]))
            return float(value),grad.ravel()
        bounds=[(-c.acceleration_limit,c.acceleration_limit)]*(N*n)
        warm=np.vstack((self.last[1:],self.last[-1:]))
        res=minimize(objective,warm.ravel(),method='L-BFGS-B',jac=True,bounds=bounds,options={'maxiter':75,'ftol':1e-10})
        if not np.isfinite(res.fun):raise RuntimeError('MPC numerical failure')
        self.last=res.x.reshape(N,n)
        return self.last[0].copy(),{'success':bool(res.success),'iterations':int(res.nit),'cost':float(res.fun)}
