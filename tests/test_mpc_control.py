import unittest
import numpy as np
from robobarista.mpc_control import JointMPC,MPCConfig

class MPCTests(unittest.TestCase):
    def test_hold(self):
        m=JointMPC(2,MPCConfig(horizon=8))
        a,d=m.solve(np.zeros(2),np.zeros(2),np.zeros((8,2)),np.zeros((8,2)))
        self.assertTrue(np.allclose(a,0,atol=1e-6))
        self.assertTrue(np.isfinite(d['cost']))
    def test_move(self):
        m=JointMPC(1)
        p=np.full((12,1),.004)
        a,d=m.solve(np.zeros(1),np.zeros(1),p,np.zeros_like(p))
        self.assertGreater(a[0],0)
        self.assertLessEqual(abs(a[0]),12)
    def test_axes(self):
        m=JointMPC(2)
        p=np.tile([.004,-.004],(12,1))
        a,d=m.solve(np.zeros(2),np.zeros(2),p,np.zeros_like(p))
        self.assertGreater(a[0],0)
        self.assertLess(a[1],0)

if __name__=='__main__':unittest.main()
