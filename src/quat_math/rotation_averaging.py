# -*- coding: utf-8 -*-
"""
@author: bokorn
"""

import numpy as np 
import scipy.interpolate as interpolate
from . import transformations as tf_trans

def projectedAverageQuaternion(quats, weights=None):
    q_avg = np.average(quats, axis=0, weights=weights)
    return q_avg / np.linalg.norm(q_avg)



