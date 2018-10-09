# -*- coding: utf-8 -*-
"""
@author: bokorn
"""

import numpy as np 
import scipy.interpolate as interpolate
from . import transformations as tf_trans

# inverse transform sampling proportional to sin2(theta)
_x = np.linspace(0, np.pi, 256)
_cdf_norm = np.pi/2.
_cdf_sin2 = 1./2. * (_x - np.sin(_x))
_cdf_sin2 /= _cdf_norm
_inv_cdf = interpolate.interp1d(_cdf_sin2, _x)

def randomQuatNear(init_quat, max_orientation_offset):
    offset_axis = np.random.randn(3)
    offset_axis /= np.linalg.norm(offset_axis)
    norm_const = 1/2 * (max_orientation_offset - np.cos(max_orientation_offset)*np.sin(max_orientation_offset))/_cdf_norm
    offset_angle = _inv_cdf(norm_const * np.random.rand())
    offset_quat = tf_trans.quaternion_about_axis(offset_angle, offset_axis)
    near_quat = tf_trans.quaternion_multiply(init_quat, offset_quat)
    return near_quat, offset_quat
  
def quatDiff(q1, q2):
    q_diff = tf_trans.quaternion_multiply(q1, tf_trans.quaternion_inverse(q2))
    if(q_diff[-1] < 0):
        q_diff *= -1.0
    return q_diff         
    
def quatAngularDiff(q1, q2):
    q_diff = quatDiff(q1, q2)
    return quat2AxisAngle(q_diff)[1]

def quat2AxisAngle(q):
    if(q[-1] < 0):
        q *= -1 
    if(abs(q[3])>1):
        q[3] = np.sign(q[3])
    angle = 2*np.arccos(q[3])
    if(np.sin(angle/2.0) != 0):
        axis = q[:3]/np.sin(angle/2.0)
    else:
        axis = np.array([0,0,1])
    return axis, angle

def angularPDF(theta):
    return 2/np.pi * np.sin(theta/2.0)**2

def anglurCDF(theta):
    return 1/np.pi * (theta - np.sin(theta))

def invAngularPDF(theta, thresh, eps=1e-6):
    return thresh / np.maximum(angularPDF(theta), eps)

