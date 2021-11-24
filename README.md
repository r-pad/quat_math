# quat_math
Math utilites for quaternions. A wrapper to Christoph Gohlke's quaternion library (tf.transformations.py in ros) as well as some quaternion distribution and difference functions.

Difference Between Two Quaternions
-----
Quaternion difference between two quaternions
```python
q_diff = quatDiff(q1, q2)
```
Angular difference between two orientation quaternions
```python
theta = quatAngularDiff(q1, q2)
```
Quaternion to Axis Angle
-----
Converts orientation quaternion to axis angle form
```python
axis, angle = quat2AxisAngle(q)
```
Quaternion Difference Distribution
-----
The pdf of two uniformly sampled orientation quaternions having an angular difference of theta
```python
pdf = angularPDF(theta)
```
Quaternion Difference Rejection Sampling
-----
To use rejection sampling to resample two uniformly sampled orientation quaternions so the difference between them is uniformly distributed, up theta_ref. Angle differences below theta_ref are not uniformly sampled but are boosted.
```python
ref_pdf = angularPDF(theta_ref)
reject = invAngularPDF(theta, ref_pdf) > np.random.rand())
```
Random Quaternion's Near a Quaternion
-----
Samples a orientation quaternion from a uniform random distribution within theta of the q
```python
q_near, q_delta = randomQuatNear(q, theta)
```
