import numpy as np

cx = 2; sx = -2
cy = 3; sy = 3
cz = 4; sz = 4

rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

acc = np.array([[1], [1], [1]])
acc_world = np.matmul(rx,np.matmul(ry,np.matmul(rz, acc)))

print(np.transpose(acc_world))