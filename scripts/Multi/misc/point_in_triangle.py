import numpy as np

a = np.array([0,0])
b = np.array([0,10])
c = np.array([10,0])
p =''
while p != 'y':
    p = input('enter p:')
    print(p)
    v0 = c - a
    v1 = b - a
    v2 = p - a
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)
    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    u = (dot11 * dot02 - dot01 * dot12) * invDenom
    print 'u', u
    v = (dot00 * dot12 - dot01 * dot02) * invDenom
    print 'v', v
    if ((u >= 0) and (v >=0) and (u+v<1)):
        print 'inside'
    else:
        print 'outside'


