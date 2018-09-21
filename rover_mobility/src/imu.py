#double_intergral
import matplotlib.pyplot as plt
import numpy as np
s = 0
v = 0
i = 0
a = 0
ss = []
vv= []
aa = []
while(i<2*np.pi):
	a = np.sin(i)
	v = v+a
	s = s+v
	i = i+np.pi/1000
	vv.append(v)
	ss.append(s)
	aa.append(a)
plt.figure()	
plt.plot(ss)
plt.figure()
plt.plot(vv)
plt.figure()
plt.plot(aa)
plt.show()	