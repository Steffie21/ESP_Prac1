import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

T_count = 5

f = 15E3 # Hz
T = 1/f #s
t = np.arange(0, T_count*T +T/50, T/50) # s
signal = np.sin(2*np.pi*f*t)

Fs = 80E3 # Hz
Ts = 1/Fs #s
n = np.arange(0, T_count*T, Ts)
xs = np.sin(2*np.pi*f*n)

plt.plot(t, signal)
plt.stem(n, xs)
plt.grid()
plt.show()

print(np.arange(0, 2*T, Ts))
