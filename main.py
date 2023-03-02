from scipy.fft import fft, fftfreq
import numpy as np
from matplotlib import pyplot as plt

# Number of sample points
N = 128
# sample spacing
T = 1.0 / 80.0

t = np.linspace(0,N*T, N, endpoint=False)
x = np.sin(20 *2*np.pi*t)+0.5*np.sin(10 *2*np.pi*t)
X = fft(x)

xfreq = fftfreq(N, T)[:N//2]
plt.plot(xfreq, 2.0/N * np.abs(X[0:N//2]))
plt.show()