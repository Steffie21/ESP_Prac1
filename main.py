from scipy.fft import fft, fftfreq
import numpy as np
from matplotlib import pyplot as plt

# # Number of sample points
# N = 128
# # sample spacing
# T = 1.0 / 80.0

fs = 1000 # Sampling Frequency
t = np.arange(0, 1, 1/fs)
f = 10  # Signal frequency
x = np.sin(2*np.pi*f*t)  # Signal

# Compute the FFT using numpy
X = np.fft.fft(x)

# Compute the frequency vector
freqs = np.fft.fftfreq(len(x), 1/fs)

# Plot the magnitude spectrum of the FFT
plt.figure()
plt.plot(freqs[:len(freqs)//2], np.abs(X[:len(X)//2]))
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.show()

# t = np.linspace(0,N*T, N, endpoint=False)
# x = np.sin(20 *2*np.pi*t)+0.5*np.sin(10 *2*np.pi*t)
# X = fft(x)

# xfreq = fftfreq(N, T)[:N//2]
# plt.plot(xfreq, 2.0/N * np.abs(X[0:N//2]))
# plt.show()