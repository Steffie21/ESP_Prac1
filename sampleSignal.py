import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

T_count = 5

f = 15E3 # Hz
T = 1/f #s
t = np.arange(0, T_count*T, T/100) # s
signal = 1.65*np.sin(2*np.pi*f*t) +1.65

Fs = 80E3 # Hz
Ts = 1/Fs #s
n = np.arange(0, T_count*T, Ts)
xs = 1.65*np.sin(2*np.pi*f*n) +1.65

# print(xs)

plt.plot(t, signal)
plt.stem(n, xs, bottom=1.65)
plt.grid()
plt.show()

ADC_Values = np.round(xs / 3.3 *2**12)
ADC_Values = [int(adc) for adc in ADC_Values]

print(ADC_Values)
