import numpy as np
import matplotlib.pyplot as plt

def plotTwiddleFactor(twiddleFactor):
    twiddleFactor = np.array(twiddleFactor)
    plt.scatter(twiddleFactor.real, twiddleFactor.imag)
    plt.scatter(0, 0, c="k")
    plt.ylabel('Imaginary')
    plt.xlabel('Real')
    plt.grid()
    plt.show()

N = 128#//(2^8)
twiddleFactor = []
for i in range(N):
    twiddleFactor.append(np.exp(-1j*2*np.pi*i/N))

plotTwiddleFactor(twiddleFactor)

