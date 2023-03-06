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

def bitReversal(n):
    result = []
    for a in range(n):
        # print(bin(a)[2:].zfill(int(np.trunc(np.ceil(np.log2(n)))))[::-1])

        result.append(int(bin(a)[2:].zfill(int(np.trunc(np.ceil(np.log2(n)))))[::-1], 2))
    return result

N = 8#//(2^8)
twiddleFactor = []
for i in range(N):
    twiddleFactor.append(np.exp(-1j*2*np.pi*i/N))

plotTwiddleFactor(twiddleFactor)
print(bitReversal(N))

