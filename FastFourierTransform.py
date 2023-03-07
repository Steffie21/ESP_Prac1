import numpy as np
import matplotlib.pyplot as plt

def calculateTwiddleFactor(n):
    twiddleFactor = []
    for i in range(N):
        twiddleFactor.append(np.exp(-1j*2*np.pi*i/N))
    return twiddleFactor

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
x = [0.3535, 0.3535, 0.6464, 1.0607, 0.3535, -1.0607, -1.3535, -0.3535]

twiddleFactor = calculateTwiddleFactor(N)
# plotTwiddleFactor(twiddleFactor)
reversedBits = bitReversal(N)
# print(reversedBits)

# Stage 1: N//2 2-Point DFT 
temp1 = []
for a in range(0, (N+2)//2 +2, 2):
    temp1.append(x[reversedBits[a]] + 1 * x[reversedBits[a +1]])
    temp1.append(x[reversedBits[a]] - 1 * x[reversedBits[a +1]])
# print(temp1)

ans_temp1 = [0.707, 0.0, -0.7071, 1.9998999999999998, -0.7072, 1.4142, 0.7072, 1.4142]
print(temp1 == ans_temp1)

# print(list(range(0, 8, 4)))

# Stage 2: N//4 4-Point DFT 
temp2 = []
for a in range(0, (N+2)//2 +2, 4):
    for b in range(0, 4, 2):
        temp2.append(np.round(temp1[a] + twiddleFactor[b*2] * temp1[a +2], 3))
        temp2.append(np.round(temp1[a +1] + twiddleFactor[(b+1)*2] * temp1[a +3], 3))
# print(temp2)

ans_temp2 = [(-0+0j), -2j, (1.414+0j), (-0+2j), 0j, (1.414-1.414j), (-1.414-0j), (1.414+1.414j)]
print(ans_temp2 == temp2)

# Stage 3: N//8 8-Point DFT 
X = []
for a in range(0, (N+2)//2 +2, 8):
    for b in range(0, 8, 4):
        X.append(np.round(temp2[a] + twiddleFactor[b] * temp2[a +4], 3))
        X.append(np.round(temp2[a +1] + twiddleFactor[b+1] * temp2[a +5], 3))
        X.append(np.round(temp2[a +2] + twiddleFactor[b+2] * temp2[a +6], 3))
        X.append(np.round(temp2[a +3] + twiddleFactor[b+3] * temp2[a +7], 3))
# print(X)

ans_X = [0j, -4j, (1.414+1.414j), 0j, 0j, (-0-0j), (1.414-1.414j), (-0+4j)]
print(X == ans_X)

# t = np.linspace(0, 4000, 1000)
# plt.plot(t, np.sin(2*np.pi*1000*t) + 0.5*np.sin(2*np.pi*2000*t+ np.pi*3/4))
# print(list(range(0, 2250, 250)))
# X.append(0)
plt.plot(range(250, 2250, 250), X)
plt.show()