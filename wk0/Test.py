import numpy as np

integers = np.array([10, 20, 30, 40, 50])

print(integers)

print(integers [0])

# integers[0] = 21.5

print(integers.dtype)

smallerIntegers = np.array(integers, dtype = np.int8)
print(smallerIntegers)

print("integers bytes", integers.nbytes)
print("smallerIntegers bytes", smallerIntegers.nbytes)

# overflow = np.array([127, 128, 129], dtype = np.int8)
# print(overflow)

floats = np.array([1.2, 2.3, 3.4, 4.5, 5.1, 8.3])
print(floats)

