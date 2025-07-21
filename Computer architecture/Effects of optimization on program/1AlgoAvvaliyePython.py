import os
import time
import sys


def bytes_to_int(bytes):
    result = 0

    for b in bytes:
        result = result * 256 + int(b)

    return result

def gcd(a,b):
	if (a == 0):
		return b
	if (b == 0):
		return a

	# base case
	if (a == b):
		return a

	# a is greater
	if (a > b):
		return gcd(a-b, b)
	return gcd(a, b-a)


if __name__=="__main__":
	timingOfThousandRun = 0
	numbersToGCD = []
	numberOfRuns = 100000
	for i in range(0,numberOfRuns):
		numbersToGCD.append((bytes_to_int(os.urandom(1)),bytes_to_int(os.urandom(1))))
	for i in numbersToGCD:
		t0 = time.process_time()
		gcdVal = gcd(i[0], i[1]);
		t1 = time.process_time()
		timingOfThousandRun += (t1-t0)

	print("Program run for "+str(timingOfThousandRun)+" seconds.")
	print("Each GCD calculation took "+str(timingOfThousandRun/numberOfRuns)+" seconds.")
