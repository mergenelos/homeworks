import os
import time

def bytes_to_int(bytes):
    result = 0

    for b in bytes:
        result = result * 256 + int(b)

    return result

def gcd(a,b):
	gcd = 1
	if (a == 0):
		return b
	if (b == 0):
		return a

	# base case
	if (a == b):
		return a

	for i in range(1,min(a,b)):
		if a%i==0 and b%i==0:
			gcd = i;
	return gcd


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
