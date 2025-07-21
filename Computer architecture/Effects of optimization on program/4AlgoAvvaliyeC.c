#include<stdio.h>
#include <stdlib.h>
#include<time.h>
#include <sys/time.h>

#define numberSets 100000

unsigned int gcd (unsigned int  a,unsigned int b){
    if (a == 0 &&b == 0)
        b = 1;
    else if (b == 0)
        b = a;
    else if (a != 0)
        while (a != b)
            if (a <b)
                b -= a;
            else
                a -= b;

    return b;
}

int main(void) {

	unsigned char numbersToGCD[numberSets][2];
	double totalTime;
	time_t t;

	/* Intializes random number generator */
	srand((unsigned) time(&t));


	for(int i = 0 ; i < numberSets ; i++) {
		for(int j = 0 ; j<2; j++)
			numbersToGCD[i][j]= (unsigned char)(rand());
	}

	struct timeval stop, start;

	for(int i = 0 ; i < numberSets ; i++) {

		gettimeofday(&start, NULL);
		gcd(numbersToGCD[i][0],numbersToGCD[i][1]);
		gettimeofday(&stop, NULL);
		totalTime += (stop.tv_usec - start.tv_usec);
	}

	printf("%f\n",totalTime/CLOCKS_PER_SEC);

}
