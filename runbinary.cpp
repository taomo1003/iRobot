#include <iostream>

using namespace std;
int main(int argc, char const *argv[])
{
	int array1[10];
	int array2[20];

  while (1) {
    for (int j = 0; j < 10000; j++) {
      array1[j % 10 ] = array2[j % 10] * 20- array2[j % 10];
    }
  for (int i = 0; i < 10; i++) {
    array1[i] = 6 * i + i * i - i % 10;
    array2[i] = array1[i] - array1[(i+60) % 10] * 60;
  }
  }
  
	
return 0;
}
