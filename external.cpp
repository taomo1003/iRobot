#include "external.hh"
#include <stdlib.h>
#include <stdio.h>

using namespace std;

void* external(void* parms) 
{	
	pid_t pid = 1;
	char *execArgs[] = { (char*)"runBinary", NULL };
	
	for (int i = 0; i < 100; ++i)
	{
		pid = fork();
		
		if (pid < 0)
		{
			cout << "Error in External forking" << endl;
		}
		else if (pid == 0)
		{
			execvp("./runBinary", execArgs);
			cout << "External not found" << endl;
			return nullptr;
		}
	}

    return 0;
}
