#include "external.hh"
#include <stdlib.h>
#include <stdio.h>

void* external(void* parms) {
    system ("./runBinary");
    return 0;
}
