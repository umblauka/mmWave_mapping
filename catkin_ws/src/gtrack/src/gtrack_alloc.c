#include <stdlib.h>

unsigned int memoryBytesUsed = 0;

void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes)
{
    return malloc(numElements*sizeInBytes);
}
void gtrack_free(void *pFree, unsigned int sizeInBytes)
{
    free (pFree);
}
