#include <cstddef>

#include "cmsis_os.h"

/* overload c memory allocator */

extern "C" void* __wrap_malloc(size_t size) {
    return pvPortMalloc(size);
}

extern "C" void __wrap_free(void* ptr) {
    vPortFree(ptr);
}

/* overload c++ default dynamic memory allocator */

void* operator new(size_t size) {
    return pvPortMalloc(size);
}

void* operator new[](size_t size) {
    return pvPortMalloc(size);
}

void operator delete(void* ptr) {
    vPortFree(ptr);
}

void operator delete(void* ptr, unsigned int) {
    vPortFree(ptr);
}

void operator delete[](void* ptr) {
    vPortFree(ptr);
}

void operator delete[](void* ptr, unsigned int) {
    vPortFree(ptr);
}
