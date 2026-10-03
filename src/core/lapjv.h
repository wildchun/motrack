#pragma once

#include <cstddef>

namespace motrack
{
    int lapjv_internal(const size_t n, double *cost[], int *x, int *y);
}