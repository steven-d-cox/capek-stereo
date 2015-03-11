
#include "functions.h"
#include "params.hpp"

#define PARAM(p) ((Params *) p)

int fact(int x)
{
    if(x == 0) return 1;
    return x * fact(x-1);
}

void * Params_create()
{
    return new Params();
}

void Params_destroy(void * p)
{
    delete static_cast<Params *>(p);
}

void Params_get_nx(void * p)
{
   return PARAMS(p)->nx;
}

void Params_set_nx(void * p, int value)
{
   PARAMS(p)->nx = value;
}


