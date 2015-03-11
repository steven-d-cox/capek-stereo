
#pragma once

#ifdef __cplusplus
extern "C" 
{
#endif

    int fact(int x);

    // (2) void usage(char * exec);

    // (3) void * Params_create();
    // (3) void Params_destroy(void * p);

    // (4) void Params_get_nx(void * p);
    // (4) void Params_set_nx(void * p, int value);

#ifdef __cplusplus
}
#endif

    /*

      Create a shared library from 'my-functions' (only)
      Call 'fact' from python

      // Create c bindings from the static library
      (1) Extend example 'fact' such that shared library links supplied library
      (2) Extend example to implement usage and call from python
      (3) ibid, for create/destroy
      (4) ibid, for get/set nx ny



     */


