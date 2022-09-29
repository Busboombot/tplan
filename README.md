# tplan 

A velocity trajectory planner for stepper motors. 


## Notes

The FastCRC library had a compile error, ' error: unknown type name 'size_t',
so it is directly included so I could modify it to include &lt;ctypes> instead of &lt;<inttypes.h>