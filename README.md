# ch32v305_dap

## changes in Cherry USB
in usbd_ep_start_write
commont data addr check
```c
    // if ((uint32_t)data & 0x03) {
    //     return -3;
    // }
```


## changes in Cherry DAP
disable iLandingPage
```c
    0,                        /* iLandingPage */
```