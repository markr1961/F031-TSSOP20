## STM32F031 TSSOP-20 on STM32C011-DK carrier.
git repo: none

Note: C0 and F0 do _not_ share common power or reset pins!
Boot0 must be grounded.

## Processor
STM32F031F4Px TSSOP
@ 48 MHz  

## Build environment
CubeMX  v6.12.0 using STM32Cube FW F0 v1.11.5
IAR 7.80.4  

### I/O
PA5     output  
PA6     output  
PA7     LED  
PB1     analog IN9
PA2     TX  
PA3     RX  
PA13    SWD  
PA14    SWD  
