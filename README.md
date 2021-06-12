# 4point_probe
Four point probe is used to measure resistive properties of semiconductor wafers and thin films [[1](https://www.pveducation.org/pvcdrom/characterisation/four-point-probe-resistivity-measurements)].
![4PP](https://github.com/Oerdna/4point_probe/blob/main/img/fig_4_pp.png)

***
In this case are studing the I - V characteristic of an element depending on the temperature of the sample. It is important to understand the I – V characteristic near the metal-insulator phase transition (MIT), which is about 68 degrees Celsius. You must understand that the ultimate goal is to stabilize the temperature to a specific value. More about the TEC control [[2](https://www.maximintegrated.com/en/design/technical-documents/tutorials/1/1757.html)].

## Stack
* Nucleo STM32L432KC
* Main PCB
* TEC element
* Termopair - MAX6675
* ADC - MCP3201
