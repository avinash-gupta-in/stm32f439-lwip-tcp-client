This is a simple TCP client example for STM32F439 Nucleo board.
It uses the internal Ethernet MAC of the STM32F439. On software
end it uses lwIP and FreeRTOS.

On your PC,  use application like Hercules to create a TCP sever
which is listening on PORT 5000.

Feed the IP of your PC on line number 322 in this file i.e. main.c
To find out the IP address of PC use the command ipconfig (Windows)

This codes uses static IP for this board (no DHCP). The
IP address can be changed using the CubeMX GUI (load .ioc file)
I have used the IP 192.168.1.120
You can confirm the same by going through the implementation
of the function.
 
void MX_LWIP_Init(void)
 
This application first connects with the server (PC) and then sends
some text messages to the server, which we can view on Hercules
terminal. Then its disconnects the socket.
 
Hardware: STM32F439 Nucleo Board
 
This code is a part of my PAID course here at
https://www.stm32tutorials.com
Author: Avinash Gupta

![STM32F439 Board Running lwIP and FreeRTOS](https://www.extremeelectronics.co.in/github/stm32-lwip/stm32f439_nucleo.jpg)

## Debug Messages
The Nucleo 439 has in built USB to UART convertor which is connected to USART3
peripheral of the main microcontroller (STM32F439). So we can easily printf to
the USART3 and view those messages on our PC using terminal programs like 
RealTerm.

![STM32F439 Board Running lwIP and FreeRTOS](https://www.extremeelectronics.co.in/github/stm32-lwip/stm32f439-tcp-client-example-lwip.png)

