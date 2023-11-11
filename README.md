# STM32F410RB

## tim1 
* Complementary pwm (up to 4 phases) with dead zone
* Drive by APB2 clock (84MHz default on nucleo)
* prescaler set to 84 => timer frequency = 1 MHz
* prescaler set to 1000 => PWM period = 1 ms
* default pulse set to 500 => duty cycle = 50 %


## serial protocol:
uart1 (TX-PB6 + RX-PA10) + uart2 (USB) are activated

The both uart are IRQ managed in parallel, they work exactly the same way.

### UART Commands
* `?` : get duty cycles. answer `=xxxx,yyyy\r\n`
* `#xxxx+yyyy` : set left duty cycle to xxxx and right to yyyy (xxxx and yyyy truncated at 1000)
	answer `!xxxx,yyyy\r\n`

**Blue Button**: change duty-cyles by 10% (cycling up and down) + toggle LD2
**Black Button**: reset set CPU and set duty cycles to 50% + uart answer: `!0500,0500\r\n`

## Output:
 Name | pin 
 --- | :---:
 UART1 RX| PA10
 UART1 TX| PB6
 Tim1 chan1N| PA7
 Tim1 chan2| PA9
 Tim1 chan3N| PB1
 Tim1 chan4| PB11

![STM32 pinout](doc/pinout-nucleo32-F410.png)

------

## TODO
* add enable signal to stop motors (BREAK / URGENCY Button)
* manage rotary encoder and return speed?
* manage jerk (perhaps with ROS2)
* Add IMU
* Mesure full speed


