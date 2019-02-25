#include <stdio.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include <stdlib.h>
#include <string.h>

#include <uart.h>
#include <misc.h>

OutPin led(&DDRB, &PORTB, 5);

class FlySkyIBus {
// header[2 bytes: size,cmd] channels[2x14 butes] control_summ[2 bytes]
public:	
	FlySkyIBus() {
	}

	void handleByte(uint8_t b) {
		
		if(index == 0) {
			if(b != len) {
				ready_ = false;
				return;
			} else {
				checksum = (0xFFFF - (uint16_t)b);
			}
		}

		if(index == 1) {
			if(b != cmd) {
				index = 0;
				ready_ = false;
				return;
			}
		}

		if (index < len) {
      buf[index] = b;
      if (index > 0 && index <= (len - 3)) {
        checksum -= (uint16_t)b;
      }
			index++;
    }
		if (index >= len) {
      index = 0;
			if(verifyCheckSum(checksum, buf[30], buf[31])) {
				ready_ = true;
			}
		}
	}

	bool ready() const { return ready_; }

	uint16_t channel(uint8_t ind) {
		uint8_t channelPtr = ind * 2 + 2;
		uint16_t value = buf[channelPtr + 1];
		value <<= 8;
		value |= buf[channelPtr];
		return value;
	}

private:
	bool verifyCheckSum(uint16_t compare, uint8_t low, uint8_t high) {
		return compare == ((uint16_t)low + ((uint16_t)high << 8));
	}

private:
	static constexpr uint8_t len = 32;   // size of ibus message
	static constexpr uint8_t cmd = 0x40; // command is always 0x40
	uint8_t index = 0;
	uint8_t buf[len] = {0};
	uint16_t checksum = 0;
	bool ready_ = false;
};

FlySkyIBus ibus;


class Track {
	public:
		
		Track(volatile uint8_t * ddr,
				volatile uint8_t * port,
				volatile uint8_t * value,
				uint8_t pDir1,
				uint8_t pDir2) 
		: port(port)
		, value(value)
		, dir1(OutPin(ddr, port, pDir1))
		, dir2(OutPin(ddr, port, pDir2))
		{}

		void setLimit(uint8_t limit)
		{
			max = limit;
		}
		
		void setSpeed(int8_t speed) 
		{
			if(speed == 0) {
				dir1.clear();
				dir2.clear();
				*value = 0;
			} else {
				if (speed > 0) {
					if(speed > Top) {
						speed = Top;
					}
					dir1.clear();
					dir2.set();
				} else if (speed < 0) {
					if(-speed > Top) {
						speed = -Top;
					}
					dir1.set();
					dir2.clear();
				}
				uint16_t v = (abs(speed)*max)/Top;
				*value = v;
			}
		}

	private:	
		static constexpr uint8_t Top = 50;
		volatile uint8_t * port;
		volatile uint8_t * value;
		OutPin dir1;
		OutPin dir2;
		uint8_t max;
};


class Shassis {
	public:
		Shassis()
			: rTrack(Track(&DDRC, &PORTC, &OCR0A, PC1, PC2))
			, lTrack(Track(&DDRC, &PORTC, &OCR0B, PC3, PC4))
	{
		rTrack.setLimit(254);
		lTrack.setLimit(254);
	}
		void setSpeedAndDirection(int8_t speed, int8_t direction, int8_t limit) {
			lTrack.setLimit(limit);	
			rTrack.setLimit(limit);	

			lTrack.setSpeed(speed + direction);
			rTrack.setSpeed(speed - direction);
			
			printNumb(speed - direction);
			uart_putchar(' ');
			printNumb(speed + direction);
			uart_putchar('\n');
			uart_putchar('\r');
		}

	private:
		Track rTrack;
		Track lTrack;
};

Shassis shassis;

void uart_rx_handler(unsigned char ch)
{
	ibus.handleByte(ch);
}
/*
ISR(TIMER0_OVF_vect)
{
	ibus.tick_ms(1);
}*/

int main(void)
{
	TCCR0A = _BV(WGM01) | _BV(WGM00) | _BV(COM0A1) | _BV(COM0B1); // fast pwm, non-invert
	TCCR0B = _BV(CS01) | _BV(CS00);  // prescaller is 32
	OCR0A  = 0; 
	OCR0B  = 0; 
	DDRD = _BV(PD5) | _BV(PD6);
	//TIMSK0 = _BV(TOIE0);

	set_receive_interrupt_handler(uart_rx_handler);
	uart_init(1);

	sei();
	
	while(!ibus.ready()) {
		_delay_ms(30);
		led.toggle();
	}
	
	while(1)
	{
		_delay_ms(100);

		uint8_t max = (ibus.channel(4)/10); // 100..200
		if(max < 100) max = 100;
		if(max > 200) max = 200;
		uint8_t limit = max + 55;


		int8_t speed = ibus.channel(1)/10 - 150;       // -50..50
		int8_t direction = (ibus.channel(0)/10 - 150); // -50..50
		
		
		if(abs(speed) < 3) {
			speed = 0;
		}
		if(abs(direction) < 3) {
			direction = 0;
		}
		

		shassis.setSpeedAndDirection(speed, direction, limit);
	
/*
		for(int i = 0; i < 15; ++i)
		{
			printHex(i);
			uart_putchar(':');
			uint16_t v = ibus.channel(i);
			printHex(static_cast<uint8_t>(v>>8));
			printHex(static_cast<uint8_t>(v));
			uart_putchar(' ');
			
		}
		uart_putchar('\n');
		uart_putchar('\r');
*/
		led.toggle();
	}
}

