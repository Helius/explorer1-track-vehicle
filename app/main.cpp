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
		if (index == len) {
      index = 0;
			if(verifyCheckSum(checksum, buf[30], buf[31])) {
				ready_ = true;
			}
		}
	}

	void timeOut() {
		index = 0;
		ready_ = false;
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
				uint8_t pEn,
				uint8_t pDir1,
				uint8_t pDir2) 
		: port(port)
		, pwm(OutPin(ddr, port, pEn))
		, dir1(OutPin(ddr, port, pDir1))
		, dir2(OutPin(ddr, port, pDir2))
		{
		}
		
		void setSpeed(int8_t speed) 
		{
			speed_ = speed;

			if(speed_ == 0) {
				pwm.clear();
				dir1.clear();
				dir2.clear();
			} 
			else if (speed > 0) {
				if(speed_ > Top) {
					speed_ = Top;
				}
				dir1.clear();
				dir2.set();
			} else if (speed < 0) {
				if(-speed_ > Top) {
					speed_ = -Top;
				}
				dir1.set();
				dir2.clear();
			}
			pwmCount = 0;
		}

		void tick()
		{
			if(speed_ == 0) {
				return;
			}
			
			if(pwmCount > abs(speed_)) {
				pwm.clear();
			} else {
				pwm.set();
			}
			
			pwmCount++;
			if(pwmCount > Top) {
				pwmCount = 0;
			}
		}

	private:	
		static constexpr uint8_t Top = 50;
		int8_t speed_ = 0;
		uint8_t pwmCount = 0;
		volatile uint8_t * port;
		OutPin pwm;
		OutPin dir1;
		OutPin dir2;
};


class Shassis {
	public:
		Shassis()
			: rTrack(Track(&DDRC, &PORTC, PC0, PC1, PC2))
			, lTrack(Track(&DDRC, &PORTC, PC5, PC3, PC4))
	{}
		void setSpeedAndDirection(int8_t speed, int8_t direction) {
			lTrack.setSpeed(speed + direction);
			rTrack.setSpeed(speed - direction);
			
			printNumb(speed - direction);
			uart_putchar(' ');
			printNumb(speed + direction);
			uart_putchar('\n');
			uart_putchar('\r');
		}

		void tick() {
			rTrack.tick();
			lTrack.tick();
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

ISR(TIMER0_COMPA_vect)
{
	shassis.tick();
}

int main(void)
{
	// setup timer (16MHz/80 = 200KHz)
	TCCR0A = _BV(WGM01); // ctc mode
	TCCR0B = _BV(CS01);  // prescaller is 1
	TIMSK0 = _BV(OCIE0A);// top is OCR0A
	OCR0A  = 250; 

	set_receive_interrupt_handler(uart_rx_handler);
	uart_init(1);

	sei();
	while(1)
	{
		_delay_ms(100);

		int8_t direction = ibus.channel(0)/10 - 150;
		int8_t speed = ibus.channel(1)/10 - 150;
		shassis.setSpeedAndDirection(speed, direction);
	
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

