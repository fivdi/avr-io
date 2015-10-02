/*
 * TODOs
 * - test with multiple devices on the bus, including maybe a 2nd avr.
 * - test with pins in analog mode and then none in analog mode (adc should be turned off)
 * - provide a version number that can be accessed through i2c? At address 0xff?
 * - merge pin_t and analog_pin_t?
 * - remove analog pin timeout hack?
 * - test more with inputs, e.g., with a button
 *
 * Possible extensions
 * - Provide access to uart. A crystal will be required for this.
 * - Provide a reset command?
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>

#ifndef I2C_ADDR
#define I2C_ADDR 0x28
#endif

#define INPUT 0
#define OUTPUT 1
#define ANALOG 2
#define PWM 3
#define SERVO 4
#define INPUT_PULLUP 5
#define MAX_MODE INPUT_PULLUP

#define BINPUT (1 << INPUT)
#define BOUTPUT (1 << OUTPUT)
#define BANALOG (1 << ANALOG)
#define BPWM (1 << PWM)
#define BSERVO (1 << SERVO)
#define BINPUT_PULLUP (1 << INPUT_PULLUP)

#define PIN_COUNT 18
#define ANALOG_PIN_COUNT 4
#define ANALOG_PIN_OFFSET 14
#define REG_COUNT (PIN_COUNT * 4)

#define REG_ADDR_RECEIVED 1
#define REG_VAL_RECEIVED 2
#define REG_VAL_TRANSMITTED 3

volatile uint8_t reg_addr;
volatile uint8_t reg_val;
volatile uint8_t comm_state;

typedef struct {
  volatile uint8_t *port;
  volatile uint8_t *ddr;
  volatile uint8_t *pinput;
  volatile uint8_t *ocr;
  uint8_t bit;
  uint8_t mode;
  uint8_t supported_modes;
} pin_t;

pin_t pins[] = {
  {&PORTD, &DDRD, &PIND, 0,                             //  0 - 0x00
    1 << PD0, INPUT,                                    //  0
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  0

  {&PORTD, &DDRD, &PIND, 0,                             //  1 - 0x04
    1 << PD1, INPUT,                                    //  1
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  1

  {&PORTD, &DDRD, &PIND, 0,                             //  2 - 0x08
    1 << PD2, INPUT,                                    //  2
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  2

  {&PORTD, &DDRD, &PIND, &OCR2B,                        //  3 - 0x0c - PWM
    1 << PD3, INPUT,                                    //  3
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM},           //  3

  {&PORTD, &DDRD, &PIND, 0,                             //  4 - 0x10
    1 << PD4, INPUT,                                    //  4
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  4

  {&PORTD, &DDRD, &PIND, &OCR0B,                        //  5 - 0x14 - PWM
    1 << PD5, INPUT,                                    //  5
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM},           //  5

  {&PORTD, &DDRD, &PIND, &OCR0A,                        //  6 - 0x18 - PWM
    1 << PD6, INPUT,                                    //  6
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM},           //  6

  {&PORTD, &DDRD, &PIND, 0,                             //  7 - 0x1c
    1 << PD7, INPUT,                                    //  7
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  7

  {&PORTB, &DDRB, &PINB, 0,                             //  8 - 0x20
    1 << PB0, INPUT,                                    //  8
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  //  8

  {&PORTB, &DDRB, &PINB, &OCR1AL,                       //  9 - 0x24 - PWM SERVO
    1 << PB1, INPUT,                                    //  9
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM | BSERVO},  //  9

  {&PORTB, &DDRB, &PINB, &OCR1BL,                       // 10 - 0x28 - PWM SERVO
    1 << PB2, INPUT,                                    // 10
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM | BSERVO},  // 10

  {&PORTB, &DDRB, &PINB, &OCR2A,                        // 11 - 0x2c - PWM
    1 << PB3, INPUT,                                    // 11
    BINPUT | BOUTPUT | BINPUT_PULLUP | BPWM},           // 11

  {&PORTB, &DDRB, &PINB, 0,                             // 12 - 0x30
    1 << PB4, INPUT,                                    // 12
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  // 12

  {&PORTB, &DDRB, &PINB, 0,                             // 13 - 0x34
    1 << PB5, INPUT,                                    // 13
    BINPUT | BOUTPUT | BINPUT_PULLUP},                  // 13

  {&PORTC, &DDRC, &PINC, 0,                             // 14 - 0x38 - A0
    1 << PC0, INPUT,                                    // 14
    BINPUT | BOUTPUT | BINPUT_PULLUP | BANALOG},        // 14

  {&PORTC, &DDRC, &PINC, 0,                             // 15 - 0x3c - A1
    1 << PC1, INPUT,                                    // 15
    BINPUT | BOUTPUT | BINPUT_PULLUP | BANALOG},        // 15

  {&PORTC, &DDRC, &PINC, 0,                             // 16 - 0x40 - A2
    1 << PC2, INPUT,                                    // 16
    BINPUT | BOUTPUT | BINPUT_PULLUP | BANALOG},        // 16

  {&PORTC, &DDRC, &PINC, 0,                             // 17 - 0x44 - A3
    1 << PC3, INPUT,                                    // 17
    BINPUT | BOUTPUT | BINPUT_PULLUP | BANALOG}         // 17
};

typedef struct {
  pin_t *pin;
  uint8_t channel;
  uint16_t value;
  uint16_t tx_value;
} analog_pin_t;

analog_pin_t analog_pins[] = {
  {&pins[14], 0, 0, 0}, // 14 - A0
  {&pins[15], 1, 0, 0}, // 15 - A1
  {&pins[16], 2, 0, 0}, // 16 - A2
  {&pins[17], 3, 0, 0}  // 17 - A3
};

uint8_t current_analog_pin = UINT8_MAX;

ISR(TWI_vect)
{
  static uint8_t i2c_state = 0;

  switch(TWSR) {
    case TW_SR_SLA_ACK:   // 0x60: sla+w received, ack returned
      i2c_state=0;

      TWCR |= (1 << TWINT);
      break;

    case TW_SR_DATA_ACK:  // 0x80: data received, ack returned
      if (i2c_state == 0) {
        reg_addr = TWDR;
        i2c_state = REG_ADDR_RECEIVED;
        comm_state = REG_ADDR_RECEIVED;
      } else {
        reg_val = TWDR;
        i2c_state = REG_VAL_RECEIVED;
        comm_state = REG_VAL_RECEIVED;
      }

      TWCR |= (1 << TWINT);
      break;

    case TW_SR_STOP:      // 0xa0: stop or repeated start condition received while selected
      if (i2c_state != REG_ADDR_RECEIVED) {
        i2c_state = 0;
      }	   

      TWCR |= (1 << TWINT);
      break;

    case TW_ST_SLA_ACK:   // 0xa8: sla+r received, ack returned
    case TW_ST_DATA_ACK:  // 0xb8: data transmitted, ack received
      TWDR = reg_val;
      i2c_state = REG_VAL_TRANSMITTED;
      comm_state = REG_VAL_TRANSMITTED;

      TWCR |= (1 << TWINT);
      break;

    case TW_ST_DATA_NACK: // 0xc0: data transmitted, nack received
    case TW_ST_LAST_DATA: // 0xc8: last data byte transmitted, ack received
    case TW_BUS_ERROR:    // 0x00: illegal start or stop condition
    default:
      i2c_state = 0;

      TWCR |= (1 << TWINT);
      break;
  }
}

void pin_mode(uint8_t pin, uint8_t mode) {
  // ensure mode is valid for pin
  if (mode > MAX_MODE || !(pins[pin].supported_modes & (1 << mode))) {
    return;
  }

  // nothing to do if pin already in correct mode
  if (mode == pins[pin].mode) {
    return;
  }

  // take pin out of current mode, if required
  switch (pins[pin].mode) {
    case ANALOG:
      // if the current mode for pin is analog and a conversion is in progress
      // for the pin, cancel the conversion
      if (pin - ANALOG_PIN_OFFSET == current_analog_pin) {
        current_analog_pin = UINT8_MAX;
        ADCSRA = 0;
      }

      DIDR0 &= ((1 << analog_pins[pin - ANALOG_PIN_OFFSET].channel));
      break;

    case PWM:
      switch (pin) {
        case 3:
          // normal port operation, OC2B disconnected
          TCCR2A &= ~(1 << COM2B1);
          break;

        case 5:
          // normal port operation, OC0B disconnected
          TCCR0A &= ~(1 << COM0B1);
          break;

        case 6:
          // normal port operation, OC0A disconnected
          TCCR0A &= ~(1 << COM0A1);
          break;

        case 9:
          // normal port operation, OC1A disconnected
          TCCR1A &= ~(1 << COM1A1);
          break;

        case 10:
          // normal port operation, OC1B disconnected
          TCCR1A &= ~(1 << COM1B1);
          break;

        case 11:
          // normal port operation, OC2A disconnected
          TCCR2A &= ~(1 << COM2A1);
          break;
      }

      switch (pin) {
        case 3:
        case 11:
          if ((pin == 3 && pins[11].mode != PWM) ||
              (pin == 11 && pins[3].mode != PWM)) {
            // normal timer/counter mode
            TCCR2A &= ~(1 << WGM20);
            // no clock source, timer/counter stopped
            TCCR2B &= ~(1 << CS22);
          }
          break;

        case 5:
        case 6:
          if ((pin == 5 && pins[6].mode != PWM) ||
              (pin == 6 && pins[5].mode != PWM)) {
            // normal timer/counter mode
            TCCR0A &= ~(1 << WGM00);
            // no clock source, timer/counter stopped
            TCCR0B &= ~((1 << CS01) | (1 << CS00));
          }
          break;

        case 9:
        case 10:
          if ((pin == 9 && pins[10].mode != PWM) ||
              (pin == 10 && pins[9].mode != PWM)) {
            // normal timer/counter mode
            TCCR1A &= ~(1 << WGM10);
            // no clock source, timer/counter stopped
            TCCR1B &= ~((1 << CS11) | (1 << CS10));
          }
          break;
      }
      break;
  }

  switch (mode) {
    case INPUT:
      *(pins[pin].ddr) &= ~pins[pin].bit;
      *(pins[pin].port) &= ~pins[pin].bit;
      break;

    case OUTPUT:
      *(pins[pin].ddr) |= pins[pin].bit;
      break;

    case ANALOG:
      *(pins[pin].ddr) &= ~pins[pin].bit;
      *(pins[pin].port) &= ~pins[pin].bit;
      DIDR0 |= (1 << analog_pins[pin - ANALOG_PIN_OFFSET].channel);
      analog_pins[pin - ANALOG_PIN_OFFSET].value = 0;
      break;

    case PWM:
      switch (pin) {
        case 3:
        case 11:
          // phase correct PWM mode
          TCCR2A |= (1 << WGM20);
          // clock from prescaler divided by 64
          TCCR2B |= (1 << CS22);
          break;

        case 5:
        case 6:
          // phase correct PWM mode
          TCCR0A |= (1 << WGM00);
          // clock from prescaler divided by 64
          TCCR0B |= (1 << CS01) | (1 << CS00);
          break;

        case 9:
        case 10:
          // 8-bit phase correct pwm
          TCCR1A |= (1 << WGM10);
          // clock from prescaler divided by 64
          TCCR1B |= (1 << CS11) | (1 << CS10);
          break;
      }

      switch (pin) {
        case 3:
          // clear OC2B on match when counting up, set when counting down
          TCCR2A |= (1 << COM2B1);
          break;

        case 5:
          // clear OC0B on match when counting up, set when counting down
          TCCR0A |= (1 << COM0B1);
          break;

        case 6:
          // clear OC0A on match when counting up, set when counting down
          TCCR0A |= (1 << COM0A1);
          break;

        case 9:
          // clear OC1A on match when counting up, set when counting down
          TCCR1A |= (1 << COM1A1);
          break;

        case 10:
          // clear OC1B on match when counting up, set when counting down
          TCCR1A |= (1 << COM1B1);
          break;

        case 11:
          // clear OC2A on match when counting up, set when counting down
          TCCR2A |= (1 << COM2A1);
          break;
      }

      *(pins[pin].ddr) |= pins[pin].bit;
      *(pins[pin].ocr) = 0;

      break;

    case SERVO:
      // TODO
      break;

    case INPUT_PULLUP:
      *(pins[pin].ddr) &= ~pins[pin].bit;
      *(pins[pin].port) |= pins[pin].bit;
      break;
  }

  pins[pin].mode = mode;
}

uint16_t read_pin_value(uint8_t pin) {
  switch (pins[pin].mode) {
    case INPUT:
    case INPUT_PULLUP:
      return *(pins[pin].pinput) & pins[pin].bit ? 1 : 0;
      break;

    case OUTPUT:
      return *(pins[pin].port) & pins[pin].bit ? 1 : 0;
      break;

    case ANALOG:
      return analog_pins[pin - ANALOG_PIN_OFFSET].value;
      break;

    case PWM:
      return *(pins[pin].ocr);
      break;

    case SERVO:
      // TODO
      break;
  }

  return 0;
}

void write_pin_value(uint8_t pin, uint8_t value) {
  switch (pins[pin].mode) {
    case INPUT:
    case INPUT_PULLUP:
      // n/a
      break;

    case OUTPUT:
      if (value) {
        *(pins[pin].port) |= pins[pin].bit;
      } else {
        *(pins[pin].port) &= ~pins[pin].bit;
      }
      break;

    case ANALOG:
      // n/a
      break;

    case PWM:
      *(pins[pin].ocr) = value;
      break;

    case SERVO:
      // TODO
      break;
  }
}

uint8_t read_register(uint8_t reg) {
  uint8_t pin = (reg >> 2);

  switch (reg & 0x03) {
    case 0x00: // mode
      return pins[pin].mode;
      break;

    case 0x01: // value lsb
      if (pins[pin].mode != ANALOG) {
        return read_pin_value(pin) & 0xff;
      } else {
        analog_pins[pin - ANALOG_PIN_OFFSET].tx_value = read_pin_value(pin);
        return analog_pins[pin - ANALOG_PIN_OFFSET].tx_value & 0xff;
      }
      break;

    case 0x02: // value msb
      if (pins[pin].mode != ANALOG) {
        return 0;
      } else {
        return (analog_pins[pin - ANALOG_PIN_OFFSET].tx_value >> 8) & 0xff;
      }
      break;

    case 0x03: // reserved
    default:
      return 0;
      break;
  }
}

void write_register(uint8_t reg, uint8_t val) {
  uint8_t pin = (reg >> 2);

  switch (reg & 0x03) {
    case 0x00: // mode
      pin_mode(pin, val);
      break;

    case 0x01: // value lsb
      write_pin_value(pin, val);
      break;

    case 0x02: // value msb
      // n/a
      break;

    case 0x03: // reserved
    default:
      break;
  }
}

void adc() {
  uint8_t i, j;
/*static uint16_t terribleHack = 0;
if (terribleHack != 0) {
  terribleHack -= 1;
  return;
}*/

  if (current_analog_pin < ANALOG_PIN_COUNT) {
    // Conversion in progress, check to see if it's finished
    if (ADCSRA & (1 << ADSC)) {
      return; // Conversion not finsihed yet, wait some more
    }

    // Conversion finished
    analog_pins[current_analog_pin].value = ADC;
  }

  // Search for next analog pin starting at current_analog_pin + 1
  for (i = 1; i <= ANALOG_PIN_COUNT; ++i) {
    j = (current_analog_pin + i) % ANALOG_PIN_COUNT;
    if (analog_pins[j].pin->mode == ANALOG) {
      break;
    }
  }

  if (i <= ANALOG_PIN_COUNT) {
    current_analog_pin = j;

    // Start a conversion
    ADMUX = (1 << REFS0) | analog_pins[current_analog_pin].channel;
    ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0);
//terribleHack = 17*100;
  } else {
    // TODO - disable adc
    current_analog_pin = UINT8_MAX;
  }
}

int main(void) {
  clock_prescale_set(clock_div_1); // 8Mhz clock

  // Start listening on i2c bus
  // Clear twint flag, enable ack, enable twi, enable twi interrupt 
  TWAR = (I2C_ADDR << 1);
  TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);

  sei();

  while (1) {
    _delay_us(56);

    if (comm_state != 0) {
      if (comm_state == REG_ADDR_RECEIVED) {
        if (reg_addr >= REG_COUNT) {
          reg_addr = 0;
        }
        reg_val = read_register(reg_addr);
      } else if (comm_state == REG_VAL_RECEIVED) {
        write_register(reg_addr, reg_val);
        ++reg_addr;
        if (reg_addr >= REG_COUNT) {
          reg_addr = 0;
        }
      } else if (comm_state == REG_VAL_TRANSMITTED) {
        ++reg_addr;
        if (reg_addr >= REG_COUNT) {
          reg_addr = 0;
        }
        reg_val = read_register(reg_addr);
      }

      comm_state = 0;
    }

    adc();
  }

  return 0;
}

