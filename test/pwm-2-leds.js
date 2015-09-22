'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr'),
  val5 = 255,
  val6 = 0;

i2c.writeByteSync(avr.ADDR, avr.P5_MODE, avr.PWM);
i2c.writeByteSync(avr.ADDR, avr.P5_VALUE, val5);

i2c.writeByteSync(avr.ADDR, avr.P6_MODE, avr.PWM);
i2c.writeByteSync(avr.ADDR, avr.P6_VALUE, val6);

setInterval(function () {
  val5 -= 1;
  if (val5 === -1) {
    val5 = 255;
  }
  i2c.writeByteSync(avr.ADDR, avr.P5_VALUE, val5);

  val6 += 1;
  if (val6 === 256) {
    val6 = 0;
  }
  i2c.writeByteSync(avr.ADDR, avr.P6_VALUE, val6);
}, 4);

