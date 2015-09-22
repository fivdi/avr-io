'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr');

function blinkLed() {
  var count = 10,
    iv;

  i2c.writeByteSync(avr.ADDR, avr.P5_MODE, avr.OUTPUT);
  i2c.writeByteSync(avr.ADDR, avr.P5_VALUE, 1);

  iv = setInterval(function () {
    count -= 1;
    if (count === -1) {
      clearInterval(iv);
      pwmLed();
    }

    i2c.writeByteSync(avr.ADDR, avr.P5_VALUE,
      i2c.readByteSync(avr.ADDR, avr.P5_VALUE) ^ 1);
  }, 100);
}

function pwmLed() {
  var val5 = 255,
    iv;

  i2c.writeByteSync(avr.ADDR, avr.P5_MODE, avr.PWM);
  i2c.writeByteSync(avr.ADDR, avr.P5_VALUE, val5);
  
  iv = setInterval(function () {
    val5 -= 1;
    if (val5 === -1) {
      clearInterval(iv);
      blinkLed();
    }

    i2c.writeByteSync(avr.ADDR, avr.P5_VALUE, val5);
  }, 4);
}

pwmLed();

