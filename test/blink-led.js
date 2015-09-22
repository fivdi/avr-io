'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr');

i2c.writeByteSync(avr.ADDR, avr.P8_MODE, avr.OUTPUT);

setInterval(function () {
  i2c.writeByteSync(avr.ADDR, avr.P8_VALUE,
    i2c.readByteSync(avr.ADDR, avr.P8_VALUE) ^ 1);
}, 500);

