'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr'),
  i;

for (i = 0; i <= 17; i += 1) {
  i2c.writeByteSync(avr.ADDR, i * 4, avr.INPUT);
}

