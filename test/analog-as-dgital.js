'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr');

i2c.writeByteSync(avr.ADDR, avr.A0_MODE, avr.INPUT);
i2c.writeByteSync(avr.ADDR, avr.A1_MODE, avr.INPUT);
i2c.writeByteSync(avr.ADDR, avr.A2_MODE, avr.INPUT);
i2c.writeByteSync(avr.ADDR, avr.A3_MODE, avr.INPUT);

(function() {
  var ops = 0,
   i1, i2, i3, i4;

  while (true) {
    ops += 1;

    i1 = i2c.readByteSync(avr.ADDR, avr.A0_VALUE);
    i2 = i2c.readByteSync(avr.ADDR, avr.A1_VALUE);
    i3 = i2c.readByteSync(avr.ADDR, avr.A2_VALUE);
    i4 = i2c.readByteSync(avr.ADDR, avr.A3_VALUE);

    console.log(ops + ': ' + i1 + ' ' + i2 + ' ' + i3 + ' ' + i4);
  }
}());

