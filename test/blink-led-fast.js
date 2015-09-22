'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr');

(function () {
  var time, ops, opsPerSec;

  i2c.writeByteSync(avr.ADDR, avr.P8_MODE, avr.OUTPUT);
  if (i2c.readByteSync(avr.ADDR, avr.P8_MODE) != avr.OUTPUT) {
    console.log('pin mode not set');
  }

  time = process.hrtime();

  for (ops = 1; ops <= 10000; ops += 1) {
    i2c.writeByteSync(avr.ADDR, avr.P8_VALUE, 0);
    if (i2c.readByteSync(avr.ADDR, avr.P8_VALUE) != 0) {
      console.log('pin value not set to 0');
    }
    i2c.writeByteSync(avr.ADDR, avr.P8_VALUE, 1);
    if (i2c.readByteSync(avr.ADDR, avr.P8_VALUE) != 1) {
      console.log('pin value not set to 1');
    }

    if (ops % 1000 === 0) {
      console.log(ops);
    }
  }

  time = process.hrtime(time);
  opsPerSec = Math.floor(ops / (time[0] + time[1] / 1E9));

  i2c.writeByteSync(avr.ADDR, avr.P8_VALUE, 0);

  i2c.closeSync();

  console.log();
  console.log(opsPerSec + 'Hz');
}());

