'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr');

(function () {
  var time, output, input, errors = 0, ops, opsPerSec;

  i2c.writeByteSync(avr.ADDR, avr.P2_MODE, avr.INPUT);
  i2c.writeByteSync(avr.ADDR, avr.P4_MODE, avr.OUTPUT);

  time = process.hrtime();

  for (ops = 1; ops <= 10000; ops += 1) {
    output = ops % 2;

    i2c.writeByteSync(avr.ADDR, avr.P4_VALUE, output);
    input = i2c.readByteSync(avr.ADDR, avr.P2_VALUE);
    if (output != input) {
      errors += 1;
    }

    if (ops % 1000 === 0) {
      console.log(ops + ' ops, ' + errors + ' errors');
    }
  }

  time = process.hrtime(time);
  opsPerSec = Math.floor(ops / (time[0] + time[1] / 1E9));

  i2c.closeSync();

  console.log();
  console.log(opsPerSec + ' ops per second');
}());

