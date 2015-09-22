'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr'),
  count = 0;

function toCelcius(rawValue) {
  var voltage = rawValue * 3.3;
  voltage /= 1024.0; 
  var temperatureC = (voltage - 0.5) * 100;
  return temperatureC;
}

i2c.writeByteSync(avr.ADDR, avr.A0_MODE, avr.ANALOG);
i2c.writeByteSync(avr.ADDR, avr.A1_MODE, avr.ANALOG);
i2c.writeByteSync(avr.ADDR, avr.A2_MODE, avr.ANALOG);
i2c.writeByteSync(avr.ADDR, avr.A3_MODE, avr.ANALOG);

setInterval(function () {
  var rawTemp, rawLdr, half, full;

  rawTemp = i2c.readWordSync(avr.ADDR, avr.A0_VALUE);
  rawLdr = i2c.readWordSync(avr.ADDR, avr.A1_VALUE);
  half = i2c.readWordSync(avr.ADDR, avr.A2_VALUE);
  full = i2c.readWordSync(avr.ADDR, avr.A3_VALUE);

  console.log(++count + ': ' + rawTemp.toString(16) + ' ' + toCelcius(rawTemp) + ', '  + rawLdr.toString(16) + ', '  + half.toString(16) + ', '  + full.toString(16));
}, 100);

