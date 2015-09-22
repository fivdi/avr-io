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

setInterval(function () {
  var rawTemp = i2c.readWordSync(avr.ADDR, avr.A0_VALUE);

  console.log(++count + ': ' + rawTemp.toString(16) + ' ' + toCelcius(rawTemp));
}, 100);

