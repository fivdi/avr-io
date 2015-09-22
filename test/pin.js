'use strict';

var i2c = require('i2c-bus').openSync(1),
  avr = require('./avr'),
  assert = require('assert');

var PIN8_VALUE_LSB = 0x21,
  PIN8_VALUE_MSB = 0x22,
  PIN8_RESERVED = 0x23;

function rbs(reg) {
  return i2c.readByteSync(avr.ADDR, reg);
}

function wbs(reg, val) {
  return i2c.writeByteSync(avr.ADDR, reg, val);
}

function rws(reg) {
  return i2c.readWordSync(avr.ADDR, reg);
}

function wws(reg, val) {
  return i2c.writeWordSync(avr.ADDR, reg, val);
}

wbs(avr.P8_MODE, avr.OUTPUT);
assert.equal(rbs(avr.P8_MODE), avr.OUTPUT, "expected pin8 to be an output");

wws(avr.P8_VALUE, 0xffff);
assert.equal(rbs(avr.P8_VALUE), 1, "expected pin8 value to be 1");
assert.equal(rbs(PIN8_VALUE_LSB), 1, "expected pin8 value lsb to be 1");
assert.equal(rbs(PIN8_VALUE_MSB), 0, "expected pin8 value msb to be 0");

wws(avr.P8_VALUE, 0xff00);
assert.equal(rbs(avr.P8_VALUE), 0, "expected pin8 value to be 0");
assert.equal(rbs(PIN8_VALUE_LSB), 0, "expected pin8 value lsb to be 0");
assert.equal(rbs(PIN8_VALUE_MSB), 0, "expected pin8 value msb to be 0");

wws(avr.P8_VALUE, 0x00ff);
assert.equal(rbs(avr.P8_VALUE), 1, "expected pin8 value to be 1");
assert.equal(rbs(PIN8_VALUE_LSB), 1, "expected pin8 value lsb to be 1");
assert.equal(rbs(PIN8_VALUE_MSB), 0, "expected pin8 value msb to be 0");

wbs(avr.P8_VALUE, 0);
assert.equal(rbs(avr.P8_VALUE), 0, "expected pin8 value to be 0");
assert.equal(rbs(PIN8_VALUE_LSB), 0, "expected pin8 value lsb to be 0");
assert.equal(rbs(PIN8_VALUE_MSB), 0, "expected pin8 value msb to be 0");

wws(avr.P8_VALUE, 0x0010);
assert.equal(rbs(avr.P8_VALUE), 1, "expected pin8 value to be 1");
assert.equal(rbs(PIN8_VALUE_LSB), 1, "expected pin8 value lsb to be 1");
assert.equal(rbs(PIN8_VALUE_MSB), 0, "expected pin8 value msb to be 0");

wbs(PIN8_RESERVED, 0);
assert.equal(rbs(PIN8_RESERVED), 0, "expected pin8 reserved to be 0");

wbs(avr.P8_VALUE, 0);

