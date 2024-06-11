import cyclone from '../../../cyclone.js'

import { roundToDigits } from '../test-utils.js';
import * as assert from 'assert'

describe('gravity forces', function() {
  let dropTests = [
    { position: new cyclone.vector3(0, 0, 0), gravity: -9.8, velocity: new cyclone.vector3(0, 100, 0), mass: 1, framerate: 60},
  ];
  const system = new cyclone.system({substep: false});
  it('should apply gravity forces reliably');
  dropTests.forEach(test => {
    it(`object with initial velocity of (${test.velocity.toArray().join(',')}) released from (${test.position.toArray().join(',')}) with gravity of ${test.gravity}m/s^2`, done => {
      let body = new cyclone.rigidbody({mass: test.mass, position: test.position, velocity: test.velocity});
      body.addForce('gravity', new cyclone.vector3(0, test.gravity, 0));

      system.add(body);

      // run for the specified number of seconds and compare with a directly-computed value
      let elapsedTime = 0;
      do {
        let stepTime = 1 / test.framerate;
        system.step(stepTime);
        elapsedTime += stepTime;
      } while (body.position.y > 0);
      console.log('reached ground', elapsedTime);

      // TODO - compare with known good values

      done();
    });
  });
});

