import { elation } from '../../../cyclone.js'
import { roundToDigits } from '../test-utils.js';
import * as assert from 'assert'

describe('static forces', function() {
  const system = new elation.physics.system({substep: false});

  let tests = [
    {mass: 1, force: 1, framerate: 60, seconds: 10},
    {mass: 1000, force: 1, framerate: 60, seconds: 10},
    {mass: .1, force: 1000, framerate: 10, seconds: 100},
    {mass: 94, force: 3432, framerate: 57, seconds: 28},
  ];

    
  tests.forEach(test => {
    it(`applies ${test.force}N static force to ${test.mass}kg object for ${test.seconds}s (${test.framerate}fps)`, function(done) {
      var body = new elation.physics.rigidbody({mass: test.mass, position: new THREE.Vector3(0,0,0), velocity: new THREE.Vector3(0,0,0)});
      body.addForce('static', new THREE.Vector3(Math.random() - .5, Math.random() - .5, Math.random() - .5).normalize().multiplyScalar(test.force));

      system.add(body);

      // run for the specified number of seconds and compare with a directly-computed value
      for (var i = 0; i < test.framerate * test.seconds; i++) {
        system.step(1/test.framerate);
      }
      let dist = body.position.length();
      let expectedDist = (1/2) * (test.force / test.mass) * Math.pow(test.seconds, 2);
      //console.log('ah', body.mass, force, expectedDist, dist, body.position);
      assert.equal(roundToDigits(dist, 5), roundToDigits(expectedDist, 5));

      system.remove(body);

      done();
    });
  });
});

