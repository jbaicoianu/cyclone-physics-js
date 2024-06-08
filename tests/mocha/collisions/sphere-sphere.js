import { elation } from '../../../cyclone.mjs'
import * as assert from 'assert'

import { roundToDigits } from '../test-utils.mjs';

describe('sphere-sphere collisions', function() {
  var system = new elation.physics.system();

  var body1 = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(-10,0,0), velocity: new THREE.Vector3(1,0,0)});
  body1.setCollider('sphere', 1);

  var body2 = new elation.physics.rigidbody({mass: 2, position: new THREE.Vector3(10,0,0), velocity: new THREE.Vector3(0,0,0)});
  body2.setCollider('sphere', 1);

  system.add(body1);
  system.add(body2);

  let tests = [
    [
      [ 1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(1, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ 1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(10, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ 1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(100, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(1, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(10, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .1, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(100, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .001, new THREE.Vector3(-10, .9, 0), new THREE.Vector3(1, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .001, new THREE.Vector3(-10, .9, 0), new THREE.Vector3(10, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .001, new THREE.Vector3(-10, 1.0009, 0), new THREE.Vector3(100, 0, 0)],
      [ 1, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .22, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(.1, 0, 0)],
      [ .232, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .22, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(1, 0, 0)],
      [ .232, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .22, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(10, 0, 0)],
      [ .232, new THREE.Vector3(10, 0, 0), new THREE.Vector3(0, 0, 0)],
    ],
    [
      [ .22, new THREE.Vector3(-10, 0, 0), new THREE.Vector3(10, 0, 0)],
      [ .232, new THREE.Vector3(10, 0, 0), new THREE.Vector3(-290, 0, 0)],
    ],
  ];

  tests.forEach(bodies => {
    let b1 = bodies[0],
        b2 = bodies[1];
    it(`should collide a ${b1[0]}m and ${b2[0]}m sphere at ${b1[2].clone().sub(b2[2]).length()}m/s`, function(done) {
      body1.collider.radius = b1[0];
      body1.position.copy(b1[1]);
      body1.velocity.copy(b1[2]);

      body2.collider.radius = b2[0];
      body2.position.copy(b2[1]);
      body2.velocity.copy(b2[2]);

      let state = {
        crashes: 0,
        collisions: []
      };
        
      elation.events.add(body1, 'physics_collide', function(ev) {
        state.collisions.push(ev.data);
        //console.log('boink', body1.position.distanceTo(body2.position), body1.collider.radius + body2.collider.radius);
        let dist = body1.position.distanceTo(body2.position),
            expectedDist = body1.collider.radius + body2.collider.radius;
        //console.log(dist, expectedDist);
        assert.equal(roundToDigits(dist, 3), expectedDist, `collided with ${expectedDist}m between centers`);
      });
      elation.events.add(body2, 'physics_collide', function(ev) {
        state.collisions.push(ev.data);
      });

      assert.strict(state.crashes == 0)

      for (var i = 0; i < 100000; i++) {
        system.step(1/60 + Math.random() / 100);
        if (state.collisions.length >= 2) {
          //console.log('crash!', body1.position.distanceTo(body2.position), state);
          state.crashes++;
          break;
        }
        //console.log('no crash!', body1.position.distanceTo(body2.position), body1.position, body1.velocity, body2.position, body2.velocity);
      }
      assert.strict(state.crashes > 0, 'collided as expected')
      //console.log('velocities:', body1.velocity.length(), body2.velocity.length());
      //assert.equal(roundToDigits(body1.velocity.length() / body2.velocity.length(), 5), roundToDigits(body1.mass / body2.mass, 5), 'velocity ratio should equal mass ratio');
      done();
    });
  });
});

