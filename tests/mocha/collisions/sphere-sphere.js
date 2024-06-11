import cyclone from '../../../cyclone.js'
import { roundToDigits } from '../test-utils.js';
import * as assert from 'assert'

describe('sphere-sphere collisions', function() {

  let tests = [
    {
      bodies: [
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(-1, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(10, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(100, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(-1, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(10, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.1 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(100, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.001, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(-1, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.001, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(10, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.001, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(100, 0, 0) },
        { mass: 1, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      seconds: 250,
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.22 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(0.1, 0, 0) },
        { mass: 0.25, collider: 'sphere', collider_args: { radius: 0.232 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      seconds: 20,
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
        { mass: 0.25, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(10, 0, 0) },
        { mass: 0.25, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(-290, 0, 0) },
      ],
    },
    {
      bodies: [
        { mass: 0.2, collider: 'sphere', collider_args: { radius: 0.001 }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(10, 0, 0) },
        { mass: 0.25, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
      ],
    },
  ];
  tests.forEach(test => {
    let obj1 = test.bodies[0],
        obj2 = test.bodies[1];
    let obj1vel = obj1.velocity.length(),
        obj2vel = obj2.velocity.length();
    let fps = test.fps || 60,
        seconds = test.seconds || 10;
    it(`${obj1.collider}(${obj1.mass > 0 ? obj1.mass + 'kg mass' : 'static'}, ${obj1.collider_args.radius}m radius, ${obj1vel == 0 ? 'stationary' : 'moving ' + obj1vel + 'm/s'}) vs ${obj2.collider}(${obj2.mass > 0 ? obj2.mass + 'kg mass' : 'static'}, ${obj2.collider_args.radius}m radius, ${obj2vel == 0 ? 'stationary' : 'moving ' + obj2vel + 'm/s'})`, function(done) {
      let system = new cyclone.system();
      let state = {
        crashes: 0,
        collisions: []
      };
      let bodies = [];
      test.bodies.forEach(bodyargs => {
        //console.log('- add body:', bodyargs);
        let body = new cyclone.rigidbody(bodyargs);
        body.setCollider(bodyargs.collider, bodyargs.collider_args);
        body.addEventListener('physics_collide', ev => {
          state.collisions.push(ev.detail);
          let contact = ev.detail;
          let dist = contact.bodies[0].position.distanceTo(contact.bodies[1].position),
              expectedDist = contact.bodies[0].collider.radius + contact.bodies[1].collider.radius;
          assert.equal(roundToDigits(dist, 3), expectedDist, `collided with ${expectedDist}m between centers`);
        });
        bodies.push(body);
        system.add(body);
      });
        
      assert.strict(state.crashes == 0)

      for (var i = 0; i < fps * seconds; i++) {
        system.step(1/fps);
        //console.log(obj1.position, obj2.position)
      }
      assert.equal(state.collisions.length, 2, 'collided as expected')

      //console.log('velocities:', body1.velocity.length(), body2.velocity.length());
      //assert.equal(roundToDigits(body1.velocity.length() / body2.velocity.length(), 5), roundToDigits(body1.mass / body2.mass, 5), 'velocity ratio should equal mass ratio');

      bodies.forEach(body => {
        system.remove(body);
      });
      done();
    });
  });
});

