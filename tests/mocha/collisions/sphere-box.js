import cyclone from '../../../cyclone.js'
import * as assert from 'assert'

describe('sphere-cube collisions', function() {
  let tests = [
    { 
      bodies: [
        { mass: 0, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(0, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
        { mass: 1, collider: 'box', collider_args: { min: new cyclone.vector3(-1, -1, -1), max: new cyclone.vector3(1, 1, 1) }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
      ]
    },
    { 
      bodies: [
        { mass: 10, collider: 'sphere', collider_args: { radius: 5 }, position: new cyclone.vector3(10, 0, 0), velocity: new cyclone.vector3(-1, 0, 0) },
        { mass: 1, collider: 'box', collider_args: { min: new cyclone.vector3(-.2, -.2, -.2), max: new cyclone.vector3(.2, .2, .2) }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
      ]
    },
    { 
      bodies: [
        { mass: 10, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(0, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
        { mass: 10, collider: 'box', collider_args: { min: new cyclone.vector3(-1, -1, -1), max: new cyclone.vector3(1, 1, 1) }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
      ]
    },
    { 
      bodies: [
        { mass: 10, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(0, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
        { mass: 10, collider: 'box', collider_args: { min: new cyclone.vector3(-1, -1, -1), max: new cyclone.vector3(1, 1, 1) }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
      ]
    },
    { 
      bodies: [
        { mass: 10, collider: 'sphere', collider_args: { radius: 1 }, position: new cyclone.vector3(0, 0, 0), velocity: new cyclone.vector3(0, 0, 0) },
        { mass: 0, collider: 'box', collider_args: { min: new cyclone.vector3(-1, -1, -1), max: new cyclone.vector3(1, 1, 1) }, position: new cyclone.vector3(-10, 0, 0), velocity: new cyclone.vector3(1, 0, 0) },
      ]
    },
  ];
  tests.forEach(test => {
    let obj1 = test.bodies[0],
        obj2 = test.bodies[1];
    let cubedims = new cyclone.vector3().subVectors(obj2.collider_args.max, obj2.collider_args.min),
        obj1vel = obj1.velocity.length(),
        obj2vel = obj2.velocity.length();
    it(`${obj1.collider}(${obj1.mass > 0 ? obj1.mass + 'kg mass' : 'static'}, ${obj1.collider_args.radius}m radius, ${obj1vel == 0 ? 'stationary' : 'moving ' + obj1vel + 'm/s'}) vs ${obj2.collider}(${obj2.mass > 0 ? obj2.mass + 'kg mass' : 'static'}, ${obj1.collider_args.radius}m radius, ${obj2vel == 0 ? 'stationary' : 'moving ' + obj2vel + 'm/s'})`, function(done) {
    //it(`${sphere.mass > 0 ? 'dynamic' : 'static'} ${sphere.collider_args.radius}m sphere ${spherevel == 0 ? '(stationary)' : '@ ' + spherevel + 'm/s'} with ${cube.mass > 0 ? 'dynamic' : 'static'} ${cubedims.toArray().join('m x ')}m cube @ ${cube.velocity.length()}m/s`, function(done) {
      let system = new cyclone.system();
      let bodies = [];
      let stats = {
        collisions: []
      }
      test.bodies.forEach(bodyargs => {
        //console.log('- add body:', bodyargs);
        let body = new cyclone.rigidbody(bodyargs);
        body.setCollider(bodyargs.collider, bodyargs.collider_args);
        body.addEventListener('physics_collide', ev => { stats.collisions.push(ev.detail); });
        bodies.push(body);
        system.add(body);
      });

      let state = {};
      for (var i = 0; i < 1000; i++) {
        system.step(1/60);
        //console.log(bodies[0].position);
        //if (stats.collisions.length > 0) break;
      }
      done();

      assert.equal(stats.collisions.length, 2, 'received exactly 2 collision events');
    });
  });
});
