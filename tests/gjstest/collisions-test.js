function CollisionsTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(CollisionsTest);

CollisionsTest.prototype.testSphereSphere = function() {
  var system = new elation.physics.system();

  var body1 = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(-10,0,0), velocity: new THREE.Vector3(1,0,0)});
  body1.setCollider('sphere', 1);

  var body2 = new elation.physics.rigidbody({mass: 2, position: new THREE.Vector3(10,0,0), velocity: new THREE.Vector3(-1,0,0)});
  body2.setCollider('sphere', 1);

  system.add(body1);
  system.add(body2);

  var state = {
    crashes: 0,
    collisions: []
  };
  elation.events.add(body1, 'physics_collide', function(ev) { state.collisions.push(ev.data); });
  elation.events.add(body2, 'physics_collide', function(ev) { state.collisions.push(ev.data); });

  for (var i = 0; i < this.numsteps; i++) {
    system.step(this.stepsize);
    if (state.collisions.length >= 2) {
      console.log('crash!');
      state.crashes++;
      state.collisions = [];
    }
    expectThat(state.crashes, lessThan(2));
    if (state.crashes == 0) {
      expectThat(body1.velocity.length(), isNearNumber(body2.velocity.length(), 1e-6));
    } else {
      // FIXME - this is incorrect
      // v = ((m1 - m2) * vi + 2 * m2 * vi) / (m1 + m2)
      //expectThat(body1.velocity.length(), isNearNumber(((body1.mass - body2.mass) + 2 * body2.mass) / (body1.mass + body2.mass), 1e-6));
    }
  }
  expectThat(state.crashes, equals(1));
  system.remove(body1);
  system.remove(body2);
}

CollisionsTest.prototype.testSpherePlane = function() {
  var system = new elation.physics.system();

  var plane = new elation.physics.rigidbody({mass: 0, position: new THREE.Vector3(0,10,0), velocity: new THREE.Vector3(0,0,0)});
  plane.setCollider('plane', {normal: new THREE.Vector3(0,1,0)});

  var sphere = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(5,10,5), velocity: new THREE.Vector3(0,-1,0)});
  sphere.setCollider('sphere', 1);

  system.add(plane);
  system.add(sphere);

  var state = {
    crashes: 0,
    collisions: []
  };
  elation.events.add(plane, 'physics_collide', function(ev) { state.collisions.push(ev.data); });
  elation.events.add(sphere, 'physics_collide', function(ev) { state.collisions.push(ev.data); });

  for (var i = 0; i < this.numsteps; i++) {
    system.step(this.stepsize);
    if (state.collisions.length >= 2) {
      var c = state.collisions[0];
      console.log('crash! ' + c.bodies[0] + '/' + c.bodies[1]);
      state.crashes++;
      state.collisions = [];
    }
    expectThat(state.crashes, lessThan(2));
    if (state.crashes == 0) {
      expectThat(sphere.velocity.length(), isNearNumber(1, 1e-6));
    }
  }
}
CollisionsTest.prototype.testSphereCylinder = function() {
  var system = new elation.physics.system();

  var sphere = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(10,0,0), velocity: new THREE.Vector3(-1,0,0)});
  sphere.setCollider('sphere', 1);

  var cylinder = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(0,0,0), velocity: new THREE.Vector3(0,0,0)});
  cylinder.setCollider('cylinder', {height: 2, radius: 1});

  system.add(sphere);
  system.add(cylinder);

  elation.events.add(sphere, 'physics_collide', function(ev) { console.log('sphere crash'); state.collisions.push(ev.data); });
  elation.events.add(cylinder, 'physics_collide', function(ev) {console.log('cylinder crash');  state.collisions.push(ev.data); });

  var state = {
    crashes: 0,
    collisions: []
  };

  for (var i = 0; i < this.numsteps; i++) {
    system.step(this.stepsize);
    if (state.collisions.length >= 2) {
      var c = state.collisions[0];
      console.log('crash! ' + c.bodies[0] + '/' + c.bodies[1]);
      state.crashes++;
      state.collisions = [];
    }
    expectThat(state.crashes, lessThan(2));
    if (state.crashes == 0) {
      //expectThat(sphere.velocity.length(), isNearNumber(1, 1e-6));
    }
  }
}
