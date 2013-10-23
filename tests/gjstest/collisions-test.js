function CollisionsTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(CollisionsTest);

CollisionsTest.prototype.testSphereSphere = function() {
  var body1 = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(-10,0,0), velocity: new THREE.Vector3(1,0,0)});
  body1.setCollider('sphere', 1);

  var body2 = new elation.physics.rigidbody({mass: 1, position: new THREE.Vector3(10,0,0), velocity: new THREE.Vector3(-1,0,0)});
  body2.setCollider('sphere', 1);

  elation.physics.system.add(body1);
  elation.physics.system.add(body2);

  var state = {
    collisions: []
  };
  elation.events.add(body1, 'physics_collide', function(ev) { state.collisions.push(ev.data); });
  elation.events.add(body2, 'physics_collide', function(ev) { state.collisions.push(ev.data); });

  for (var i = 0; i < this.numsteps; i++) {
    elation.physics.system.step(this.stepsize);
    if (state.collisions.length >= 2) {
      console.log('crash!');
      state.collisions = [];
      //break;
    }
  }
  
}

