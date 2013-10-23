function ForcesTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(ForcesTest);

ForcesTest.prototype.testForceStatic = function() {
  var strength = 100;

  var body = new elation.physics.rigidbody({mass: 1});
  body.addForce('static', new THREE.Vector3(strength, 0, 0));
  elation.physics.system.add(body);
  elation.physics.system.start();
  
  for (var i = 0; i < this.numsteps; i++) {
    elation.physics.system.step(this.stepsize);
    expectThat(body.state.sleeping, evalsToFalse);

    // velocity = force * mass * time
    expectThat(body.velocity, isNearVector([strength * body.mass * this.stepsize * (i+1), 0, 0]));
    // position = 1/2 * force * time * time^2
    expectThat(body.position, isNearVector([.5 * strength * body.mass * Math.pow(this.stepsize * (i+1), 2), 0, 0]));
  }

  elation.physics.system.remove(body);
  expectThat(elation.physics.system.objects.length, equals(0));

  elation.physics.system.stop();
  expectThat(elation.physics.system.active, evalsToFalse);
}

