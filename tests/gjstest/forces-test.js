function ForcesTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(ForcesTest);

ForcesTest.prototype.testForceStatic = function() {
  var strength = 100;

  var system = new elation.physics.system();

  var body = new elation.physics.rigidbody({mass: 1});
  body.addForce('static', new THREE.Vector3(strength, 0, 0));
  system.add(body);
  system.start();
  
  for (var i = 0; i < this.numsteps; i++) {
    system.step(this.stepsize);
    expectThat(body.state.sleeping, evalsToFalse);

    // velocity = force * mass * time
    expectThat(body.velocity, isNearVector([strength * body.mass * this.stepsize * (i+1), 0, 0]));
    // position = 1/2 * force * time * time^2
    expectThat(body.position, isNearVector([.5 * strength * body.mass * Math.pow(this.stepsize * (i+1), 2), 0, 0]));
  }

  system.remove(body);
  expectThat(system.objects.length, equals(0));

  system.stop();
  expectThat(system.active, evalsToFalse);
}

ForcesTest.prototype.testForceStaticOffCenter = function() {
  var system = new elation.physics.system();

  var body = new elation.physics.rigidbody({mass: 10});
  body.setCollider('box', {min: new THREE.Vector3(-1,-1,-1), max: new THREE.Vector3(1,1,1)});
  body.addForce('static', {force: new THREE.Vector3(0,0,-1), point: new THREE.Vector3(1,0,0)});
  system.add(body);
  system.start();

  for (var i = 0; i < this.numsteps; i++) {
    system.step(this.stepsize);
    expectThat(body.velocity, not(isNearVector([0, 0, 0]))); 
    expectThat(body.angular, not(isNearVector([0, 0, 0]))); 

    // FIXME - this just checks the general direction.  we should do fancy math here and figure out the real expected values
    // Object should move forward and slightly to the left before it starts spinning quickly in-place with a slight wobble
    //expectThat(body.velocity.x, lessOrEqual(0)); 
    expectThat(body.velocity.y, equals(0)); 
    expectThat(body.velocity.z, lessOrEqual(0)); 
    expectThat(body.angular.x, equals(0)); 
    expectThat(body.angular.y, greaterThan(0)); 
    expectThat(body.angular.z, equals(0)); 
  }
}
