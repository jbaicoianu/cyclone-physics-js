function ElationTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(ElationTest);
if (typeof console == 'undefined') {
  console = { log: log }
}

isNearVector = function(v, absoluteError) {
  if (typeof absoluteError == 'undefined') absoluteError = 1e-6;
  return new gjstest.Matcher(
    'is a vector having elements within ' + absoluteError + ' of [' + (v instanceof Array ? v : v.toArray()) + ']',
    'is not a vector having elements within ' + absoluteError + ' of [' + (v instanceof Array ? v : v.toArray()) + ']',
    function(actual) {

      if (!(actual instanceof THREE.Vector3)) {
        return "which is not a vector";
      }
      var expect = (v instanceof THREE.Vector3 ? v.toArray() : v);
      return (Math.abs(expect[0] - actual.x) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.x))) &&
             (Math.abs(expect[1] - actual.y) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.y))) &&
             (Math.abs(expect[2] - actual.z) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.z)));
    }
  );
}

ElationTest.prototype.isLoaded = function() {
  expectThat(elation, not(isUndefined));
  expectThat(elation.events, not(isUndefined));
  expectThat(elation.physics, not(isUndefined));
  expectThat(elation.physics.system, not(isUndefined));
}

ElationTest.prototype.testRigidBody = function() {
  var body = new elation.physics.rigidbody();
  expectThat(body.position, isNearVector([0, 0, 0]));
  expectThat(body.velocity, isNearVector([0, 0, 0]));
  expectThat(body.acceleration, isNearVector([0, 0, 0]));

}

ElationTest.prototype.testCoordinateSpacesOrientation = function() {
  var body = new elation.physics.rigidbody();
  //expectThat(body.orientation, isNearVector([0, 0, 0, 1]));

  // Check coordinate space transforms
  var forward = new THREE.Vector3(0,0,-1);
  expectThat(body.worldToLocalDir(forward.clone()), isNearVector([0, 0, -1]));
  expectThat(body.localToWorldDir(forward.clone()), isNearVector([0, 0, -1]));

  var tests = [
    // up 90 degrees
    [ [0, Math.PI/2, 0], [1, 0, 0], [-1, 0, 0] ],

    // left 45 degrees
    [ [Math.PI/4, 0, 0], [0, -Math.sin(Math.PI/4), -Math.sin(Math.PI/4)], [0, Math.sin(Math.PI/4), -Math.sin(Math.PI/4)] ],

    // up 37 degrees, left 24, roll 75 
    [ [37 * Math.PI/180, 24 * Math.PI/180, 75 * Math.PI/180], [-0.49723538756370544, -0.46952706575393677, -0.7295898199081421], [-0.4067366421222687, 0.5497853755950928, -0.7295898199081421] ],
  ]

  for (var i = 0; i < tests.length; i++) {
    body.orientation.setFromEuler(new THREE.Euler(0, Math.PI/2, 0));
    body.updateState(); // FIXME - should happen automatically
    expectThat(body.worldToLocalDir(forward.clone()), isNearVector([1, 0, 0]));
    expectThat(body.localToWorldDir(forward.clone()), isNearVector([-1, 0, 0]));
  }
}

ElationTest.prototype.testCoordinateSpacePosition = function() {
  var parent = new elation.physics.rigidbody();
  var self = new elation.physics.rigidbody();
  parent.add(self);

  var point = new THREE.Vector3(0,0,-5);
  var tests = [
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { positionLocal: [-10, -10, -5], positionWorld: [10, 10, -5] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [Math.PI/2, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { positionLocal: [10, 5, 10], positionWorld: [-10, -5, -10] }
    },
/*
    {
      parent: { position: [10, 0, 0], orientation: [0, Math.PI/2, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { positionLocal: [10, 10, 0], positionWorld: [-10, -10, -5] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, Math.PI/2] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { positionLocal: [10, 10, 0], positionWorld: [-10, -10, -5] }
    }
*/
  ];

  for (var i = 0; i < tests.length; i++) {
    var test = tests[i];
    parent.position.fromArray(test.parent.position);
    parent.orientation.setFromEuler(new THREE.Euler().fromArray(test.parent.orientation));
    self.position.fromArray(test.self.position);
    self.orientation.setFromEuler(new THREE.Euler().fromArray(test.self.orientation));

    // FIXME - should happen automatically
    parent.updateState();
    self.updateState();

    expectThat(self.localToWorldPos(point.clone()), isNearVector(test.expect.positionWorld));
    expectThat(self.worldToLocalPos(point.clone()), isNearVector(test.expect.positionLocal));
log(parent.orientation.toArray());
log(parent.position.toArray());
log(parent.positionWorld.toArray());
log('====');
log(self.orientation.toArray());
log(self.position.toArray());
log(self.positionWorld.toArray());
  }
}

ElationTest.prototype.testVelocity = function() {
  var body = new elation.physics.rigidbody();

  expectThat(elation.physics.system.objects.length, equals(0));
  elation.physics.system.add(body);
  expectThat(elation.physics.system.objects, contains(body));

  expectThat(elation.physics.system.active, evalsToFalse);
  elation.physics.system.start();
  expectThat(elation.physics.system.active, evalsToTrue);

  body.velocity.set(1,0,0);
  expectThat(body.velocity, isNearVector([1, 0, 0]));

  for (var i = 0; i < this.numsteps; i++) {
    elation.physics.system.step(this.stepsize);
    expectThat(body.position, isNearVector([this.stepsize * (i+1), 0, 0]));
  }

  elation.physics.system.remove(body);
  expectThat(elation.physics.system.objects.length, equals(0));

  elation.physics.system.stop();
  expectThat(elation.physics.system.active, evalsToFalse);
}
ElationTest.prototype.testAcceleration = function() {
  var body = new elation.physics.rigidbody({mass: 1});

  body.acceleration.set(1,0,0);

  expectThat(elation.physics.system.objects.length, equals(0));
  elation.physics.system.add(body);
  expectThat(elation.physics.system.objects.length, equals(1));
  elation.physics.system.start();

  expectThat(body.position, isNearVector([0, 0, 0]));
  expectThat(body.velocity, isNearVector([0, 0, 0]));
  expectThat(body.acceleration, isNearVector([1,0,0]));

  for (var i = 0; i < this.numsteps; i++) {
    elation.physics.system.step(this.stepsize);
    expectThat(body.state.sleeping, evalsToFalse);

    // velocity = acceleration * time
    expectThat(body.velocity, isNearVector([this.stepsize * (i+1), 0, 0]));
    // position = 1/2 * acceleration * time^2
    expectThat(body.position, isNearVector([.5 * Math.pow(this.stepsize * (i+1), 2), 0, 0]));
  }

  elation.physics.system.remove(body);
  expectThat(elation.physics.system.objects.length, equals(0));

  elation.physics.system.stop();
  expectThat(elation.physics.system.active, evalsToFalse);
}
ElationTest.prototype.testForceStatic = function() {
  var strength = 100;

  var body = new elation.physics.rigidbody({mass: 1});
  body.addForce('static', new THREE.Vector3(strength, 0, 0));
  elation.physics.system.add(body);
  elation.physics.system.start();
  
  for (var i = 0; i < this.numsteps; i++) {
    elation.physics.system.step(this.stepsize);
    expectThat(body.state.sleeping, evalsToFalse);

/*
    // velocity = acceleration * time
    expectThat(body.velocity, isNearVector([this.stepsize * (i+1), 0, 0]));
    // position = 1/2 * acceleration * time^2
    expectThat(body.position, isNearVector([.5 * Math.pow(this.stepsize * (i+1), 2), 0, 0]));
*/
  }

  elation.physics.system.remove(body);
  expectThat(elation.physics.system.objects.length, equals(0));

  elation.physics.system.stop();
  expectThat(elation.physics.system.active, evalsToFalse);
}
