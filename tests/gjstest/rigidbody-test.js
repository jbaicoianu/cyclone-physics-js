function RigidbodyTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(RigidbodyTest);

RigidbodyTest.prototype.testRigidBody = function() {
  var body = new elation.physics.rigidbody();
  expectThat(body.position, isNearVector([0, 0, 0]));
  expectThat(body.velocity, isNearVector([0, 0, 0]));
  expectThat(body.acceleration, isNearVector([0, 0, 0]));

}

RigidbodyTest.prototype.testCoordinateSpacesOrientation = function() {
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

RigidbodyTest.prototype.testCoordinateSpacePosition = function() {
  var parent = new elation.physics.rigidbody();
  var middle = new elation.physics.rigidbody();
  var self = new elation.physics.rigidbody();
  parent.add(middle);
  middle.add(self);

  var point = new THREE.Vector3(0,0,-5);
  // FIXME - these should come from a data provider
  var tests = [
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { worldToLocal: [-10, -10, -5], localToWorld: [10, 10, -5] }
    },

    // move/rotate parent
    {
      parent: { position: [10, 0, 0], orientation: [Math.PI/2, 0, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { worldToLocal: [-10, -15, 0], localToWorld: [10, 5, 10] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [0, Math.PI/2, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { worldToLocal: [5, -10, -10], localToWorld: [5, 10, 0] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, Math.PI/2] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { worldToLocal: [0, 0, -5], localToWorld: [0, 0, -5] }
    },

    // move/rotate middle
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, 0] },
      middle: { position: [0, 6, -12], orientation: [0, Math.PI/4, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, 0] },
      expect: { worldToLocal: [-12.020815908908844, -16, -2.1213203072547913], localToWorld: [6.464466154575348, 16, -15.535533845424652] }
    },
    {
      parent: { position: [58.2, -19.9, -54.1], orientation: [Math.PI/7, Math.PI, -Math.PI] },
      middle: { position: [-12.5, 18.3, 103.8], orientation: [Math.PI*2/5, -Math.PI*5/9, Math.PI*1/3] },
      self: { position: [108.8, 27.8, -23.4], orientation: [-1, Math.PI*34/56, Math.PI/77] },
      expect: { worldToLocal: [131.98530060052872, -126.06583851575851, -195.00476348400116], localToWorld: [63.21874148398638, 52.69714975357056, -255.2816796898842] }
    },

    // move/rotate self
    {
      parent: { position: [10, 0, 0], orientation: [0, 0, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [Math.PI/2, 0, 0] },
      expect: { worldToLocal: [-10, -5, 10], localToWorld: [10, 15, 0] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [Math.PI/2, 0, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [0, 0, Math.PI/2] },
      expect: { worldToLocal: [-15, 10, 0], localToWorld: [10, 5, 10] }
    },
    {
      parent: { position: [10, 0, 0], orientation: [0, Math.PI/2, 0] },
      middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
      self: { position: [0, 10, 0], orientation: [Math.PI/32, Math.PI/16, -Math.PI/8] },
      expect: { worldToLocal: [10.331193923950195, -7.5534107983112335, -7.823836177587509], localToWorld: [5.11968731880188, 10.480668842792511, 0.9754516184329987] }
    },

  ];

  for (var i = 0; i < tests.length; i++) {
    var test = tests[i];
    parent.position.fromArray(test.parent.position);
    parent.orientation.setFromEuler(new THREE.Euler().fromArray(test.parent.orientation));
    middle.position.fromArray(test.middle.position);
    middle.orientation.setFromEuler(new THREE.Euler().fromArray(test.middle.orientation));
    self.position.fromArray(test.self.position);
    self.orientation.setFromEuler(new THREE.Euler().fromArray(test.self.orientation));

    // FIXME - should happen automatically
    parent.updateState();
    middle.updateState();
    self.updateState();

    expectThat(self.localToWorldPos(point.clone()), isNearVector(test.expect.localToWorld));
    expectThat(self.worldToLocalPos(point.clone()), isNearVector(test.expect.worldToLocal));
  }
}

RigidbodyTest.prototype.testVelocity = function() {
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
RigidbodyTest.prototype.testAcceleration = function() {
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
