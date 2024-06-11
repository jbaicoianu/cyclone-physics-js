import cyclone from '../../../cyclone.js';
import { roundToDigits } from '../test-utils.js';
import { Euler } from 'three';
import * as assert from 'assert';

describe('RigidbodyTest', function() {
  beforeEach(function() {
    this.numsteps = 1000;
    this.stepsize = 0.1;
  });

  it('testRigidBody', function() {
    const body = new cyclone.rigidbody();
    assert.deepStrictEqual(roundToDigits(body.position.toArray(), 3), [0, 0, 0]);
    assert.deepStrictEqual(roundToDigits(body.velocity.toArray(), 3), [0, 0, 0]);
    assert.deepStrictEqual(roundToDigits(body.acceleration.toArray(), 3), [0, 0, 0]);
  });

  it('testCoordinateSpacesOrientation', function() {
    const body = new cyclone.rigidbody();
    const forward = new cyclone.vector3(0, 0, -1);

    assert.deepStrictEqual(roundToDigits(body.worldToLocalDir(forward.clone()).toArray(), 3), [0, 0, -1]);
    assert.deepStrictEqual(roundToDigits(body.localToWorldDir(forward.clone()).toArray(), 3), [0, 0, -1]);

    const tests = [
      [[0, Math.PI/2, 0], [1, 0, 0], [-1, 0, 0]],
      [[Math.PI/4, 0, 0], [0, -Math.sin(Math.PI/4), -Math.sin(Math.PI/4)], [0, Math.sin(Math.PI/4), -Math.sin(Math.PI/4)]],
      [[37 * Math.PI/180, 24 * Math.PI/180, 75 * Math.PI/180], [-0.497, -0.470, -0.730], [-0.407, 0.550, -0.730]]
    ];

    tests.forEach(([euler, worldToLocal, localToWorld]) => {
      body.orientation.setFromEuler(new Euler(...euler));
      body.updateState(); // Ensure this method updates orientation
      assert.deepStrictEqual(roundToDigits(body.localToWorldDir(forward.clone()).toArray(), 3), localToWorld);
      assert.deepStrictEqual(roundToDigits(body.worldToLocalDir(forward.clone()).toArray(), 3), worldToLocal);
    });
  });

  it('testCoordinateSpacePosition', function() {
    const parent = new cyclone.rigidbody();
    const middle = new cyclone.rigidbody();
    const self = new cyclone.rigidbody();
    parent.add(middle);
    middle.add(self);

    const point = new cyclone.vector3(0, 0, -5);
    const tests = [
      {
        parent: { position: [10, 0, 0], orientation: [0, 0, 0] },
        middle: { position: [0, 0, 0], orientation: [0, 0, 0] },
        self: { position: [0, 10, 0], orientation: [0, 0, 0] },
        expect: { worldToLocal: [-10, -10, -5], localToWorld: [10, 10, -5] }
      },
      // Include the transformations with different orientations and positions
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
        expect: { worldToLocal: [10.331, -7.553, -7.824], localToWorld: [5.120, 10.481, 0.975] }
      }
    ];

    tests.forEach((test) => {
      parent.position.fromArray(test.parent.position);
      parent.orientation.setFromEuler(new Euler(...test.parent.orientation));
      middle.position.fromArray(test.middle.position);
      middle.orientation.setFromEuler(new Euler(...test.middle.orientation));
      self.position.fromArray(test.self.position);
      self.orientation.setFromEuler(new Euler(...test.self.orientation));

      parent.updateState();
      middle.updateState();
      self.updateState();

      assert.deepStrictEqual(roundToDigits(self.localToWorldPos(point.clone()).toArray(), 3), test.expect.localToWorld);
      assert.deepStrictEqual(roundToDigits(self.worldToLocalPos(point.clone()).toArray(), 3), test.expect.worldToLocal);
    });
  });

  it('testVelocity', function() {
    const system = new cyclone.system();
    const body = new cyclone.rigidbody();

    assert.strictEqual(system.objects.length, 0);
    system.add(body);
    assert.strictEqual(system.objects.includes(body), true);
    assert.strictEqual(system.active, true);

    body.velocity.set(1, 0, 0);
    assert.deepStrictEqual(roundToDigits(body.velocity.toArray(), 3), [1, 0, 0]);

    for (let i = 0; i < this.numsteps; i++) {
      system.step(this.stepsize);
      assert.deepStrictEqual(roundToDigits(body.position.toArray(), 3), [this.stepsize * (i + 1), 0, 0]);
    }

    system.remove(body);
    assert.strictEqual(system.objects.length, 0);
    system.stop();
    assert.strictEqual(system.active, false);
  });

  it('testAcceleration', function() {
    const system = new cyclone.system();
    const body = new cyclone.rigidbody({ mass: 1 });
    body.acceleration.set(1, 0, 0);

    assert.strictEqual(system.objects.length, 0);
    system.add(body);
    assert.strictEqual(system.objects.length, 1);
    system.start();

    assert.deepStrictEqual(roundToDigits(body.position.toArray(), 3), [0, 0, 0]);
    assert.deepStrictEqual(roundToDigits(body.velocity.toArray(), 3), [0, 0, 0]);
    assert.deepStrictEqual(roundToDigits(body.acceleration.toArray(), 3), [1, 0, 0]);

    for (let i = 0; i < this.numsteps; i++) {
      system.step(this.stepsize);
      assert.strictEqual(body.state.sleeping, false);

      assert.deepStrictEqual(roundToDigits(body.velocity.toArray(), 3), [this.stepsize * (i + 1), 0, 0]);
      assert.deepStrictEqual(roundToDigits(body.position.toArray(), 3), [0.5 * Math.pow(this.stepsize * (i + 1), 2), 0, 0]);
    }

    system.remove(body);
    assert.strictEqual(system.objects.length, 0);
    system.stop();
    assert.strictEqual(system.active, false);
  });
});

