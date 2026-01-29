/**
 * Dynamic simulation tests for cyclone-physics-js
 * Steps the physics engine forward and verifies results match theoretical predictions.
 */

import { describe, it, before } from 'node:test';
import assert from 'node:assert';
import { elation, THREE } from './setup.js';

const GRAVITY = 9.81;
const DT = 1 / 120; // Small dt for accuracy

/**
 * Create a physics system, add bodies, step for N frames, return results.
 */
function simulate({ bodies, steps, dt = DT }) {
  const system = new elation.physics.system({ autostart: true });

  for (const body of bodies) {
    system.add(body);
  }

  for (let i = 0; i < steps; i++) {
    system.step(dt);
  }

  return { system, bodies };
}

/**
 * Create a rigid body with common defaults.
 */
function createBody(opts = {}) {
  const body = new elation.physics.rigidbody({
    mass: opts.mass ?? 1,
    position: opts.position ?? new THREE.Vector3(0, 0, 0),
    velocity: opts.velocity ?? new THREE.Vector3(0, 0, 0),
    restitution: opts.restitution ?? 0.2,
    linearDamping: 1,   // no artificial damping
    angularDamping: 1,
    sleepEpsilon: opts.sleepEpsilon ?? 0.3,
    sleepDelay: opts.sleepDelay ?? 0.5,
  });

  if (opts.collider) {
    body.setCollider(opts.collider.type, opts.collider.args);
  }

  if (opts.gravity !== false) {
    body.addForce('gravity', new THREE.Vector3(0, -GRAVITY, 0));
  }

  return body;
}

/**
 * Create a static ground plane triangle at y=0.
 */
function createGroundTriangle() {
  const body = new elation.physics.rigidbody({
    mass: 0,
    position: new THREE.Vector3(0, 0, 0),
  });
  body.setCollider('triangle', [
    new THREE.Vector3(-100, 0, -100),
    new THREE.Vector3(100, 0, -100),
    new THREE.Vector3(0, 0, 100),
  ]);
  return body;
}

describe('Dynamic simulation', () => {

  describe('Free fall (gravity only)', () => {
    it('should accelerate downward at g', () => {
      const steps = 120; // 1 second at dt=1/120
      const t = steps * DT;
      const y0 = 100;

      const body = createBody({
        position: new THREE.Vector3(0, y0, 0),
      });

      simulate({ bodies: [body], steps });

      // Euler integration: v = g*t, y = y0 - 0.5*g*t^2
      // With semi-implicit Euler there's some deviation, use 5% tolerance
      const expectedV = -GRAVITY * t;
      const expectedY = y0 - 0.5 * GRAVITY * t * t;

      assert.ok(
        Math.abs(body.velocity.y - expectedV) < Math.abs(expectedV) * 0.05,
        `Velocity should be ~${expectedV.toFixed(2)}, got ${body.velocity.y.toFixed(2)}`
      );
      assert.ok(
        Math.abs(body.position.y - expectedY) < Math.abs(y0 - expectedY) * 0.05,
        `Position should be ~${expectedY.toFixed(2)}, got ${body.position.y.toFixed(2)}`
      );
    });

    it('should have zero horizontal velocity', () => {
      const body = createBody({
        position: new THREE.Vector3(0, 50, 0),
      });

      simulate({ bodies: [body], steps: 60 });

      assert.ok(Math.abs(body.velocity.x) < 1e-10, 'No horizontal velocity');
      assert.ok(Math.abs(body.velocity.z) < 1e-10, 'No lateral velocity');
    });
  });

  describe('Free fall onto a plane (collision + settling)', () => {
    it('should come to rest on the ground plane', () => {
      const boxHalf = 0.5;
      const body = createBody({
        mass: 1,
        position: new THREE.Vector3(0, 3, 0),
        restitution: 0.0,
        collider: {
          type: 'box',
          args: {
            min: new THREE.Vector3(-boxHalf, -boxHalf, -boxHalf),
            max: new THREE.Vector3(boxHalf, boxHalf, boxHalf),
          }
        },
        sleepEpsilon: 0.3,
        sleepDelay: 0.25,
      });

      const ground = createGroundTriangle();

      // Simulate 5 seconds
      simulate({ bodies: [body, ground], steps: 600 });

      // Should be near ground level (y ≈ boxHalf = 0.5)
      assert.ok(
        Math.abs(body.position.y - boxHalf) < 0.5,
        `Should rest near y=${boxHalf}, got y=${body.position.y.toFixed(3)}`
      );

      // Should have near-zero velocity
      const speed = body.velocity.length();
      assert.ok(speed < 1.0, `Should be nearly stopped, speed=${speed.toFixed(3)}`);
    });
  });

  describe('Restitution', () => {
    it('should bounce higher with restitution=1 than restitution=0', () => {
      const boxHalf = 0.5;
      const dropHeight = 5;

      function dropBox(restitution) {
        const body = createBody({
          mass: 1,
          position: new THREE.Vector3(0, dropHeight, 0),
          restitution,
          collider: {
            type: 'box',
            args: {
              min: new THREE.Vector3(-boxHalf, -boxHalf, -boxHalf),
              max: new THREE.Vector3(boxHalf, boxHalf, boxHalf),
            }
          },
          // Don't let it sleep during the test
          sleepEpsilon: 0,
          sleepDelay: 999,
        });

        const ground = createGroundTriangle();
        simulate({ bodies: [body, ground], steps: 240 }); // 2 seconds
        return body;
      }

      const elastic = dropBox(1.0);
      const inelastic = dropBox(0.0);

      // The elastic body should have more kinetic energy (higher speed or position)
      const elasticEnergy = elastic.velocity.lengthSq() + GRAVITY * elastic.position.y;
      const inelasticEnergy = inelastic.velocity.lengthSq() + GRAVITY * inelastic.position.y;

      assert.ok(
        elasticEnergy > inelasticEnergy,
        `Elastic energy (${elasticEnergy.toFixed(2)}) should exceed inelastic (${inelasticEnergy.toFixed(2)})`
      );
    });
  });

  describe('Static body', () => {
    it('should not move when mass=0', () => {
      const body = new elation.physics.rigidbody({
        mass: 0,
        position: new THREE.Vector3(5, 10, 3),
      });

      const system = new elation.physics.system({ autostart: true });
      system.add(body);

      // Even with gravity force, mass=0 body shouldn't integrate
      // (division by mass=0 would be infinity, but the body should be treated as static)
      for (let i = 0; i < 120; i++) {
        system.step(DT);
      }

      assert.ok(
        Math.abs(body.position.x - 5) < 1e-6 &&
        Math.abs(body.position.y - 10) < 1e-6 &&
        Math.abs(body.position.z - 3) < 1e-6,
        `Static body should not move, got (${body.position.x}, ${body.position.y}, ${body.position.z})`
      );
    });
  });

  describe('Conservation of momentum (two-body collision)', () => {
    it('should conserve total momentum', () => {
      const boxHalf = 0.5;
      const colliderArgs = {
        type: 'box',
        args: {
          min: new THREE.Vector3(-boxHalf, -boxHalf, -boxHalf),
          max: new THREE.Vector3(boxHalf, boxHalf, boxHalf),
        }
      };

      const body1 = createBody({
        mass: 1,
        position: new THREE.Vector3(-5, 0, 0),
        velocity: new THREE.Vector3(5, 0, 0),
        gravity: false,
        collider: colliderArgs,
        sleepEpsilon: 0,
        sleepDelay: 999,
      });

      const body2 = createBody({
        mass: 1,
        position: new THREE.Vector3(5, 0, 0),
        velocity: new THREE.Vector3(-5, 0, 0),
        gravity: false,
        collider: colliderArgs,
        sleepEpsilon: 0,
        sleepDelay: 999,
      });

      const m1 = body1.mass, m2 = body2.mass;
      const initialMomentumX = m1 * body1.velocity.x + m2 * body2.velocity.x;

      // Simulate until they collide and separate (2 seconds)
      simulate({ bodies: [body1, body2], steps: 240 });

      const finalMomentumX = m1 * body1.velocity.x + m2 * body2.velocity.x;

      assert.ok(
        Math.abs(finalMomentumX - initialMomentumX) < 1.0,
        `Momentum should be conserved: initial=${initialMomentumX.toFixed(2)}, final=${finalMomentumX.toFixed(2)}`
      );
    });
  });

  describe('Sleep system', () => {
    it('should eventually sleep when at rest with no forces', () => {
      const body = new elation.physics.rigidbody({
        mass: 1,
        position: new THREE.Vector3(0, 0, 0),
        velocity: new THREE.Vector3(0, 0, 0),
        sleepEpsilon: 0.3,
        sleepDelay: 0.5,
      });

      const system = new elation.physics.system({ autostart: true });
      system.add(body);

      // Step enough for sleepDelay to elapse
      for (let i = 0; i < 120; i++) {
        system.step(DT);
      }

      assert.strictEqual(body.state.sleeping, true, 'Body at rest should be sleeping');
    });

    it('should wake up when force is applied', () => {
      const body = new elation.physics.rigidbody({
        mass: 1,
        position: new THREE.Vector3(0, 0, 0),
        sleepEpsilon: 0.3,
        sleepDelay: 0.5,
      });

      const system = new elation.physics.system({ autostart: true });
      system.add(body);

      // Let it sleep
      for (let i = 0; i < 120; i++) {
        system.step(DT);
      }
      assert.strictEqual(body.state.sleeping, true, 'Should be sleeping first');

      // Apply a force - should wake it
      body.addForce('static', {
        force: new THREE.Vector3(10, 0, 0),
      });

      system.step(DT);

      assert.strictEqual(body.state.sleeping, false, 'Should wake up after force applied');
    });
  });
});
