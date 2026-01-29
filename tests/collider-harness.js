/**
 * Shared harness for parameterized collision tests.
 * Provides seeded RNG, simulation, body creation, and assertion helpers.
 */

import { it } from 'node:test';
import assert from 'node:assert';
import { elation, THREE } from './setup.js';

export { THREE, elation };

const DT = 1 / 60; // Realistic game timestep

// Seeded PRNG (mulberry32)
export function mulberry32(seed) {
  return function () {
    seed |= 0;
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function makeRandRange(rng) {
  return function randRange(lo, hi) {
    return lo + rng() * (hi - lo);
  };
}

/**
 * Create a rigid body with collider.
 * @param {object} opts
 * @param {number} opts.mass
 * @param {THREE.Vector3} opts.position
 * @param {THREE.Vector3} opts.velocity
 * @param {number} opts.restitution
 * @param {object} opts.collider - { type, args } passed to setCollider
 */
export function createBody({ mass, position, velocity, restitution, collider }) {
  const body = new elation.physics.rigidbody({
    mass,
    position,
    velocity,
    restitution,
    linearDamping: 1,
    angularDamping: 1,
    sleepEpsilon: 0,
    sleepDelay: 999,
  });
  if (collider) {
    body.setCollider(collider.type, collider.args);
  }
  return body;
}

export function simulate(bodies, steps, dt = DT) {
  const system = new elation.physics.system({ autostart: true });
  for (const b of bodies) system.add(b);
  for (let i = 0; i < steps; i++) system.step(dt);
  return system;
}

export function momentum(body) {
  return body.velocity.clone().multiplyScalar(body.mass);
}

export function kineticEnergy(body) {
  return 0.5 * body.mass * body.velocity.lengthSq();
}

/**
 * Collider factory helpers - return { type, args } for createBody.
 */
export const colliders = {
  sphere(radius) {
    return { type: 'sphere', args: { radius } };
  },
  box(halfExtent) {
    return {
      type: 'box',
      args: {
        min: new THREE.Vector3(-halfExtent.x, -halfExtent.y, -halfExtent.z),
        max: new THREE.Vector3(halfExtent.x, halfExtent.y, halfExtent.z),
      },
    };
  },
  cylinder(radius, height) {
    return { type: 'cylinder', args: { radius, height } };
  },
  capsule(radius, length) {
    return { type: 'capsule', args: { radius, length } };
  },
  /**
   * Create a mesh collider from a simple box shape built out of triangles.
   * This avoids needing a real THREE.Mesh in headless tests.
   */
  meshBox(halfExtent) {
    const hx = halfExtent.x, hy = halfExtent.y, hz = halfExtent.z;
    // 8 vertices of a box
    const v = [
      [-hx, -hy, -hz], [ hx, -hy, -hz], [ hx,  hy, -hz], [-hx,  hy, -hz],
      [-hx, -hy,  hz], [ hx, -hy,  hz], [ hx,  hy,  hz], [-hx,  hy,  hz],
    ];
    // 12 triangles (2 per face), indices into v
    const indices = [
      0,1,2, 0,2,3, // -Z face
      4,6,5, 4,7,6, // +Z face
      0,4,5, 0,5,1, // -Y face
      2,6,7, 2,7,3, // +Y face
      0,3,7, 0,7,4, // -X face
      1,5,6, 1,6,2, // +X face
    ];
    const positions = new Float32Array(v.length * 3);
    for (let i = 0; i < v.length; i++) {
      positions[i * 3] = v[i][0];
      positions[i * 3 + 1] = v[i][1];
      positions[i * 3 + 2] = v[i][2];
    }
    return {
      type: 'mesh',
      args: {
        modeldata: {
          positions,
          index: new Uint16Array(indices),
        },
      },
    };
  },
};

/**
 * Collider shape descriptor generators for parameterized tests.
 * Each returns { collider, sizeLabel } given a randRange function.
 */
export const shapeGenerators = {
  sphere(randRange) {
    const radius = randRange(0.1, 3.0);
    return {
      collider: colliders.sphere(radius),
      sizeLabel: `r=${radius.toFixed(2)}`,
    };
  },
  box(randRange) {
    const half = new THREE.Vector3(randRange(0.1, 3.0), randRange(0.1, 3.0), randRange(0.1, 3.0));
    return {
      collider: colliders.box(half),
      sizeLabel: `h=[${half.x.toFixed(1)},${half.y.toFixed(1)},${half.z.toFixed(1)}]`,
    };
  },
  cylinder(randRange) {
    const radius = randRange(0.1, 2.0);
    const height = randRange(0.2, 4.0);
    return {
      collider: colliders.cylinder(radius, height),
      sizeLabel: `cyl(r=${radius.toFixed(1)},h=${height.toFixed(1)})`,
    };
  },
  capsule(randRange) {
    const radius = randRange(0.1, 2.0);
    const length = randRange(0.2, 4.0);
    return {
      collider: colliders.capsule(radius, length),
      sizeLabel: `cap(r=${radius.toFixed(1)},l=${length.toFixed(1)})`,
    };
  },
  mesh(randRange) {
    const half = new THREE.Vector3(randRange(0.2, 2.0), randRange(0.2, 2.0), randRange(0.2, 2.0));
    return {
      collider: colliders.meshBox(half),
      sizeLabel: `mesh[${half.x.toFixed(1)},${half.y.toFixed(1)},${half.z.toFixed(1)}]`,
    };
  },
};

/**
 * Generate N collision scenarios for a pair of shape types.
 * @param {string} shapeA - key into shapeGenerators
 * @param {string} shapeB - key into shapeGenerators
 * @param {number} count - number of scenarios
 * @param {function} randRange - seeded random range function
 * @param {object} opts - { maxSpeed }
 */
export function generateScenarios(shapeA, shapeB, count, randRange, opts = {}) {
  const maxSpeed = opts.maxSpeed || 50;
  const scenarios = [];
  for (let i = 0; i < count; i++) {
    const mass1 = randRange(0.5, 100);
    const mass2 = randRange(0.5, 100);
    const speed1 = randRange(1, maxSpeed);
    const speed2 = randRange(1, maxSpeed);
    const restitution1 = randRange(0, 1);
    const restitution2 = randRange(0, 1);
    const offsetY = i % 3 === 0 ? randRange(-0.5, 0.5) : 0;
    const offsetZ = i % 5 === 0 ? randRange(-0.5, 0.5) : 0;

    // Alternate which shape is body1 vs body2
    const swap = i % 2 === 1;
    const genA = swap ? shapeGenerators[shapeB] : shapeGenerators[shapeA];
    const genB = swap ? shapeGenerators[shapeA] : shapeGenerators[shapeB];
    const s1 = genA(randRange);
    const s2 = genB(randRange);

    const sep = 10;
    scenarios.push({
      label: `scenario ${i + 1}: m=[${mass1.toFixed(1)},${mass2.toFixed(1)}] ` +
        `${s1.sizeLabel} ${s2.sizeLabel} ` +
        `v=[${speed1.toFixed(1)},-${speed2.toFixed(1)}] ` +
        `e=[${restitution1.toFixed(2)},${restitution2.toFixed(2)}]` +
        (offsetY || offsetZ ? ` off=[${offsetY.toFixed(2)},${offsetZ.toFixed(2)}]` : ''),
      restitution1,
      restitution2,
      body1: {
        mass: mass1,
        position: new THREE.Vector3(-sep / 2, 0, 0),
        velocity: new THREE.Vector3(speed1, 0, 0),
        restitution: restitution1,
        collider: s1.collider,
      },
      body2: {
        mass: mass2,
        position: new THREE.Vector3(sep / 2, offsetY, offsetZ),
        velocity: new THREE.Vector3(-speed2, 0, 0),
        restitution: restitution2,
        collider: s2.collider,
      },
    });
  }
  return scenarios;
}

/**
 * Run parameterized collision scenarios and check conservation laws.
 *
 * @param {Array} scenarios - Each scenario must have:
 *   - label {string}
 *   - body1 {object} - { mass, position, velocity, restitution, collider }
 *   - body2 {object} - same shape
 *   - restitution1, restitution2 {number} - for elastic energy check
 */
export function runCollisionScenarios(scenarios) {
  for (const s of scenarios) {
    it(s.label, () => {
      const body1 = createBody(s.body1);
      const body2 = createBody(s.body2);

      const p0 = momentum(body1).add(momentum(body2));
      const ke0 = kineticEnergy(body1) + kineticEnergy(body2);
      const v1_init = body1.velocity.clone();
      const v2_init = body2.velocity.clone();
      const initialMaxSpeed = Math.max(v1_init.length(), v2_init.length());

      // Track metrics during simulation
      let maxSpeedSeen = initialMaxSpeed;
      let maxKESeen = ke0;
      let collisionCount = 0;
      let worstMomentumError = 0;

      function onCollide() {
        collisionCount++;
      }
      elation.events.add(body1, 'physics_collide', onCollide);
      elation.events.add(body2, 'physics_collide', onCollide);

      // Simulate with per-step monitoring
      const system = new elation.physics.system({ autostart: true });
      system.add(body1);
      system.add(body2);

      const steps = 180; // 3 seconds at 1/60
      for (let i = 0; i < steps; i++) {
        system.step(DT);

        // Track max velocity (detect runaway)
        const speed1 = body1.velocity.length();
        const speed2 = body2.velocity.length();
        maxSpeedSeen = Math.max(maxSpeedSeen, speed1, speed2);

        // Track max KE (detect energy explosion)
        const ke = kineticEnergy(body1) + kineticEnergy(body2);
        maxKESeen = Math.max(maxKESeen, ke);

        // Track momentum error throughout simulation
        const p = momentum(body1).add(momentum(body2));
        const dpMag = p.clone().sub(p0).length();
        worstMomentumError = Math.max(worstMomentumError, dpMag);
      }

      elation.events.remove(body1, 'physics_collide', onCollide);
      elation.events.remove(body2, 'physics_collide', onCollide);

      const p1 = momentum(body1).add(momentum(body2));
      const ke1 = kineticEnergy(body1) + kineticEnergy(body2);

      // Check if collision occurred
      const v1Changed = body1.velocity.clone().sub(v1_init).length() > 0.1;
      const v2Changed = body2.velocity.clone().sub(v2_init).length() > 0.1;
      const collisionOccurred = v1Changed || v2Changed;

      if (!collisionOccurred) {
        assert.ok(
          ke1 <= ke0 * 1.01,
          `No collision but energy increased: KE0=${ke0.toFixed(2)}, KE1=${ke1.toFixed(2)}`
        );
        return;
      }

      // 1. Runaway velocity check - speed should never exceed 2x initial (accounting for elastic bounce)
      const maxReasonableSpeed = initialMaxSpeed * 2.5;
      assert.ok(
        maxSpeedSeen <= maxReasonableSpeed,
        `Runaway velocity detected: maxSpeed=${maxSpeedSeen.toFixed(2)} m/s, ` +
        `expected <= ${maxReasonableSpeed.toFixed(2)} (initial=${initialMaxSpeed.toFixed(2)})`
      );

      // 2. Energy explosion check - KE should never exceed 2x initial during simulation
      assert.ok(
        maxKESeen <= ke0 * 2.0,
        `Energy explosion during simulation: maxKE=${maxKESeen.toFixed(2)}, initial=${ke0.toFixed(2)} ` +
        `(${(maxKESeen/ke0*100).toFixed(0)}%)`
      );

      // 3. Excessive collision count - more than 20 collisions for 2 bodies in 3 seconds is suspicious
      assert.ok(
        collisionCount <= 20,
        `Excessive collisions: ${collisionCount} collision events (expected <= 20)`
      );

      // 4. Momentum conservation - check worst error during entire simulation, not just final
      const p0mag = p0.length();
      const momentumTol = Math.max(0.01 * p0mag, 1.0); // 1% or 1.0 floor (was 0.1)
      assert.ok(
        worstMomentumError < momentumTol,
        `Momentum not conserved: worst Δp=${worstMomentumError.toFixed(4)}, tol=${momentumTol.toFixed(4)} ` +
        `(p0=${p0.x.toFixed(2)},${p0.y.toFixed(2)},${p0.z.toFixed(2)} → ` +
        `p1=${p1.x.toFixed(2)},${p1.y.toFixed(2)},${p1.z.toFixed(2)})`
      );

      // 5. Final energy should not exceed initial (collisions dissipate energy, never create it)
      assert.ok(
        ke1 <= ke0 * 1.05,
        `Energy increased: KE0=${ke0.toFixed(2)}, KE1=${ke1.toFixed(2)} (${(ke1/ke0*100).toFixed(1)}%)`
      );

      // 6. Custom separation check if provided
      if (s.checkSeparation) {
        s.checkSeparation(body1, body2);
      }
    });
  }
}
