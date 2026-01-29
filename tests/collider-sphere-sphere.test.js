/**
 * Parameterized sphere-sphere collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import assert from 'node:assert';
import { THREE, mulberry32, makeRandRange, colliders, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(42);
const randRange = makeRandRange(rng);

const scenarios = [];
for (let i = 0; i < 500; i++) {
  const mass1 = randRange(0.5, 100);
  const mass2 = randRange(0.5, 100);
  const radius1 = randRange(0.01, 5.0);
  const radius2 = randRange(0.01, 5.0);
  const speed1 = randRange(1, 200);
  const speed2 = randRange(1, 200);
  const restitution1 = randRange(0, 1);
  const restitution2 = randRange(0, 1);

  const offsetY = i % 3 === 0 ? randRange(-0.5, 0.5) : 0;
  const offsetZ = i % 5 === 0 ? randRange(-0.5, 0.5) : 0;

  const sep = 10;
  scenarios.push({
    label: `scenario ${i + 1}: m=[${mass1.toFixed(1)},${mass2.toFixed(1)}] ` +
      `r=[${radius1.toFixed(2)},${radius2.toFixed(2)}] ` +
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
      collider: colliders.sphere(radius1),
    },
    body2: {
      mass: mass2,
      position: new THREE.Vector3(sep / 2, offsetY, offsetZ),
      velocity: new THREE.Vector3(-speed2, 0, 0),
      restitution: restitution2,
      collider: colliders.sphere(radius2),
    },
    checkSeparation(body1, body2) {
      const dist = body1.position.distanceTo(body2.position);
      const minSep = (radius1 + radius2) * 0.95;
      assert.ok(
        dist >= minSep,
        `Bodies interpenetrating: dist=${dist.toFixed(4)}, minSep=${minSep.toFixed(4)}`
      );
    },
  });
}

describe('Sphere-Sphere collisions (parameterized)', () => {
  runCollisionScenarios(scenarios);
});
