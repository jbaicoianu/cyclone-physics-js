/**
 * Parameterized box-box collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import assert from 'node:assert';
import { THREE, mulberry32, makeRandRange, colliders, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(7777);
const randRange = makeRandRange(rng);

const scenarios = [];
for (let i = 0; i < 500; i++) {
  const mass1 = randRange(0.5, 100);
  const mass2 = randRange(0.5, 100);
  const half1 = new THREE.Vector3(randRange(0.1, 3.0), randRange(0.1, 3.0), randRange(0.1, 3.0));
  const half2 = new THREE.Vector3(randRange(0.1, 3.0), randRange(0.1, 3.0), randRange(0.1, 3.0));
  const speed1 = randRange(1, 50);
  const speed2 = randRange(1, 50);
  const restitution1 = randRange(0, 1);
  const restitution2 = randRange(0, 1);

  const offsetY = i % 3 === 0 ? randRange(-0.5, 0.5) : 0;
  const offsetZ = i % 5 === 0 ? randRange(-0.5, 0.5) : 0;

  const sep = 10;
  scenarios.push({
    label: `scenario ${i + 1}: m=[${mass1.toFixed(1)},${mass2.toFixed(1)}] ` +
      `h1=[${half1.x.toFixed(1)},${half1.y.toFixed(1)},${half1.z.toFixed(1)}] ` +
      `h2=[${half2.x.toFixed(1)},${half2.y.toFixed(1)},${half2.z.toFixed(1)}] ` +
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
      collider: colliders.box(half1),
    },
    body2: {
      mass: mass2,
      position: new THREE.Vector3(sep / 2, offsetY, offsetZ),
      velocity: new THREE.Vector3(-speed2, 0, 0),
      restitution: restitution2,
      collider: colliders.box(half2),
    },
    checkSeparation(body1, body2) {
      const diff = body1.position.clone().sub(body2.position);
      const sepX = Math.abs(diff.x) - (half1.x + half2.x);
      const sepY = Math.abs(diff.y) - (half1.y + half2.y);
      const sepZ = Math.abs(diff.z) - (half1.z + half2.z);
      const maxSep = Math.max(sepX, sepY, sepZ);
      assert.ok(
        maxSep >= -0.5,
        `Bodies deeply interpenetrating: sep=[${sepX.toFixed(3)},${sepY.toFixed(3)},${sepZ.toFixed(3)}]`
      );
    },
  });
}

describe('Box-Box collisions (parameterized)', () => {
  runCollisionScenarios(scenarios);
});
