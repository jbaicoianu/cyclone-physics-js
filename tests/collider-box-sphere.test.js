/**
 * Parameterized box-sphere collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import assert from 'node:assert';
import { THREE, mulberry32, makeRandRange, colliders, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(31337);
const randRange = makeRandRange(rng);

const scenarios = [];
for (let i = 0; i < 500; i++) {
  const mass1 = randRange(0.5, 100);
  const mass2 = randRange(0.5, 100);
  const half = new THREE.Vector3(randRange(0.1, 3.0), randRange(0.1, 3.0), randRange(0.1, 3.0));
  const radius = randRange(0.1, 3.0);
  const speed1 = randRange(1, 100);
  const speed2 = randRange(1, 100);
  const restitution1 = randRange(0, 1);
  const restitution2 = randRange(0, 1);

  const offsetY = i % 3 === 0 ? randRange(-0.5, 0.5) : 0;
  const offsetZ = i % 5 === 0 ? randRange(-0.5, 0.5) : 0;

  // Alternate which body is the box vs sphere
  const boxFirst = i % 2 === 0;

  const sep = 10;
  const collider1 = boxFirst ? colliders.box(half) : colliders.sphere(radius);
  const collider2 = boxFirst ? colliders.sphere(radius) : colliders.box(half);
  const sizeDesc1 = boxFirst
    ? `h=[${half.x.toFixed(1)},${half.y.toFixed(1)},${half.z.toFixed(1)}]`
    : `r=${radius.toFixed(2)}`;
  const sizeDesc2 = boxFirst
    ? `r=${radius.toFixed(2)}`
    : `h=[${half.x.toFixed(1)},${half.y.toFixed(1)},${half.z.toFixed(1)}]`;

  scenarios.push({
    label: `scenario ${i + 1}: m=[${mass1.toFixed(1)},${mass2.toFixed(1)}] ` +
      `${sizeDesc1} ${sizeDesc2} ` +
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
      collider: collider1,
    },
    body2: {
      mass: mass2,
      position: new THREE.Vector3(sep / 2, offsetY, offsetZ),
      velocity: new THREE.Vector3(-speed2, 0, 0),
      restitution: restitution2,
      collider: collider2,
    },
  });
}

describe('Box-Sphere collisions (parameterized)', () => {
  runCollisionScenarios(scenarios);
});
