/**
 * Parameterized capsule-sphere collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1008);
const randRange = makeRandRange(rng);

describe('Capsule-Sphere collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('capsule', 'sphere', 500, randRange));
});
