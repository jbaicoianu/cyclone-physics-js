/**
 * Parameterized capsule-box collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1009);
const randRange = makeRandRange(rng);

describe('Capsule-Box collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('capsule', 'box', 500, randRange));
});
