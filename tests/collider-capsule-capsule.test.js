/**
 * Parameterized capsule-capsule collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1002);
const randRange = makeRandRange(rng);

describe('Capsule-Capsule collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('capsule', 'capsule', 500, randRange));
});
