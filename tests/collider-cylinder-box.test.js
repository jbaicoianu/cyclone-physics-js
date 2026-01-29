/**
 * Parameterized cylinder-box collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1005);
const randRange = makeRandRange(rng);

describe('Cylinder-Box collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('cylinder', 'box', 500, randRange));
});
