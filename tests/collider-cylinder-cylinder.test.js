/**
 * Parameterized cylinder-cylinder collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1001);
const randRange = makeRandRange(rng);

describe('Cylinder-Cylinder collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('cylinder', 'cylinder', 500, randRange));
});
