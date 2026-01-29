/**
 * Parameterized mesh-box collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1012);
const randRange = makeRandRange(rng);

describe('Mesh-Box collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('mesh', 'box', 500, randRange));
});
