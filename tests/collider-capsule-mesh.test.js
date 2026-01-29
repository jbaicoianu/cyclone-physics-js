/**
 * Parameterized capsule-mesh collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1010);
const randRange = makeRandRange(rng);

describe('Capsule-Mesh collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('capsule', 'mesh', 500, randRange));
});
