/**
 * Parameterized mesh-mesh collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1003);
const randRange = makeRandRange(rng);

describe('Mesh-Mesh collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('mesh', 'mesh', 500, randRange));
});
