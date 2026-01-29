/**
 * Parameterized mesh-sphere collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1011);
const randRange = makeRandRange(rng);

describe('Mesh-Sphere collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('mesh', 'sphere', 500, randRange));
});
