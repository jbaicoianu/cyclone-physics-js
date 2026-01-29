/**
 * Parameterized cylinder-mesh collision tests.
 * 500 randomized scenarios verifying conservation laws and collision behavior.
 */

import { describe } from 'node:test';
import { mulberry32, makeRandRange, generateScenarios, runCollisionScenarios } from './collider-harness.js';

const rng = mulberry32(1007);
const randRange = makeRandRange(rng);

describe('Cylinder-Mesh collisions (parameterized)', () => {
  runCollisionScenarios(generateScenarios('cylinder', 'mesh', 500, randRange));
});
