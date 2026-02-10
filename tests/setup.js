/**
 * Test setup for cyclone-physics-js
 * Loads THREE.js, elation framework, and physics modules
 */

import * as THREE from 'three';
import { createRequire } from 'module';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { readFileSync } from 'fs';
import vm from 'vm';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const require = createRequire(import.meta.url);

// Make THREE global
globalThis.THREE = THREE;

// Load elation core
const elationPath = join(__dirname, '../node_modules/elation/components/utils/scripts');
const elationCore = readFileSync(join(elationPath, 'elation.js'), 'utf-8');
const elationEvents = readFileSync(join(elationPath, 'events.js'), 'utf-8');
const elationMath = readFileSync(join(elationPath, 'math.js'), 'utf-8');

// Create a context with globals
const context = {
  THREE,
  console,
  setTimeout,
  clearTimeout,
  setInterval,
  clearInterval,
  Math,
  Array,
  Object,
  String,
  Number,
  Boolean,
  Date,
  JSON,
  Error,
  TypeError,
  RangeError,
  RegExp,
  Promise,
  Map,
  Set,
  WeakMap,
  WeakSet,
  Uint16Array,
  Float32Array,
  Float64Array,
  Int32Array,
  ArrayBuffer,
  DataView,
};

vm.createContext(context);

// Execute elation core
vm.runInContext(elationCore, context);
vm.runInContext(elationEvents, context);
vm.runInContext(elationMath, context);

// Make elation global
globalThis.elation = context.elation;

// Load physics modules
const scriptsPath = join(__dirname, '../scripts');
const physicsFiles = [
  'common.js',
  'cyclone.js',
  'rigidbody.js',
  'forces.js',
  'collisions.js',
  'octree.js',
  'processors.js',
  'processors/cpu.js',
];

for (const file of physicsFiles) {
  const code = readFileSync(join(scriptsPath, file), 'utf-8');
  vm.runInContext(code, context);
}

// Export for tests
export { THREE, context };
export const elation = context.elation;
