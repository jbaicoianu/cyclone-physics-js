/**
 * Tests for box-triangle collision detection
 */

import { describe, it, before } from 'node:test';
import assert from 'node:assert';
import { elation, THREE } from './setup.js';

describe('box_triangle collision', () => {
  let helperfuncs;

  before(() => {
    helperfuncs = elation.physics.colliders.helperfuncs;
  });

  describe('basic setup', () => {
    it('should have box_triangle function', () => {
      assert.ok(helperfuncs.box_triangle, 'box_triangle should exist');
      assert.strictEqual(typeof helperfuncs.box_triangle, 'function');
    });

    it('should have point_in_triangle function', () => {
      assert.ok(helperfuncs.point_in_triangle, 'point_in_triangle should exist');
      assert.strictEqual(typeof helperfuncs.point_in_triangle, 'function');
    });
  });

  describe('point_in_triangle', () => {
    it('should return true for point inside triangle', () => {
      const a = new THREE.Vector3(0, 0, 0);
      const b = new THREE.Vector3(1, 0, 0);
      const c = new THREE.Vector3(0, 0, 1);
      const point = new THREE.Vector3(0.25, 0, 0.25);

      assert.strictEqual(helperfuncs.point_in_triangle(point, a, b, c), true);
    });

    it('should return false for point outside triangle', () => {
      const a = new THREE.Vector3(0, 0, 0);
      const b = new THREE.Vector3(1, 0, 0);
      const c = new THREE.Vector3(0, 0, 1);
      const point = new THREE.Vector3(2, 0, 2);

      assert.strictEqual(helperfuncs.point_in_triangle(point, a, b, c), false);
    });

    it('should return true for point on edge (within epsilon)', () => {
      const a = new THREE.Vector3(0, 0, 0);
      const b = new THREE.Vector3(1, 0, 0);
      const c = new THREE.Vector3(0, 0, 1);
      const point = new THREE.Vector3(0.5, 0, 0); // On edge AB

      assert.strictEqual(helperfuncs.point_in_triangle(point, a, b, c), true);
    });

    it('should return true for point at vertex', () => {
      const a = new THREE.Vector3(0, 0, 0);
      const b = new THREE.Vector3(1, 0, 0);
      const c = new THREE.Vector3(0, 0, 1);

      assert.strictEqual(helperfuncs.point_in_triangle(a, a, b, c), true);
      assert.strictEqual(helperfuncs.point_in_triangle(b, a, b, c), true);
      assert.strictEqual(helperfuncs.point_in_triangle(c, a, b, c), true);
    });
  });

  describe('box-triangle collision detection', () => {
    function createBox(position, halfsize) {
      const body = new elation.physics.rigidbody({
        position: new THREE.Vector3(position.x, position.y, position.z),
        mass: 1
      });
      body.init();

      const box = new elation.physics.colliders.box(body, {
        min: new THREE.Vector3(-halfsize.x, -halfsize.y, -halfsize.z),
        max: new THREE.Vector3(halfsize.x, halfsize.y, halfsize.z)
      });

      return box;
    }

    function createTriangle(p1, p2, p3) {
      const body = new elation.physics.rigidbody({
        position: new THREE.Vector3(0, 0, 0),
        mass: 0 // Static
      });
      body.init();

      const triangle = new elation.physics.colliders.triangle(body, [
        new THREE.Vector3(p1.x, p1.y, p1.z),
        new THREE.Vector3(p2.x, p2.y, p2.z),
        new THREE.Vector3(p3.x, p3.y, p3.z)
      ]);

      return triangle;
    }

    it('should detect collision when box rests on triangle', () => {
      // Box at y=0.49 with halfsize 0.5, so bottom is at y=-0.01 (slightly penetrating)
      // Triangle is a horizontal plane at y=0
      const box = createBox({ x: 0, y: 0.49, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      const result = helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      // Should have contacts (box touching triangle)
      assert.ok(contacts.length > 0, 'Should detect contacts');
    });

    it('should generate 4 contacts when box face rests flat on triangle', () => {
      // Box slightly penetrating the triangle
      const box = createBox({ x: 0, y: 0.48, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      // Should have 4 contacts (one per bottom vertex)
      assert.strictEqual(contacts.length, 4, 'Should have 4 contacts for flat face');
    });

    it('should generate 1 contact when box corner touches triangle', () => {
      // Rotate box 45 degrees on two axes so corner points down
      const body = new elation.physics.rigidbody({
        position: new THREE.Vector3(0, 0.8, 0),
        mass: 1
      });
      body.init();

      // Rotate to put corner down
      body.orientation.setFromEuler(new THREE.Euler(Math.PI/4, 0, Math.PI/4));
      body.updateState();

      const box = new elation.physics.colliders.box(body, {
        min: new THREE.Vector3(-0.5, -0.5, -0.5),
        max: new THREE.Vector3(0.5, 0.5, 0.5)
      });

      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      // Should have 1 contact (corner)
      assert.ok(contacts.length >= 1 && contacts.length <= 2,
        `Should have 1-2 contacts for corner, got ${contacts.length}`);
    });

    it('should not detect collision when box is above triangle', () => {
      const box = createBox({ x: 0, y: 2, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      const result = helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      assert.strictEqual(contacts.length, 0, 'Should have no contacts when box is above');
    });

    it('should not detect collision when box is outside triangle bounds', () => {
      // Box far to the side, not over the triangle
      const box = createBox({ x: 20, y: 0.5, z: 20 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -1, y: 0, z: -1 },
        { x: 1, y: 0, z: -1 },
        { x: 0, y: 0, z: 1 }
      );

      const contacts = [];
      const result = helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      assert.strictEqual(contacts.length, 0, 'Should have no contacts when box is outside triangle');
    });

    it('should have correct contact normal (pointing from box toward triangle)', () => {
      const box = createBox({ x: 0, y: 0.48, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      assert.ok(contacts.length > 0, 'Should have contacts');

      // Normal should point down (from box toward triangle below)
      const normal = contacts[0].normal;
      assert.ok(normal.y < -0.9, `Normal should point down, got y=${normal.y}`);
    });

    it('should have negative penetration (indicating overlap)', () => {
      const box = createBox({ x: 0, y: 0.48, z: 0 }, { x: 0.5, y: 0.5, z: 0.5 });
      const triangle = createTriangle(
        { x: -10, y: 0, z: -10 },
        { x: 10, y: 0, z: -10 },
        { x: 0, y: 0, z: 10 }
      );

      const contacts = [];
      helperfuncs.box_triangle(box, triangle, contacts, 1/60);

      assert.ok(contacts.length > 0, 'Should have contacts');
      assert.ok(contacts[0].penetration < 0,
        `Penetration should be negative, got ${contacts[0].penetration}`);
    });
  });
});
