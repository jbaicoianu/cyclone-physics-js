import { Vector3, Quaternion } from 'three';
import * as elation from 'elation';
import { CPUPhysicsProcessor } from './processors/cpu.js';

/**
 * A self-contained physical system
 * @extends EventTarget
 */
export class PhysicsSystem extends EventTarget {
  static uniqueid = 1;
  /**
   * Create a new PhysicsSystem
   * @param {object} args
   * @param {boolean} args.autostart - start simulation immediately
   * @param {boolean} args.substep - Divide each iteration into substeps
   * @param {float} args.substepMaxDelta - maximum length of each iteration substep
   * @param {int} args.substepMaxSteps - maximum number of ubsteps
   * @param {string} args.processortype - which processor to use
   * @param {float} args.timescale - physics system time scale
   */
  constructor(args={}) {
    super();
    this.active = false;
    this.children = [];
    this.processor = false;
    this.args = args || {};
    this.position = this.positionWorld = new Vector3();
    this.orientation = this.orientationWorld = new Quaternion();
    this.scale = this.scaleWorld = new Vector3(1, 1, 1);
    this.substep = elation.any(this.args.substep, true);
    this.substepMaxDelta = elation.any(this.args.substepMaxDelta, 20/1000);
    this.substepMaxSteps = elation.any(this.args.substepMaxSteps, 4);
    this.processortype = elation.any(this.args.processortype, 'cpu');
    this.timescale = elation.any(this.args.timescale, 1);

    if (this.args.autostart !== false) {
      this.start();
    }
  }

  /**
   * Activate simulation and create processor
   * @param {object} args - arguments to pass to the physics processor
   */
  start(args) {
    if (!this.processor) {
      //let processortype = (this.processortype in elation.physics.processor ? this.processortype : 'cpu');
      //this.processor = new elation.physics.processor[processortype](this, args);
      this.processor = new CPUPhysicsProcessor(this, args);
    }
    this.active = true;
  }
  /**
   * Stop simulation
   */
  stop() {
    this.active = false;
  }
  /**
   * Step the simulation forward by t seconds
   * @param {float} t - step time, in seconds
   */
  step(t) {
    // If there are no objects we have nothing to do
    if (!this.active || this.children.length == 0) return;

    let steps = 1;
    if (this.substep && t > this.substepMaxDelta) {
      steps = Math.min(Math.round(t / this.substepMaxDelta), this.substepMaxSteps);
    }
    
    let step = 1;
    while (t > 0) {
      let steptime = (step < steps ? Math.min(t, this.substepMaxDelta) : t);

      // step 1: update forces for each object, gather array of active objects
      let objects = this.processor.update(this.children, steptime);
      if (objects.length > 0) {

        // step 2: detect contacts
        let collisions = this.processor.collide(steptime);

        // step 3: resolve collisions
        if (collisions && collisions.length > 0) {
          this.processor.resolve(steptime, collisions);
        }

        // step 4: update positions for all active objects
        this.processor.iteratePositions(objects, steptime);
      }
      t -= steptime;
      step++;
    }
  }
  /**
   * Add an object to the simulation
   * @param {RigidBody} obj - object to add
   */
  add(obj) {
    obj.parent = this;
    this.children.push(obj);
    this.dispatchEvent(new CustomEvent('add', { detail: obj }));
  }
  /**
   * Remove an object from the simulation
   * @param {RigidBody} obj - object to remove
   */
  remove(obj) {
    if (obj.parent && obj.parent != this) {
      obj.parent.remove(obj);
      this.dispatchEvent(new CustomEvent('remove', { detail: obj }));
      obj.parent = false;
    } else {
      let i = this.children.indexOf(obj);
      if (i != -1) {
        this.children.splice(i, 1);
        this.dispatchEvent(new CustomEvent('remove', { detail: obj }));
      }
    }
  }
  /**
   * Return a flatened list of all objects in this system
   * @param {array} objects - list of objects to recurse (optional)
   * @param {array} all - list of all objects in the scene (optional, used for recursion)
   */
  getObjects(objects, all) {
    if (typeof objects == 'undefined') objects = this.children;
    if (typeof all == 'undefined') all = [];

    for (let i = 0; i < objects.length; i++) {
      all.push(objects[i]);
      if (objects[i].children.length > 0) {
        this.getObjects(objects[i].children, all);
      }
    }
    return all;
  }

  // no-ops, for recursion
  worldToLocalPos(point) {
    return point;
  }
  localToWorldPos(point) {
    return point;
  }
  localToParentPos(point) {
    return point;
  }
  parentToLocalPos(point) {
    return point;
  }
  worldToLocalDir(dir) {
    return dir;
  }
  localToWorldDir(dir) {
    return dir;
  }
  localToWorldScale(scale) {
    return scale;
  }
  worldToLocalScale(scale) {
    return scale;
  }
}

