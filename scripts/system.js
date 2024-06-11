import { Vector3, Quaternion } from 'three';
import * as elation from 'elation';
import { CPUPhysicsProcessor } from './processors/cpu.js';

export class PhysicsSystem extends EventTarget {
  static uniqueid = 1;
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
    this.timescale = 1;

    this.init();
  }

  init() {
    if (this.args.autostart !== false) {
      this.start();
    }
  }
  start(args) {
    this.active = true;
    if (!this.processor) {
      //let processortype = (this.processortype in elation.physics.processor ? this.processortype : 'cpu');
      //this.processor = new elation.physics.processor[processortype](this, args);
      this.processor = new CPUPhysicsProcessor(this, args);
    }
  }
  stop() {
    this.active = false;
  }
  step(t) {
    // If there are no objects we have nothing to do
    if (!this.active || this.children.length == 0) return;

    var steps = 1;
    if (this.substep && t > this.substepMaxDelta) {
      steps = Math.min(Math.round(t / this.substepMaxDelta), this.substepMaxSteps);
    }
    
    var step = 1;
    while (t > 0) {
      var steptime = (step < steps ? Math.min(t, this.substepMaxDelta) : t);

      // step 1: update forces for each object, gather array of active objects
      var objects = this.processor.update(this.children, steptime);
      if (objects.length > 0) {

        // step 2: detect contacts
        var collisions = this.processor.collide(steptime);

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
  add(obj) {
    obj.parent = this;
    this.children.push(obj);
    this.dispatchEvent(new CustomEvent('add', { detail: obj }));
  }
  remove(obj) {
    if (obj.parent && obj.parent != this) {
      obj.parent.remove(obj);
      this.dispatchEvent(new CustomEvent('remove', { detail: obj }));
      obj.parent = false;
    } else {
      var i = this.children.indexOf(obj);
      if (i != -1) {
        this.children.splice(i, 1);
        this.dispatchEvent(new CustomEvent('remove', { detail: obj }));
      }
    }
  }
  getObjects(objects, all) {
    if (typeof objects == 'undefined') objects = this.children;
    if (typeof all == 'undefined') all = [];

    for (var i = 0; i < objects.length; i++) {
      all.push(objects[i]);
      if (objects[i].children.length > 0) {
        this.getObjects(objects[i].children, all);
      }
    }
    return all;
  }
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

