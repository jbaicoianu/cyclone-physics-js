import { Vector3, Quaternion, MathUtils } from 'three';

export class Constraint {
  constructor(body, args) {
    this.body = body;
    this.args = args;
  }
  apply(contactlist) {
  }
  update(args) {
  }
}
export class PivotConstraint extends Constraint {
  // TODO - implement pivot constraint
}

// hinge mounts to one body, and pivots another
export class HingeConstraint extends Constraint {
  // TODO - implement hinge constraint
}

// rod links two objects to each other with a fixed length
export class RodConstraint extends Constraint {
  // TODO - implement rod constraint
}

// cable links two objects to each other with a flexible length of string
export class CableConstraint extends Constraint {
  // TODO - implement cable constraint
}

// axis restricts an object's rotation to a single axis, optionally with a min and max angle
export class AxisConstraint extends Constraint {
  constructor(body, args) {
    super(body, args);
    this.axis = args.axis;
    this.min = args.min || false;
    this.max = args.max || false;
    this.enabled = true;
  }
  apply = (function() {
    const ortho = new Vector3(),
          trans = new Vector3(),
          flat = new Vector3(),
          cross = new Vector3(),
          scaledAxis = new Vector3(),
          neworient = new Quaternion();

    return function(contactlist) {
      if (!this.enabled) return false;

      //ortho.set(0,0,-1); // FIXME - figure out orthogonal vector based on this.axis
      ortho.set(0,0,1); // FIXME - figure out orthogonal vector based on this.axis
      trans.copy(ortho).applyQuaternion(this.body.orientation);
      flat.subVectors(trans, scaledAxis.copy(this.axis).multiplyScalar(trans.dot(this.axis))).normalize();
      cross.crossVectors(ortho, flat);
      // FIXME - wow, what a hack
      var sign = (cross.x ? cross.x / Math.abs(cross.x) : 1) * (cross.y ? cross.y / Math.abs(cross.y) : 1) * (cross.z ? cross.z / Math.abs(cross.z) : 1);
      var angle = Math.acos(ortho.dot(flat)) * sign;
      //console.log(angle, sign, this.axis.toArray(), ortho.toArray(), cross.toArray());
      if (this.min && this.max) {
        angle = MathUtils.clamp(angle, this.min, this.max);
      }
      if (angle == 0.0) angle = 0.0001;
      //angle = MathUtils.clamp(angle, min, max);
      //console.log(angle, sign, this.min, this.max);
      
      neworient.setFromAxisAngle(this.axis, angle);
      if (!neworient.equals(this.body.orientation)) {
        this.body.orientation.copy(neworient);
      }
      return false;
    }
  })();
}
// speed constraint restricts the object to a maximum speed
export class SpeedConstraint extends Constraint {
  constructor(body, args) {
    super(body, args);
    this.maxspeed = (typeof args != 'undefined' ? args : Infinity);
    this.enabled = true;
  }
  apply(contactlist) {
    if (!this.enabled) return false;
    var speedSq = this.body.velocity.lengthSq(),
        maxSpeedSq = this.maxspeed * this.maxspeed;
    if (speedSq > maxSpeedSq) {
      this.body.velocity.normalize().multiplyScalar(this.maxspeed);
    }
  }
}
const constraints = {
  'pivot': PivotConstraint,
  'hinge': HingeConstraint,
  'rod': RodConstraint,
  'cable': CableConstraint,
  'axis': AxisConstraint,
  'speed': SpeedConstraint,
}
export { constraints }
