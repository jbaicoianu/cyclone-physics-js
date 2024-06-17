import { Vector3, Matrix4 } from 'three';
import { CycloneVector3, CycloneQuaternion } from './common.js';
import * as elation from 'elation';

/**
 * forces
 * @module Forces
 */
/**
 * Gravity force generator
 */
class GravityForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'gravity';
    this.others = [];
    this.accel = (args.isVector3 ? args : new Vector3());
    this.timescale = args.timescale || 1;
    this.gravsum = new Vector3();
    let _tmpvec = new Vector3();
    this.update(args);
  }
  apply() {
    if (this.others.length > 0) {
      this.gravsum.copy(this.getForceAtPoint(this.body.position, this.body.mass));
    } else {
      this.gravsum.copy(this.accel).multiplyScalar(this.body.mass);
    }
    //console.log("Gravity force: " + [this.gravsum.x, this.gravsum.y, this.gravsum.z] + " m/s^2", this.accel);
    //return [this.gravsum, false];
    this.body.applyForce(this.gravsum);
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    if (updateargs.isVector3) {
      //this.gravsum.copy(updateargs).multiplyScalar(this.body.mass);
      this.accel.copy(updateargs);
    } else {
      if (typeof updateargs.timescale != 'undefined') {
        this.timescale = updateargs.timescale;
      }
      if (typeof updateargs.others != 'undefined') {
        this.others = updateargs.others;
      }
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  getOrbitalVelocity(point) {
    _tmpvec.copy(point).normalize().cross(new Vector3(0,1,0)).normalize();
//foo.multiplyScalar(2 * Math.PI * this.position.length());
    var m = this.others[0].mass;
    _tmpvec.multiplyScalar(Math.sqrt((m * m * 6.67384e-11) / ((m + this.body.mass) * point.length())));
    return _tmpvec;
  }
  getForceAtPoint = (function() {
    var _gravsum = new Vector3();
    return function(point, mass) {
      if (typeof mass == 'undefined') {
        mass = this.body.mass;
      }
      if (this.others.length > 0) {
        _gravsum.set(0,0,0);
        for (var i = 0; i < this.others.length; i++) {
          // Calculate gravity force for all objects in the set, excluding ourselves
          if (this.others[i] && this.others[i] !== this.body) {
            _tmpvec.subVectors(this.others[i].position, point);
            var rsq = _tmpvec.lengthSq();
            var r = Math.sqrt(rsq);
            var a = (6.67384e-11 * mass * this.others[i].mass) / rsq;
            _gravsum.x += a * _tmpvec.x / r;
            _gravsum.y += a * _tmpvec.y / r;
            _gravsum.z += a * _tmpvec.z / r;
          }
        }
      } else {
        _gravsum.copy(this.accel);
      }
      return _gravsum;
    }
  })();
  sleepstate() {
    return (this.gravsum.lengthSq() <= 1e-6);
  }
  toJSON() {
    return {
      type: this.type,
      others: false, // FIXME - should serialize other objects based on their body ids
      accel: {x: this.accel.x, y: this.accel.y, z: this.accel.z},
      timescale: this.timescale,
    };
  }
}

/**
 * Static force generator
 */
class StaticForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'static';
    this.force = args.force || (args.isVector3 ? args : new Vector3());
    this.point = args.point || false;
    this.absolute = elation.any(args.absolute, false);
    this.relative = !this.absolute;

    if (!(this.force.isVector3)) this.force = new Vector3().copy(this.force);
    if (this.point && !(this.point.isVector3)) this.point = new Vector3().copy(this.point);

    this._tmpvec = new Vector3();
    this._tmpvec2 = new Vector3();
  }
  apply() {
    this._tmpvec.copy(this.force);
    if (this.point) {
      this._tmpvec2.copy(this.point);
      //var torque = this.point.clone().crossSelf(this.accel);
      //body.momentInverse.multiplyVector3(torque);
      //return [this._tmpvec.clone(), torque];
      this.body.applyForceAtPoint(this._tmpvec.clone(), this._tmpvec2.clone(), !this.absolute);
    } else {
      this.body.applyForce(this._tmpvec, !this.absolute);
    }
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    let changed = false;
    if (updateargs.isVector3) {
      if (!this.force.equals(updateargs)) {
        this.force.copy(updateargs);
        changed = true;
      }
    } else {
      if (!this.force.equals(updateargs.force)) {
        this.force.copy(updateargs.force);
        changed = true;
      }
      if (updateargs.point.isVector3 && !this.point.equals(updateargs.point)) {
        this.point = updateargs.point;
        changed = true;
      }
      this.relative = updateargs.relative;
    }
    //if (changed) {
      this.dispatchEvent(new CustomEvent('physics_force_update'));
    //}
  }
  sleepstate() {
    return (this.force.lengthSq() <= 1e-6);
  }
  toJSON() {
    return {
      type: this.type,
      force: {x: this.force.x, y: this.force.y, z: this.force.z},
      point: (this.point ? {x: this.point.x, y: this.point.y, z: this.point.z} : false),
      absolute: this.absolute,
      relative: this.relative,
    };
  }
}

/**
 * Friction force generator
 */
class FrictionForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'friction';
    this.friction = args.friction || args;

    let _force = new Vector3();

  }
  apply() {
    _force.set(0,0,0);
    if (this.friction > 0) {
      var vsq = this.body.velocity.lengthSq();
      _force.copy(this.body.velocity).multiplyScalar(-1 * this.friction * this.body.mass);
    }
    this.body.applyForce(_force);
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    let friction = elation.any(updateargs.friction, updateargs);
    if (friction != this.friction) {
      this.friction = friction;
      this.dispatchEvent(new CustomEvent('physics_force_update'));
    }
  }
  computeVelocityForDistance(dist, friction, mass) {
    var a = friction;
    //var v0 = (dist - .5*a*time^2) / time;


    var v0 = Math.sqrt((a * dist) / (.5 * mass));

    return v0;
  }
  sleepstate() {
    return !(this.friction > 1e-6 && this.body.velocity.lengthSq() > 1e-6);
  }
  toJSON() {
    return {
      type: this.type,
      friction: this.friction,
    };
  }
}

/**
 * Anisotropic friction force generator
 */
class AnisotropicFrictionForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'anisotropicfriction';
    this.friction = args;

    let _force = new Vector3();
  }
  apply() {
    _force.set(0,0,0);
    if (this.friction) {
      var relvel = body.worldToLocalDir(body.velocity.clone());
      _force.x = relvel.x * -1 * this.friction.x * body.mass;
      _force.y = relvel.y * -1 * this.friction.y * body.mass;
      _force.z = relvel.z * -1 * this.friction.z * body.mass;
      body.localToWorldDir(_force);
    }
    body.applyForce(_force);
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    this.friction = updateargs;
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  sleepstate() {
    return !(this.friction.lengthSq() > 1e-6 && body.velocity.lengthSq() > 1e-6);
  }
  toJSON() {
    return {
      type: this.type,
      friction: this.friction,
    };
  }
}

/**
 * Drag force generator
 */
class DragForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'drag';
    this.drag = args;

    let _force = new Vector3();
    this.sleeping = false;
  }
  apply() {
    _force.set(0,0,0);
    if (this.drag > 0) {
      var v = body.velocity.length();
      _force.copy(body.velocity).multiplyScalar(-.5*this.drag*v);
    }
    body.applyForce(_force);
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    this.drag = updateargs;
    console.log('set drag to ', this.drag);
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      drag: this.drag,
    };
  }
}

/**
 * Aerodynamic force generator
 */
class AeroForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'aero';
    if (!args) args = {};
    this.body = body;
    this.tensor = args.tensor || new Matrix4();
    this.position = args.position || new Vector3();
    this.sleeping = false;

    let _tmpvec = new Vector3();
  }
  apply() {
    var force = this.getForceFromTensor(this.getTensor());
    this.body.applyForceAtPoint(force, this.position, true);
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  getForceFromTensor(tensor) {
    this.body.worldToLocalDir(_tmpvec.copy(this.body.velocity)).negate();
    //var before = _tmpvec.clone();
    this.getTensor().multiplyVector3(_tmpvec).multiplyScalar(.5);
    //console.log('parent: ' + VECDUMP(this.body.velocity) + 'before: ' + VECDUMP(before) + ', after: ' + VECDUMP(_tmpvec));
    return _tmpvec;
  }
  getTensor() {
    return this.tensor;
  }
  updateTensor(tensor) {
    this.tensor.copy(tensor);
  }
  update(updateargs) {
    if (updateargs.tensor) {
      this.updateTensor(updateargs.tensor);
    }
    if (args.position) {
      if (args.position.isVector3) {
        this.position.copy(args.position);
      } else {
        this.position.set(args.position[0], args.position[1], args.position[2]);
      }
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      drag: this.drag,
      tensor: this.tensor,
      position: {x: this.position.x, y: this.position.y, z: this.position.z},
    };
  }
}

/**
 * Aerodynamic controller force generator
 */
class AeroControlForce extends AeroForce {
  constructor(body, args) {
    super(body, args);
    this.type = 'aerocontrol';
    this.body = body;
    this.tensor = args.tensor || new Matrix4();
    this.tensor_min = args.tensor_min || new Matrix4();
    this.tensor_max = args.tensor_max || new Matrix4();
    this.position = args.position || new Vector3();
    this.control = 0;
    this.sleeping = false;

    let _tmpmat = new Matrix4();
  }
  getTensor() {
    if (this.control <= -1) {
      return this.tensor_min;
    } else if (this.control >= 1) {
      return this.tensor_max;
    } else if (this.control < 0) {
      return this.interpolate(this.tensor_min, this.tensor, this.control+1);
    } else if (this.control > 0) {
      return this.interpolate(this.tensor, this.tensor_max, this.control);
    }
    return this.tensor;
  }
  interpolate(a, b, prop) {
    _tmpmat.set(
      a.n11 * (1 - prop) + b.n11 * prop, a.n12 * (1 - prop) + b.n12 * prop, a.n13 * (1 - prop) + b.n13 * prop, 0,
      a.n21 * (1 - prop) + b.n21 * prop, a.n22 * (1 - prop) + b.n22 * prop, a.n23 * (1 - prop) + b.n23 * prop, 0,
      a.n31 * (1 - prop) + b.n31 * prop, a.n32 * (1 - prop) + b.n32 * prop, a.n33 * (1 - prop) + b.n33 * prop, 0,
      0, 0, 0, 1
    );
    return _tmpmat;
  }
  setControl(c) {
    this.control = c;
  }
  toJSON() {
    return {
      type: this.type,
      drag: this.drag,
      tensor: this.tensor,
      position: {x: this.position.x, y: this.position.y, z: this.position.z},
    };
  }
}

/**
 * Buoyancy force generator
 */
class BuoyancyForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'buoyancy';
    this.density = args.density || 1000; // water = 1000 kg/m^3
    this.volume = args.volume || 1;
    this.maxdepth = args.maxdepth || 10;
    this.waterheight = args.waterheight || 0;
    this.position = args.position || new Vector3(0,0,0);
    this.submerged = 0;
    this.force = new Vector3();
    this.positionworld = new Vector3();
    this.sleeping = false;
  }
  apply() {
    var point = this.body.localToWorldPos(this.position.clone());
    var depth = point.y - this.maxdepth / 2;
    if (depth >= this.waterheight) {
      this.submerged = 0;
      this.force.y = 0;
    } else {
      if (depth <= this.waterheight - this.maxdepth) {
        this.force.y = this.density * this.volume;
        this.submerged = 1;
      } else {
        //ratio = depth / (this.waterheight - this.maxdepth);
        //this.submerged = depth / (this.waterheight - this.maxdepth) / 2 + this.maxdepth;
        //this.submerged = -(depth - this.maxdepth - this.waterheight) / 2 * this.maxdepth;
        this.submerged = -(depth / (this.maxdepth));
        //force.y = this.density * this.volume * -1 / ((depth - this.maxdepth - this.waterheight) / 2 * this.maxdepth);
        this.force.y = this.density * this.volume * this.submerged;
      }
      //this.force.y *= 0.0098;
      //force.y = this.density * this.volume * ratio * .0098;
      //this.body.applyForceAtPoint(this.force, this.position, false);
      this.positionworld.copy(this.position);
      this.body.applyForceAtPoint(this.force, this.body.localToWorldDir(this.positionworld), false);
    }
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update() {
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      density: this.density,
      volume: this.volume,
      maxdepth: this.maxdepth,
      waterheight: this.waterheight,
      position: {x: this.position.x, y: this.position.y, z: this.position.z},
    };
  }
}

/**
 * Spring force generator
 */
class SpringForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'spring';
    this.connectionpoint = args.connectionpoint || new Vector3(0,0,0);
    this.otherconnectionpoint = args.otherconnectionpoint || new Vector3(0,0,0);
    this.other = args.other || false;
    this.anchor = args.anchor || false;
    this.strength = elation.any(args.strength, 1);
    this.midpoint = args.midpoint || false;
    this.restlength = elation.any(args.restlength, 0);
    this.bungee = args.bungee || false;
    this.hard = args.hard || false;
    this.force = new Vector3();

    var _tmpvec1 = new Vector3();
    var _tmpvec2 = new Vector3();

  }
  apply() {
    var lws = this.body.localToWorldPos(_tmpvec1.copy(this.connectionpoint));
    var ows = (this.other ? this.other.localToWorldPos(_tmpvec2.copy(this.otherconnectionpoint)) : this.anchor);

    this.body.worldToLocalDir(this.force.subVectors(lws, ows));
    if (this.midpoint) {
      this.force.divideScalar(2);
    }
    //var magnitude = Math.abs(this.force.length() - this.restlength) * this.strength;
    var magnitude = this.force.length() + 1e-5;
    if (this.bungee && magnitude <= this.restlength) {
      this.force.set(0,0,0);
    } else if (this.hard && magnitude <= this.restlength) {
      this.force.set(0,0,0);
    } else {
      this.force.divideScalar(magnitude);
      magnitude = this.strength * (magnitude - this.restlength);
      this.force.multiplyScalar(-magnitude);
      this.force = this.body.worldToLocalDir(this.force);
      
      this.body.applyForceAtPoint(this.force, this.connectionpoint, true);
      if (this.other && this.other.mass) {
        this.other.applyForceAtPoint(this.force.multiplyScalar(-1), this.otherconnectionpoint, true);
      }
    }
    this.dispatchEvent(new CustomEvent('physics_force_apply'));
  }
  update(updateargs) {
    for (var k in updateargs) {
      this[k] = updateargs[k];
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  sleepstate() {
    var lws = this.body.localToWorldPos(_tmpvec1.copy(this.connectionpoint));
    var ows = (this.other ? this.other.localToWorldPos(_tmpvec2.copy(this.otherconnectionpoint)) : this.anchor);
    return (lws.distanceToSquared(ows) <= 1e6);
  }
  toJSON() {
    return {
      type: this.type,
      connectionpoint: this.connectionpoint,
      otherconnectionpoint: this.otherconnectionpoint,
      other: this.other,
      anchor: this.anchor,
      strength: this.strength,
      midpoint: this.midpoint,
      restlength: this.restlength,
      bungee: this.bungee,
      hard: this.hard,
    };
  }
}

/**
 * Magnet force generator
 */
class MagnetForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'magnet';
    this.force = new Vector3();
    this.anchor = args.anchor;
    this.strength = args.strength || 1;
    this.sleeping = false;
  }
  apply = (function() {
    var tmpvec = new Vector3();
    return function() {
      var mu = 1.256636e-6;
      var qm1 = 1,
          qm2 = this.strength;
      this.force.subVectors(body.position, this.anchor);
      var rsq = this.force.lengthSq();

      var F = (mu * qm1 * qm2) / (4 * Math.PI * rsq);
      this.force.normalize().multiplyScalar(F);
//console.log(this.force.toArray().toString());

      body.applyForce(this.force);
      this.dispatchEvent(new CustomEvent('physics_force_apply'));
    }
  })();
  update(updateargs) {
    if (updateargs.anchor) {
      this.anchor = updateargs.anchor;
    }
    if (updateargs.strength) {
      this.strength = updateargs.strength;
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      anchor: this.anchor,
      strength: this.strength,
    };
  }
}

/**
 * Repel force generator
 */
class RepelForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'repel';
    this.force = new Vector3();
    this.anchor = args.anchor;
    this.strength = args.strength || 1;
    this.sleeping = false;
  }
  apply = (function() {
    var tmpvec = new Vector3();
    return function() {
      this.force.subVectors(body.position, this.anchor);
      var rsq = this.force.lengthSq();

      this.force.normalize().multiplyScalar(this.strength);
//console.log(this.force.toArray().toString());

      body.applyForce(this.force);
      this.dispatchEvent(new CustomEvent('physics_force_apply'));
    }
  })();
  update(updateargs) {
    if (updateargs.anchor) {
      this.anchor = updateargs.anchor;
    }
    if (updateargs.strength) {
      this.strength = updateargs.strength;
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      anchor: this.anchor,
      strength: this.strength,
    };
  }
}

/**
 * Electrostatic force generator
 */
class ElectrostaticForce extends EventTarget {
  constructor(body, args) {
    super();
    this.body = body;
    this.type = 'electrostatic';
    this.force = new Vector3();
    this.charge = elation.any(args.charge, 1);
    this.maxdist = elation.any(args.maxdist, Infinity);
    this.others = args.others || false;
    this.sleeping = false;
  }
  apply = (function() {
    var tmpvec = new Vector3();
    // TODO - the constant Ke is actually dependent on the electric permittivity of the material the charges are immersed in, but this is good enough for most cases
    var Ke = 8.9875517873681764e9; 
    return function(framedata) {
      var nearby = this.others;// || body.parent.children;
      if (!nearby) return;

      if (typeof framedata['electrostaticID'] == 'undefined') {
        framedata['electrostaticID'] = 0;
        framedata['electrostaticMatrix'] = {};
        framedata['electrostaticSeen'] = {};
      }
      var thisID = framedata['electrostaticID'];
      var matrix = framedata['electrostaticMatrix'];
      var seen = framedata['electrostaticSeen'];

      this.force.set(0,0,0);
      var mycharge = Ke * this.charge;
      for (var i = 0; i < nearby.length; i++) {
        var other = nearby[i];
        if (other === body) continue;
        if (other.position.distanceToSquared(body.position) > this.maxdist * this.maxdist) {
          continue;
        }
        var pairID = Math.min(thisID, i) + '_' + Math.max(thisID, i);
        if (matrix[pairID]) {
          console.log('found an existing solution', pairID, matrix[pairID]);
            //matrix[pairID] = tmpvec.toArray();
          this.force.add(tmpvec.fromArray(matrix[pairID]).multiplyScalar(1));
        } else {
          var forces = other.getForces('electrostatic');
          if (forces) {
            tmpvec.subVectors(body.position, other.position);
            var r = tmpvec.length();
            var rsq = Math.pow(r + 1e6, 2);
            //var rsq = tmpvec.lengthSq() + 1e6;
//console.log(rsq, body.position.toArray(), other.position.toArray());

            //this.force.add(tmpvec.normalize().multiplyScalar(mycharge * forces[0].charge / rsq));
            //this.force.add(tmpvec.normalize().multiplyScalar(Ke * this.charge * forces[0].charge / rsq));
            this.force.add(tmpvec.multiplyScalar((mycharge * forces[0].charge) / (rsq * r)));

            //matrix[pairID] = tmpvec.toArray();
          }
        }
//console.log(this.force.toArray().toString());
      }
      framedata['electrostaticID']++;
      body.applyForce(this.force);
      this.dispatchEvent(new CustomEvent('physics_force_apply'));
    }
  })();
  update(updateargs) {
    for (var k in updateargs) {
      this[k] = updateargs[k];
    }
    this.dispatchEvent(new CustomEvent('physics_force_update'));
  }
  toJSON() {
    return {
      type: this.type,
      charge: this.charge,
      maxdist: this.maxdist,
    };
  }
}

const forces = {
  'static': StaticForce,
  'gravity': GravityForce,
  'friction': FrictionForce,
  'anisotropicfriction': AnisotropicFrictionForce,
  'gravity': GravityForce,
  'drag': DragForce,
  'buoyancy': BuoyancyForce,
  'spring': SpringForce,
  'magnet': MagnetForce,
  'repel': RepelForce,
  'electrostatic': ElectrostaticForce,
}
export { forces }
