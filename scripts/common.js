import { Vector3, Quaternion } from 'three';

export class CycloneVector3 extends Vector3 {
  get x() { return this._x; }
  set x(v) { this.changed = (this.changed || v !== this._x); this._x = v; }

  get y() { return this._y; }
  set y(v) { this.changed = (this.changed || v !== this._y); this._y = v; }

  get z() { return this._z; }
  set z(v) { this.changed = (this.changed || v !== this._z); this._z = v; }

  reset() {
    this.changed = false;
  }
  toJSON() {
    return {x: this._x, y: this._y, z: this._z};
  }
}
// FIXME - orientation doesn't currently work properly because THREE.Quaternion already uses getters and setters internally
export class CycloneQuaternion extends Quaternion {
  get x() { return this._x; }
  set x(v) { this.changed = (this.changed || v !== this._x); this._x = v; }

  get y() { return this._y; }
  set y(v) { this.changed = (this.changed || v !== this._y); this._y = v; }

  get z() { return this._z; }
  set z(v) { this.changed = (this.changed || v !== this._z); this._z = v; }

  get w() { return this._w; }
  set w(v) { this.changed = (this.changed || v !== this._w); this._w = v; }

  copy(quat) {
    this.changed = true;
    return super.copy(quat);
  }
  set(x, y, z, w) {
    this.changed = true;
    return super.set(x, y, z, w);
  }
  setFromEuler(euler, update) {
    this.changed = true;
    return super.setFromEuler(euler, update);
  }
  setFromAxisAngle(axis, angle) {
    this.changed = true;
    return super.setFromAxisAngle(axis, angle);
  }
  setFromRotationMatrix(m) {
    this.changed = true;
    return super.setFromRotationMatrix(m);
  }
  setFromUnitVectors(v1, v2) {
    this.changed = true;
    return super.setFromUnitVectors(v1, v2);
  }
  reset() {
    this.changed = false;
  }
  toJSON() {
    return {x: this._x, y: this._y, z: this._z, w: this.w};
  }
}
