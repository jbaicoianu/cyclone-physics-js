import { PhysicsProcessor } from '../processors.js'

export class CPUPhysicsProcessor extends PhysicsProcessor {
  constructor(parent) {
    super(parent)
  }
  iteratePositions(objects, t) {
    if (t == 0) return; // paused, do nothing
    for (var i = 0; i < objects.length; i++) {
      objects[i].updateAcceleration();
      var scaledtime = objects[i].getTimescale() * t;
      if (objects[i].state.accelerating || objects[i].state.moving) {
        let obj = objects[i],
            pos = obj.position,
            vel = obj.velocity,
            accel = obj.acceleration,
            damping = Math.pow(obj.linearDamping, t);

        pos.x += (t * vel.x) + 1/2 * accel.x * Math.pow(t, 2);
        pos.y += (t * vel.y) + 1/2 * accel.y * Math.pow(t, 2);
        pos.z += (t * vel.z) + 1/2 * accel.z * Math.pow(t, 2);

        vel.x = (vel.x + accel.x * t) * damping;
        vel.y = (vel.y + accel.y * t) * damping;
        vel.z = (vel.z + accel.z * t) * damping;

      }
      if (objects[i].state.rotating) {
        this.iterateRotation(objects[i], scaledtime);
      }
      objects[i].updateState();
      if (!objects[i].state.sleeping) {
        objects[i].dispatchEvent(new CustomEvent('physics_update', { detail: t }));
      }
    }
  }
  iterateRotation(obj, t) {
    // TODO - should probably be split out like we did velocity / position above
    this._tmpvec.copy(obj.angularacceleration).multiplyScalar(t);
    obj.angular.add(this._tmpvec).multiplyScalar(Math.pow(obj.angularDamping, t));

    this._tmpvec.copy(obj.angular);
    var theta = this._tmpvec.length();
    if (theta > 0) this._tmpvec.divideScalar(theta);
    this._tmpquat.setFromAxisAngle(this._tmpvec, theta*t);
    obj.orientation.multiply(this._tmpquat);
  }
}
