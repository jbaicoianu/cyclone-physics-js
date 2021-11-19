elation.require(['physics.processors'], function() {
  elation.extend("physics.processor.cpu", function(parent) {
    elation.physics.processor.base.call(this, parent);
    this.iterateVelocities = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      var framedata = {};
      for (var i = 0; i < objects.length; i++) {
        objects[i].updateAcceleration(framedata);
        var scaledtime = objects[i].getTimescale() * t;
        if (objects[i].state.accelerating || objects[i].state.moving) {
          this.iterateAxisVel(objects[i], 'x', scaledtime);
          this.iterateAxisVel(objects[i], 'y', scaledtime);
          this.iterateAxisVel(objects[i], 'z', scaledtime);
        }
        if (objects[i].state.rotating) {
          this.iterateRotation(objects[i], scaledtime);
        }
        objects[i].updateState(framedata);
      }
    }
    this.iteratePositions = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      var framedata = {};
      for (var i = 0; i < objects.length; i++) {
        var scaledtime = objects[i].getTimescale() * t;
        if (objects[i].state.accelerating || objects[i].state.moving) {
          this.iterateAxisPos(objects[i], 'x', scaledtime);
          this.iterateAxisPos(objects[i], 'y', scaledtime);
          this.iterateAxisPos(objects[i], 'z', scaledtime);
        }
        if (objects[i].state.rotating) {
          //this.iterateRotation(objects[i], scaledtime);
        }
        objects[i].updateState(framedata);
        if (!objects[i].state.sleeping) {
          elation.events.fire({type: "physics_update", element: objects[i], data: t});
        }
      }
    }
    this.iterateAxisVel = function(obj, axis, t) {
      var pos = obj.position[axis],
          vel = obj.velocity[axis],
          accel = obj.acceleration[axis];

      vel += accel * t;
      obj.velocity[axis] = vel * Math.pow(obj.linearDamping, t);
    }
    this.iterateAxisPos = function(obj, axis, t) {
      var pos = obj.position[axis],
          vel = obj.velocity[axis],
          accel = obj.acceleration[axis];

      pos += vel * t;

      obj.position[axis] = pos;
    }
    this.iterateRotation = function(obj, t) {
      this._tmpvec.copy(obj.angularacceleration).multiplyScalar(t);
      obj.angular.add(this._tmpvec).multiplyScalar(Math.pow(obj.angularDamping, t));

      this._tmpvec.copy(obj.angular);
      var theta = this._tmpvec.length();
      if (theta > 0) this._tmpvec.divideScalar(theta);
      this._tmpquat.setFromAxisAngle(this._tmpvec, theta*t);
      obj.orientation.multiply(this._tmpquat);
    }
  }, false, elation.physics.processor.base);

});
