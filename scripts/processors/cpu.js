elation.require(['physics.processors'], function() {
  elation.extend("physics.processor.cpu", function(parent) {
    elation.physics.processor.base.call(this, parent);
    this.iterateVelocities = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      for (var i = 0; i < objects.length; i++) {
        objects[i].updateAcceleration();
        let scaledtime = objects[i].getTimescale() * t;
        if (objects[i].state.accelerating || objects[i].state.moving) {
          let obj = objects[i],
              vel = obj.velocity,
              accel = obj.acceleration,
              damping = Math.pow(obj.linearDamping, t);

          vel.x = (vel.x + accel.x * t) * damping;
          vel.y = (vel.y + accel.y * t) * damping;
          vel.z = (vel.z + accel.z * t) * damping;
        }
        if (objects[i].state.rotating) {
          this.iterateRotation(objects[i], scaledtime);
        }
      }
    }
    this.iteratePositions = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      for (var i = 0; i < objects.length; i++) {
        var scaledtime = objects[i].getTimescale() * t;
        if (objects[i].state.accelerating || objects[i].state.moving) {
          let obj = objects[i],
              pos = obj.position,
              vel = obj.velocity;

          pos.x += vel.x * t;
          pos.y += vel.y * t;
          pos.z += vel.z * t;
        }
        if (objects[i].state.rotating) {
          //this.iterateRotation(objects[i], scaledtime);
        }
        objects[i].updateState();
        if (!objects[i].state.sleeping) {
          elation.events.fire({type: "physics_update", element: objects[i], data: t});
        }
      }
    }
    this.iterateRotation = function(obj, t) {
      // TODO - should probably be split out like we did velocity / position above
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
