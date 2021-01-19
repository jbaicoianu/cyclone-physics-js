elation.require(['physics.common', 'physics.cyclone', 'physics.processors.cpu', 'janusweb.external.physx.physx-cyclone-processor'], function() {

  // `elation.physics.worker` is the code that runs in a worker when using multithreaded physics
  // It interfaces with an `elation.physics.processor.worker` processor in the main thread, and keeps
  // the worker's scene in sync with the main thread's scene.

  elation.extend("physics.worker", function(parent) {
    this.fps = 120;
    this.objects = {};
    this.system = new elation.physics.system({autostart: false, processortype: 'physx'});
    this.system.start();
    this.lastframe = performance.now();
    setInterval(() => { 
      this.advance();
    }, 1000 / this.fps);

    this.handleMessage = function(msg) {
      if (msg.type == 'object_changes') {
        let framedata = {};
        for (let i = 0; i < msg.changes.length; i++) {
          let obj = msg.changes[i];
          if (!this.objects[obj.id]) {
            this.objects[obj.id] = new elation.physics.rigidbody({
              id: obj.id,
              position: new elation.physics.vector3().copy(obj.position),
              orientation: new elation.physics.quaternion().copy(obj.orientation),
              velocity: new elation.physics.vector3().copy(obj.velocity),
              acceleration: new elation.physics.vector3().copy(obj.acceleration),
              angular: new elation.physics.vector3().copy(obj.angular),
              scale: new elation.physics.vector3().copy(obj.scale),
              mass: obj.mass,
              restitution: obj.restitution,
              dynamicfriction: obj.dynamicfriction,
              staticfriction: obj.staticfriction,
              //forces: object.force_accumulator,
              lineardamping: obj.lineardamping,
              angulardamping: obj.angulardamping,
            });
//console.log('new rigidbody', obj.id, obj.collider, obj, this.objects[obj.id], this.objects, msg.changes);
            if (obj.collider) {
              this.objects[obj.id].collider = new elation.physics.colliders[obj.collider.type](this.objects[obj.id], obj.collider);
              this.objects[obj.id].collider.getInertialMoment();
              this.objects[obj.id].collider.trigger = obj.collider.trigger;
              elation.events.add(this.objects[obj.id], 'physics_collide', (ev) => {
                let contact = ev.data,
                    body1 = contact.bodies[0],
                    body2 = contact.bodies[1];
                postMessage({type: 'contact_begin', thing1: body1.id, thing2: body2.id, point: contact.point, normal: contact.normal, penetration: contact.penetration});
              });
            }
            if (obj.forces) {
//console.log('cool forces bro', obj.forces, this.objects[obj.id]);
              obj.forces.forEach(f => {
                this.objects[obj.id].addForce(f.type, f);
              });
            }
            if (obj.parentid) {
              if (this.objects[obj.parentid]) {
                this.objects[obj.parentid].add(this.objects[obj.id]);
              } else {
                console.log('Couldn\'t find parent for object', obj.id, obj.parentid, this.objects[obj.id]);
              }
            } else {
              this.system.add(this.objects[obj.id]);
            }
          } else {
            let body = this.objects[obj.id];
//console.log('copy it', obj, body);
            body.position.copy(obj.position);
            body.orientation.copy(obj.orientation);
            body.velocity.copy(obj.velocity);
            body.angular.copy(obj.angular);
            body.acceleration.copy(obj.acceleration);
            body.scale.copy(obj.scale);
            body.mass = obj.mass;
            body.restitution = obj.restitution;
            body.dynamicfriction = obj.dynamicfriction;
            body.staticfriction = obj.staticfriction;
            //body.force_accumulator.copy(obj.forces);
            body.lineardamping = obj.lineardamping;
            body.angulardamping = obj.angulardamping;

            if (body.parent.id != obj.parentid) {
console.log('object parent changed in worker', body.parent.id, obj.parentid, body, obj, this.objects[obj.parentid]);
              if (this.objects[obj.parentid]) {
                //body.parent.remove(body);
                this.objects[obj.parentid].add(body);
              }
            }
            body.updateAcceleration(framedata);
          }
        }

      } else if (msg.type == 'force_update') {
        let object = this.objects[msg.objectid];
        if (object && object.forces[msg.forcenum]) {
//console.log('update force', object, object.forces[msg.forcenum], msg.force);
          object.forces[msg.forcenum].update(msg.force);
          object.updateAcceleration({});
        }
      }
      //console.log('[physicsworker] got a message', msg);
    }
    onmessage = (ev) => this.handleMessage(ev.data);

    this.advance = function() {
      let now = performance.now(),
          dt = (now - this.lastframe) / 1000;
      this.lastframe = now;

      this.system.step(dt);
      this.sync();
    }
    this.sync = function() {
      //console.log('sync it back');
      postMessage({type: 'object_updates', updates: this.summarizeUpdates()});
    }
    this.summarizeUpdates = function(objects, updates) {
      if (!objects) objects = this.objects;
      if (!updates) updates = {};
      for (let k in objects) {
        if (!objects[k].state.sleeping) {
          updates[k] = {
            position: objects[k].position.toJSON(),
            orientation: objects[k].orientation.toJSON(),
            velocity: objects[k].velocity.toJSON(),
            angular: objects[k].angular.toJSON(),
          };
        }
      }
      return updates;
    }
  });
});

