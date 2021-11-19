elation.require(['physics.common', 'physics.cyclone', 'physics.processors.cpu', 'janusweb.external.physx.physx-cyclone-processor'], function() {

  // `elation.physics.worker` is the code that runs in a worker when using multithreaded physics
  // It interfaces with an `elation.physics.processor.worker` processor in the main thread, and keeps
  // the worker's scene in sync with the main thread's scene.

  elation.extend("physics.worker", function(parent) {
    this.fps = 120;
    this.objects = {};
    this.addqueue = [];

    this.colliderupdates = new Set();

    this.handleMessage = function(msg) {
      if (msg.type == 'system_init') {
        this.system = new elation.physics.system({autostart: false, processortype: msg.processortype});
        this.system.start(msg.args);

        if (this.addqueue.length > 0) {
          this.addqueue.forEach(obj => {
            this.system.add(obj);
          });
        }

        this.lastframe = performance.now();
/*
        setInterval(() => { 
          this.advance();
        }, 1000 / this.fps);
  */
this.advance();      
      } else if (msg.type == 'object_add') {
        let obj = msg.object;
        if (!this.objects[obj.id]) {
          this.addObjectToSystem(obj);
          console.log('worker added new object', obj.id, obj, this.objects);
        }
      } else if (msg.type == 'object_remove') {
        let object = this.objects[msg.objectid],
            parent = this.objects[msg.parentid];
        //console.log('worker object removed', parent, object, object.parent, msg);
        if (object && parent) {
          parent.remove(object);
        }
      } else if (msg.type == 'object_changes') {
        let framedata = {};
        for (let i = 0; i < msg.changes.length; i++) {
          let obj = msg.changes[i];
          if (!this.objects[obj.id]) {
            this.addObjectToSystem(obj);
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
            body.linearDamping = obj.linearDamping;
            body.angulardamping = obj.angulardamping;
            body.gravity = obj.gravity;

            if (body.parent.id != obj.parentid) {
              //console.log('object parent changed in worker', body.parent.id, obj.parentid, body, obj, this.objects[obj.parentid]);
              if (this.objects[obj.parentid]) {
                //body.parent.remove(body);
                this.objects[obj.parentid].add(body);
              }
            }
            body.updateAcceleration(framedata);
          }
        }
      } else if (msg.type == 'collider_change') {
        let object = this.objects[msg.objectid];
        console.log('worker says a collider changed', msg.objectid, object, msg.collider.type, msg.collider.length, msg.collider.offset, msg.collider.meshdata, msg.collider, this.objects);
        if (object) {
          object.setCollider(msg.collider.type, msg.collider);
          this.colliderupdates.add(object);
console.log('add collider update to set', this.colliderupdates);
        } else {
console.log('couldnt find object', msg.objectid, this.objects);
        }
      } else if (msg.type == 'force_update') {
        let object = this.objects[msg.objectid];
        if (object && object.forces[msg.forcenum]) {
          //console.log('update force', object, object.forces[msg.forcenum], msg.force.force);
          object.forces[msg.forcenum].update(msg.force);
          object.updateAcceleration({});
/*
if (object.forces[msg.forcenum].force && object.forces[msg.forcenum].force.length() > 0) {
  console.log(object.forces[msg.forcenum].force.toArray());
}
*/
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
setTimeout(() => this.advance(), 1000 / this.fps);
    }
    this.sync = function() {
      //console.log('sync it back');
      postMessage({type: 'object_updates', updates: this.summarizeUpdates()});
    }
    this.summarizeUpdates = function(objects, updates) {
      if (!objects) objects = this.objects;
      if (!updates) updates = {};
      for (let k in objects) {
        if (!objects[k].state.sleeping || this.colliderupdates.has(objects[k])) {
          updates[k] = {
            position: objects[k].position.toJSON(),
            orientation: objects[k].orientation.toJSON(),
            velocity: objects[k].velocity.toJSON(),
            angular: objects[k].angular.toJSON(),
          };
//console.log('sync', updates[k].position, updates[k].velocity, objects[k].acceleration);
        }
      }
      this.colliderupdates.clear();
      return updates;
    }
    this.addObjectToSystem = function(obj) {
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
        linearDamping: obj.linearDamping,
        angularDamping: obj.angularDamping,
        gravity: obj.gravity,
      });
//console.log('new rigidbody', obj.id, obj.collider, obj, this.objects[obj.id], this.objects, msg.changes);
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
        if (this.system) {
          this.system.add(this.objects[obj.id]);
        } else {
          // System not created yet, add to queue
          this.addqueue.push(this.objects[obj.id]);
        }
      }
      if (obj.collider) {
        let collider = new elation.physics.colliders[obj.collider.type](this.objects[obj.id], obj.collider);
        this.objects[obj.id].collider = collider;
        this.objects[obj.id].collider.getInertialMoment();
        this.objects[obj.id].collider.trigger = obj.collider.trigger;
        let root = (collider.type == 'mesh' ? collider.getRoot() : collider.body);
        elation.events.add(root, 'physics_collide', (ev) => {
          let contact = ev.data,
              body1 = contact.bodies[0],
              body2 = contact.bodies[1];
          postMessage({type: 'contact_begin', thing1: body1.id, thing2: body2.id, point: contact.point, normal: contact.normal, penetration: contact.penetration});
        });
      }
    }
  });
});

