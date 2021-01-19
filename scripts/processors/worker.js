elation.require(['physics.processors'], function() {
  elation.extend("physics.processor.worker", function(parent) {
    elation.physics.processor.base.call(this, parent);

    this.worker = new elation.worker.thread('physics.worker', 'physicsworker');
    elation.events.add(this.worker, 'message', ev => this.handleMessage(ev));
    //this.worker.postMessage({type: 'start', processortype: 'cpu'});
    this.emptyresponse = [];
    this.objects = {};

    this.update = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      let changes = this.summarizeChanges(objects);
//console.log(changes);
      this.worker.postMessage({type: 'object_changes', changes: changes});
      return this.emptyresponse;
    }
    this.iterate = function(objects, t) {
    }
    this.summarizeChanges = function(objects, summary) {
      // TODO - this could be made more efficient by only summarizing RigidBody objects that changed in response to something in this thread (eg, scripts)
      // We should also be summarizing to an ArrayBuffer, and swapping buffers back and forth, rather than allocating a new array each frame
      if (!summary) summary = [];
      for (let i = 0; i < objects.length; i++) {
        let object = objects[i];
        //object.updateState();
        if (object.hasChanged()) {
          // FIXME - for now, we just return a full serialization of all changed objects, with a full list of the properties we care about for physics
          let orientation = object.orientation;

          let objdata = {
            id: object.id || object.object.objects['3d'].uuid,
            position: object.position.toJSON(),
            orientation: object.orientation.toJSON(), //{ x: orientation.x, y: orientation.y, z: orientation.z, w: orientation.w }, // Three.js's quaternion uses _-prefixed names when serialized for some reason
            scale: object.scale.toJSON ? object.scale.toJSON() : object.scale,
            velocity: object.velocity.toJSON(),
            acceleration: object.acceleration.toJSON(),
            angular: object.angular.toJSON(),
            mass: object.mass,
            restitution: object.restitution,
            dynamicfriction: object.material.dynamicfriction,
            staticfriction: object.material.staticfriction,
            //forces: object.force_accumulator,
            lineardamping: object.linearDamping,
            angulardamping: object.angularDamping,
            //sleeping: true,
          };
          if (object.parent) {
//console.log('object in main thread had parent', object.id, object.parent.id, object, object.parent);
            objdata.parentid = object.parent.id;
          }
          if (!this.objects[objdata.id] || this.objects[objdata.id] !== object) {
            this.objects[objdata.id] = object;
            if (object.forces.length > 0) {
              objdata.forces = [];
              for (let i = 0; i < object.forces.length; i++) {
                let f = object.forces[i];
                objdata.forces.push(f.toJSON());
                elation.events.add(f, 'physics_force_update', ev => this.worker.postMessage({type: 'force_update', objectid: objdata.id, forcenum: i, force: f.toJSON()}));
              }
            }
          }
          if (object.collider) {
            objdata.collider = object.collider.toJSON();
          }
          summary.push(objdata);
          //object.resetChangedFlag();
        }
        if (object.children && object.children.length > 0) {
          this.summarizeChanges(object.children, summary);
        }
      }
      return summary;
    }
    this.handleMessage = function(ev) {
      // TODO - see physx-worker.js for all message types
      let msg = ev.data;
      if (msg.type == 'object_updates') {
        this.handleObjectUpdates(msg.updates);
      } else if (msg.type == 'contact_begin') {
        let body1 = this.objects[msg.thing1],
            thing1 = body1.object,
            body2 = this.objects[msg.thing2],
            thing2 = body2.object;
        // FIXME - PhysX wasm port doesn't return contact information, need to implement that in the web bindings
        //         For now we just treat it like a sphere so we can provide a best guess
        // https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/AdvancedCollisionDetection.html#extracting-contact-information
        // https://github.com/prestomation/PhysX/blob/emscripten/physx/source/physxwebbindings/src/PxWebBindings.cpp#L43-L59
        // FIXME^2 - we should take into account the radii of each object's bounding sphere when faking our collision point
        let point = V().copy(msg.point),
            normal = V().copy(msg.normal),
            penetration = msg.penetration || 0;
        if (!point) point = V().copy(body1.positionWorld).add(body2.positionWorld).multiplyScalar(.5);
        if (!normal) normal = V().copy(body1.positionWorld).sub(body2.positionWorld).normalize();

        let contact = new elation.physics.contact({
          normal: normal,
          point: point,
          penetration: penetration,
          bodies: [body1, body2]
        });
        elation.events.fire({type: 'physics_collide', element: body1, data: contact});
        elation.events.fire({type: 'physics_collide', element: body2, data: contact});
/*
        let contact2 = new elation.physics.contact({
          normal: normal.clone().multiplyScalar(-1),
          point: point,
          penetration: penetration,
          bodies: [body2, body1]
        });
        elation.events.fire({type: 'physics_collide', element: body2, data: contact2});
*/
      }
    }
    this.handleObjectUpdates = function(updates) {
      for (let k in updates) {
        let update = updates[k],
            obj = this.objects[k];
        if (obj && /*obj.hasChanged() &&*/ obj.parent) {
          if (update.position && !obj.position.changed) obj.position.copy(update.position);
          if (update.orientation && !obj.orientation.changed) obj.orientation.copy(update.orientation);
          if (update.velocity && !obj.velocity.changed) obj.velocity.copy(update.velocity);
          if (update.angular && !obj.angular.changed) obj.angular.copy(update.angular);
          obj.updateState();
          obj.position.reset();
          obj.orientation.reset();
          obj.positionWorld.reset();
          obj.orientationWorld.reset();
          obj.velocity.reset();
          obj.angular.reset();
          elation.events.fire({element: obj, type: "physics_update"});
        }
      }
    }
  }, false, elation.physics.processor.base);
});
