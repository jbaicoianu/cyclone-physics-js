elation.require(['physics.processors'], function() {
  elation.extend("physics.processor.physx", function(parent, args={}) {
    elation.physics.processor.base.call(this, parent);

    this.path = args.path || '';
console.log('my new physx processor', this.path, parent, parent.children.length, parent.children);
    this.emptyresponse = [];
    //this.objects = { player: player.objects.dynamics };

    elation.events.add(parent, 'add', this);
    elation.events.add(parent, 'remove', this);

    let physx;
    if (typeof importScripts == 'function') {
      importScripts(this.path + 'physx.release.js');
      physx = new PhysXWrapper(this.path, this);
    } else {
      elation.file.get('javascript', this.path + 'physx.release.js', ev => {
        physx = new PhysXWrapper(this.path, this);
      });
    }
    this.iterate = function(objects, t) {
//console.log('agh', objects);
      if (physx) {
        physx.handleObjectChanges(objects);
        //physx.step(t * 1000);
      }
    }
    this.handleAdd = function(obj) {
      console.log('physx added a guy', obj);

      elation.events.add(obj, 'add', this);
      elation.events.add(obj, 'remove', this);
    }
    this.handleRemove = function(obj) {
      console.log('physx removed a guy', obj);
      elation.events.remove(obj, 'add', this);
      elation.events.remove(obj, 'remove', this);
    }
    this.handleEvent = function(ev) {
      if (ev.type == 'add') {
        this.handleAdd(ev.data);
      } else if (ev.type == 'remove') {
        this.handleRemove(ev.data);
      }
    }

    parent.children.forEach(obj => this.handleAdd(obj));

  });
  class PhysXWrapper {
    constructor(path, processor) {
      console.log('Create PhysX wrapper', processor);

      this.framerate = 120;
      this.physxobjects = {};
      this.objects = {};
      this.path = '';
      if (path) {
        this.setPath(path);
      }
      this.initWASM();    
      this.shapemap = new Map();

      //onmessage = (ev) => this.handleMessage(ev);
    }
    setPath(path) {
      this.path = path;
    }
    initWASM() {
      let self = this;
console.log('load physx js');
      //elation.file.get('js', this.path + 'physx.release.js', ev => {
        PHYSX({
          locateFile(path) {
            if (path.endsWith('.wasm')) {
              return self.path + 'physx.release.wasm';
            }
            return path;
          },
        }).then(p => {
          this.PhysX = p;
          console.log('PhysX loaded', this);
          self.initPhysx();
        });
    console.log('do wasm init', this);
      //});
    }
    initPhysx() {
      const PhysX = this.PhysX;
      const version = PhysX.PX_PHYSICS_VERSION;
      const defaultErrorCallback = new PhysX.PxDefaultErrorCallback();
      const allocator = new PhysX.PxDefaultAllocator();
      const foundation = PhysX.PxCreateFoundation(
        version,
        allocator,
        defaultErrorCallback
      );
      const triggerCallback = {
        onContactBegin: (shape1, shape2, contact) => {
          let thing1 = this.shapemap.get(shape1.$$.ptr);
          let thing2 = this.shapemap.get(shape2.$$.ptr);
          if (thing1 && thing2) {
            //postMessage({type: 'contact_begin', thing1: thing1.userData, thing2: thing2.userData, point: contact.position, normal: contact.normal, penetration: -contact.separation});
            elation.events.fire({type: 'physics_collide', element: this.objects[thing1.userData], data: { other: this.objects[thing2.userData], point: contact.position, normal: contact.normal, penetration: -contact.separation}});
            elation.events.fire({type: 'physics_collide', element: this.objects[thing2.userData], data: { other: this.objects[thing1.userData], point: contact.position, normal: contact.normal, penetration: -contact.separation}});
          }
        },
        onContactEnd: () => {},
        onContactPersist: () => {},
        onTriggerBegin: () => {},
        onTriggerEnd: () => {},
      }
  console.log('go physx go, it\'s green ahead');
      const physxSimulationCallbackInstance = PhysX.PxSimulationEventCallback.implement(
        triggerCallback
      )
      let scale = new PhysX.PxTolerancesScale();
      this.physics = PhysX.PxCreatePhysics(
        version,
        foundation,
        scale,
        false,
        null
      )
      this.cooking = PhysX.PxCreateCooking(version, foundation, new PhysX.PxCookingParams(scale));
      PhysX.PxInitExtensions(this.physics, null)
      const sceneDesc = PhysX.getDefaultSceneDesc(
        this.physics.getTolerancesScale(),
        0,
        physxSimulationCallbackInstance
      )
      sceneDesc.gravity = {x: 0, y: -9.8, z: 0}
      sceneDesc.flags |= PhysX.PxSceneFlags.eENABLE_CCD;
  console.log('physx sceneDesc', sceneDesc);
      this.physxscene = this.physics.createScene(sceneDesc)

  /*
      let controllermanager = PhysX.PxCreateControllerManager(this.physxscene, true);
      controllermanager.setTessellation(true, .1);
      controllermanager.setOverlapRecoveryModule(true);
  console.log('I got a controller manager', controllermanager);
      let controllerdesc = new PhysX.PxCapsuleControllerDesc();
      controllerdesc.position = {x: 0, y: 5, z: 0};
      controllerdesc.radius = 0.05;
      controllerdesc.height = 1.9 - controllerdesc.radius * 2;
      controllerdesc.contactOffset = .005;
      controllerdesc.setMaterial(this.physics.createMaterial(1, .9, 0));

  console.log('the controllerdesc', controllerdesc, controllerdesc.radius, controllerdesc.height);
      const queryFilterCallback = PhysX.PxQueryFilterCallback.implement({
        preFilter: (a) => console.log('prefilter', a),   
        postFilter: (a) => console.log('postfilter', a),   
      });
      const controllerFilterCallback = null; //PhysX.PxControllerFilterCallback.implement({ });
      this.controllerfilters = new PhysX.PxControllerFilters(new PhysX.PxFilterData(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff), queryFilterCallback, controllerFilterCallback);
      this.controller = controllermanager.createController(controllerdesc);
  console.log('my controller', this.controller);
  */

      this.start();
      this.physxInitialized = true;
    }
    start() {
      console.log('[worker] PhysX starting');
      this.lastframetime = performance.now();
      this.stepinterval = setInterval(() => this.step(), 1000 / this.framerate);
    }
    stop() {
      if (this.stepinterval) {
        clearTimeout(this.stepinterval);
        this.stepinterval = false;
      }
    }
    step(objects) {
      let now = performance.now(),
          dt = now - this.lastframetime;

      this.physxscene.simulate(dt / 1000, true);
      this.physxscene.fetchResults(true);

      let updates = {};
//console.log('step', dt / 100, this.children, this);
      for (let k in this.physxobjects) {
        if (this.physxobjects[k]) { // FIXME - related to the "stupid hack" mentioned in createPhysxObject
          let obj = this.physxobjects[k];
          //console.log('sync object', k, obj);
          if (obj.isSleeping && !obj.isSleeping()) {
            let transform = obj.getGlobalPose();
            updates[k] = {
              position: transform.translation,
              orientation: transform.rotation,
              velocity: (obj.getLinearVelocity ? obj.getLinearVelocity() : {x:0,y:0,z:0}),
            };
//console.log('update object', obj, updates[k], this.objects[obj.userData]);
            let realobj = this.objects[obj.userData];
            realobj.position.copy(updates[k].position);
            realobj.velocity.copy(updates[k].velocity);
            //realobj.orientation.copy(updates[k].orientation);
          }
        }
      }
      if (this.controller) {
        updates['player'] = {
          position: this.controller.getFootPosition(),
        };
      }
      //console.log(updates);
      //postMessage({type: "object_changes", changes: updates});

      this.lastframetime = now;
    }
/*
    handleMessage(ev) {
      let msg = ev.data;
      //console.log('[worker] PhysX message', msg);
      if (msg.type == 'object_changes') {
        this.handleObjectChanges(msg.changes);
      } else if (msg.type == 'player_move') {
        this.handlePlayerMove(msg.move);
      }
    }
*/
    handleObjectChanges(changes) {
      if (!this.physxInitialized) return;
      const PhysX = this.PhysX;
//console.log('received changes', changes);
      for (let i = 0; i < changes.length; i++) {
        let obj = changes[i];
        if (!(obj.id in this.physxobjects)) {
          // FIXME - For now, we'll just assume that the changes list contains every object in the scene, and we're not removing objects from the scene either
          //         A full implementation will only send diffs, and object add / remove would be handled via separate events
          this.physxobjects[obj.id] = this.createPhysxObject(obj);
          this.objects[obj.id] = obj;
console.log('changed and created', obj);
          elation.events.add(obj, 'collider_change', ev => console.log('object collider changed', obj, ev));
        } else { //if (this.physxobjects[obj.id].updateValues) {
          //this.objects[obj.id].updateValues(obj);
          if (obj.forces && this.physxobjects[obj.id]) {
  /*
            for (let i = 0; i < obj.forces.length; i++) {
              let force = obj.forces[i];
              force.apply();
            }
  console.log('add a force', forcevec, obj, this.physxobjects[obj.id]);
            this.physxobjects[obj.id].addForceAtLocalPos(forcevec, {x: 0, y: 0, z: 0});
  */
  //console.log('add a force', obj.forces, obj, this.physxobjects[obj.id]);
            this.physxobjects[obj.id].setGlobalPose({translation: obj.position, rotation: obj.orientation}, true);
            this.physxobjects[obj.id].setLinearVelocity(obj.velocity, false);
            this.physxobjects[obj.id].setAngularVelocity(obj.angular, false);
//console.log(this.physxobjects[obj.id]);
            obj.updateAcceleration()
            if (this.physxobjects[obj.id].addForceAtLocalPos) {
//console.log('add fvorce', obj.id, obj, obj.force_accumulator);
              this.physxobjects[obj.id].addForceAtLocalPos(obj.force_accumulator.multiplyScalar(obj.mass), {x:0, y: 0, z: 0});
            }
          }
        }
      }
    }
    handlePlayerMove(move) {
      if (this.controller) {
        this.controller.move(move, 1e-6, 1000/60, this.controllerfilters, null);
      }
    }
    createPhysxObject(obj) {
      const PhysX = this.PhysX;
      let physgeo;
      const flags = new PhysX.PxShapeFlags(
        PhysX.PxShapeFlag.eSCENE_QUERY_SHAPE.value | PhysX.PxShapeFlag.eSIMULATION_SHAPE.value
      );
      let relativePose = false;
      if (obj.collider) {
console.log('create new physx collider', obj.collider);
        if (obj.collider.type == 'sphere') {
          physgeo = new PhysX.PxSphereGeometry(obj.scale.x / 2);
        } else if (obj.collider.type == 'box') {
          let colsize = {
            x: obj.scale.x / 2,
            y: obj.scale.y / 2,
            z: obj.scale.z / 2
          };
          
    /*
          if (obj.collider_scale) {
            colsize.multiply(obj.collider_scale);
          }
          colsize.multiplyScalar(.5);
    */
          physgeo = new PhysX.PxBoxGeometry(colsize.x, colsize.y, colsize.z);
          //physgeo = new PhysX.PxBoxGeometry(obj.scale.x * (bbox.max.x - bbox.min.x) / 2, obj.scale.y * (bbox.max.y - bbox.min.y) / 2, obj.scale.z * (bbox.max.z - bbox.min.z) / 2);
        } else if (obj.collider.type == 'capsule') {
          physgeo = new PhysX.PxCapsuleGeometry(obj.scale.x / 2, obj.scale.y);
          relativePose = { translation: {x: 0, y: 0, z: 0}, rotation: {x: 0, y: 0, z: 0.7071, w: 0.7071}};
        } else if (obj.collider.type == 'cylinder') {
          //physgeo = new PhysX.Px(obj.scale.x / 2, obj.scale.y / 2);
          //let cylverts = [{x: 0, y: 0.5, z: 1}, {x: 0.7071, y: 0.5, z: 0.7071}, {x: 1, y: 0.5, z: 0}, {x: 0.7071, y: 0.5, z: -0.7071}, {x: 0, y: 0.5, z: -1}, {x: -0.7071, y: 0.5, z: -0.7071}, {x: -1, y: 0.5, z: 0}, {x: -0.7071, y: 0.5, z: 0.7071}, {x: 0, y: -0.5, z: 1}, {x: 0.7071, y: -0.5, z: 0.7071}, {x: 1, y: -0.5, z: 0}, {x: 0.7071, y: -0.5, z: -0.7071}, {x: 0, y: -0.5, z: -1}, {x: -0.7071, y: -0.5, z: -0.7071}, {x: -1, y: -0.5, z: 0}, {x: -0.7071, y: -0.5, z: 0.7071}, {x: 0, y: 0.5, z: 0}, {x: 0, y: -0.5, z: 0}];
          //let cylverts = [{x: 0, y: 0.5, z: 1}, {x: 0.7071, y: 0.5, z: 0.7071}, {x: 1, y: 0.5, z: 0}, {x: 0.7071, y: 0.5, z: -0.7071}, {x: 0, y: 0.5, z: -1}, {x: -0.7071, y: 0.5, z: -0.7071}, {x: -1, y: 0.5, z: 0}, {x: -0.7071, y: 0.5, z: 0.7071}, {x: 0, y: -0.5, z: 1}, {x: 0.7071, y: -0.5, z: 0.7071}, {x: 1, y: -0.5, z: 0}, {x: 0.7071, y: -0.5, z: -0.7071}, {x: 0, y: -0.5, z: -1}, {x: -0.7071, y: -0.5, z: -0.7071}, {x: -1, y: -0.5, z: 0}, {x: -0.7071, y: -0.5, z: 0.7071}, {x: 0, y: 0.5, z: 0}, {x: 0, y: -0.5, z: 0}];
          //let cylverts = [{"x":0.5,"y":1.0,"z":0.5}, {"x":0.5,"y":1.0,"z":-0.5}, {"x":0.5,"y":0.0,"z":0.5}, {"x":0.5,"y":0.0,"z":-0.5}, {"x":-0.5,"y":1.0,"z":-0.5}, {"x":-0.5,"y":1.0,"z":0.5}, {"x":-0.5,"y":0.0,"z":-0.5}, {"x":-0.5,"y":0.0,"z":0.5}];
          let cylverts = [{x: 0, y: 1, z: 0.5}, {x: 0.3536, y: 1, z: 0.3536}, {x: 0.5, y: 1, z: 0}, {x: 0.3536, y: 1, z: -0.3536}, {x: 0, y: 1, z: -0.5}, {x: -0.3536, y: 1, z: -0.3536}, {x: -0.5, y: 1, z: 0}, {x: -0.3536, y: 1, z: 0.3536}, {x: 0, y: 0, z: 0.5}, {x: 0.3536, y: 0, z: 0.3536}, {x: 0.5, y: 0, z: 0}, {x: 0.3536, y: 0, z: -0.3536}, {x: 0, y: 0, z: -0.5}, {x: -0.3536, y: 0, z: -0.3536}, {x: -0.5, y: 0, z: 0}, {x: -0.3536, y: 0, z: 0.3536}, {x: 0, y: 1, z: 0}, {x: 0, y: 0, z: 0}];
          let foo = new PhysX.PxVec3Vector();
          cylverts.forEach(v => {
            v.x *= obj.scale.x;
            v.y *= obj.scale.y;
            v.z *= obj.scale.z;
            foo.push_back(v)
          });
          physgeo = new PhysX.PxConvexMeshGeometry(this.cooking.createConvexMesh(foo, this.physics), new PhysX.PxMeshScale({x: 1, y: 1, z: 1}, {x: 0, y: 0, z: 0, w: 1}), new PhysX.PxConvexMeshGeometryFlags(1));

          //relativePose = { translation: {x: 0, y: 0, z: 0}, rotation: {x: 0, y: 0, z: 0, w: 1}};
        } else if (obj.collider.type == 'mesh') {
console.log('PHYSX MESH COLLIDER', obj.collider.meshdata, obj.collider);
        }
      } 
      let transform = { translation: obj.position, rotation: obj.orientation };
      let body = false; // FIXME - stupid hack, need to find out how to support objects which we want physically simulated but without colliders
      if (physgeo) {
        body = (obj.mass > 0 ? this.physics.createRigidDynamic(transform) : this.physics.createRigidStatic(transform));
        let physmat = this.physics.createMaterial(obj.material.dynamicfriction, obj.material.dynamicfriction, obj.restitution)
        let shape = this.physics.createShape(physgeo, physmat, true, flags)
        if (relativePose) {
          shape.setLocalPose(relativePose);
        }
        shape.setSimulationFilterData(new PhysX.PxFilterData(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff));
        body.attachShape(shape);
        body.setActorFlag(PhysX.PxActorFlag.eDISABLE_GRAVITY, true);

        if (obj.mass > 0) {
          body.setMassAndUpdateInertia(obj.mass);
          body.setLinearVelocity(obj.velocity, false);
          body.setAngularVelocity(obj.angular, false);
          body.setAngularDamping(obj.angularDamping);
        //body.wakeUp();
    //body.setRigidBodyFlag(PhysX.PxRigidBodyFlag.eKINEMATIC, true);
          body.setLinearDamping(obj.linearDamping);
          body.setRigidBodyFlag(PhysX.PxRigidBodyFlag.eENABLE_CCD, true);
        }
        this.physxscene.addActor(body, null);

        //body.userData = obj;
        shape.userData = obj.id;
        body.userData = obj.id;

        // FIXME - userData doesn't seem to be working, so keep our own map
        this.shapemap.set(shape.$$.ptr, body);

        if (obj.sleeping && obj.mass > 0) {
          //body.putToSleep();
          //body.setSleepThreshold(100000);
          //body.setWakeCounter(-100);
  //console.log('try to make object sleep', obj, body.getSleepThreshold(), body.getWakeCounter());
        }
      }

      return body;
    }
  }

});
