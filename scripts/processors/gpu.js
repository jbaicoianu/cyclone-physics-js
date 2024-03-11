elation.require(["physics.processors"], function() {
  class GPUPhysicsBufferGroup {
    constructor(device, length, mapped=true, usage=GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST | GPUBufferUsage.COPY_SRC, name='') {
      this.length = length;
      this.device = device;
      let buffers = this.buffers = {};
      //let usage = (mapped ? GPUBufferUsage.COPY_SRC | GPUBufferUsage.MAP_WRITE : GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST | GPUBufferUsage.COPY_SRC);
      buffers.position = device.createBuffer({
        label: 'position buffer '+ name,
        mappedAtCreation: mapped,
        size: length * 3 * 4,
        usage: usage,
      });
      buffers.velocity = device.createBuffer({
        label: 'velocity buffer ' + name,
        mappedAtCreation: mapped,
        size: length * 3 * 4,
        usage: usage,
      });
      buffers.force = device.createBuffer({
        label: 'force buffer ' + name,
        mappedAtCreation: mapped,
        size: length * 3 * 4,
        usage: usage,
      });
      buffers.mass = device.createBuffer({
        label: 'mass buffer ' + name,
        mappedAtCreation: mapped,
        size: length * 4,
        usage: usage,
      });
      buffers.uniforms = device.createBuffer({
        label: 'uniforms',
        mappedAtCreation: mapped,
        size: 8,
        usage: usage,
      });
    }
    fill(objects, time) {
      let buffers = this.buffers;
      let positions = new Float32Array(buffers.position.getMappedRange()),
          velocities = new Float32Array(buffers.velocity.getMappedRange()),
          forces = new Float32Array(buffers.force.getMappedRange()),
          masses = new Float32Array(buffers.mass.getMappedRange()),
          uniform_buffer = buffers.uniforms.getMappedRange(),
          uniform_objcount = new Uint32Array(uniform_buffer, 0, 1),
          uniform_time = new Float32Array(uniform_buffer, 4, 1);

      uniform_time[0] = time;
      uniform_objcount[0] = objects.length;

      for (let i = 0; i < objects.length; i++) {
        let offset = i * 3,
            obj = objects[i],
            pos = obj.position,
            vel = obj.velocity;
        masses[i] = obj.mass;

        positions[offset    ] = pos.x;
        positions[offset + 1] = pos.y;
        positions[offset + 2] = pos.z;
        pos.reset();

        velocities[offset    ] = vel.x;
        velocities[offset + 1] = vel.y;
        velocities[offset + 2] = vel.z;
        vel.reset();

        forces[offset    ] = 0;
        forces[offset + 1] = 0;
        forces[offset + 2] = 0;
      }
    }
    update(objects) {
      let buffers = this.buffers;
      let positions = new Float32Array(buffers.position.getMappedRange()),
          velocities = new Float32Array(buffers.velocity.getMappedRange()),
          forces = new Float32Array(buffers.force.getMappedRange()),
          masses = new Float32Array(buffers.mass.getMappedRange());

      for (let i = 0; i < objects.length; i++) {
        let offset = i * 3,
            obj = objects[i],
            pos = obj.position,
            vel = obj.velocity;
        masses[i] = obj.mass;

        if (!pos.changed) {
          pos.x = positions[offset    ];
          pos.y = positions[offset + 1];
          pos.z = positions[offset + 2];
          pos.reset();
        }

        if (!vel.changed) {
          vel.x = velocities[offset    ];
          vel.y = velocities[offset + 1];
          vel.z = velocities[offset + 2];
          vel.reset();
        }

        //forces[offset    ] = 0;
        //forces[offset + 1] = 0;
        //forces[offset + 2] = 0;
      }
    }
    mapAsync(mode=GPUMapMode.WRITE) {
      let promises = [];
      for (let buffer in this.buffers) {
        promises.push(this.buffers[buffer].mapAsync(mode));
      }
      return Promise.all(promises);
    }
    release() {
      for (let buffer in this.buffers) {
        this.buffers[buffer].unmap();
      }
    }
    copyTo(buffers, commandEncoder) {
      for (let name in this.buffers) {
        // This might be a source of bugs, if the length of our destination buffer is shorter we should probably resize it
        let size = Math.min(this.buffers[name].size, buffers.buffers[name].size);
        commandEncoder.copyBufferToBuffer(
          this.buffers[name], 0,
          buffers.buffers[name], 0,
          size
        );
      }
    }
    getBindings() {
      const bindGroupLayout = this.device.createBindGroupLayout({
        label: 'physics bind group layout',
        entries: [

          {
            binding: 0,
            visibility: GPUShaderStage.COMPUTE,
            buffer: {
              type: 'storage'
            },
          },
          {
            binding: 1,
            visibility: GPUShaderStage.COMPUTE,
            buffer: {
              type: 'storage'
            },
          },
          {
            binding: 2,
            visibility: GPUShaderStage.COMPUTE,
            buffer: {
              type: 'storage'
            },
          },
          {
            binding: 3,
            visibility: GPUShaderStage.COMPUTE,
            buffer: {
              type: 'storage'
            },
          },
          {
            binding: 4,
            visibility: GPUShaderStage.COMPUTE,
            buffer: {
              type: 'storage'
            },
          },
        ]
      });

      const bindGroup = this.device.createBindGroup({
        label: 'physics bind group',
        layout: bindGroupLayout,
        entries: [

          {
            binding: 0,
            resource: { buffer: this.buffers.mass }
          },
          {
            binding: 1,
            resource: { buffer: this.buffers.position }
          },
          {
            binding: 2,
            resource: { buffer: this.buffers.velocity }
          },
          {
            binding: 3,
            resource: { buffer: this.buffers.force }
          },
          {
            binding: 4,
            resource: { buffer: this.buffers.uniforms }
          },
        ]
      });
      return { bindGroupLayout, bindGroup };
    }
    dump() {
      console.log('=================', this.length);
      for (let k in this.buffers) {
        console.log(k, new Float32Array(this.buffers[k].getMappedRange()));
      }
    }
  }


  elation.extend("physics.processor.gpu", function(parent) {
    elation.physics.processor.base.call(this, parent);

    // Initialize GPU device
    // set up initial buffers
    this.stagingBuffers = [];
    this.readBuffers = [];

    this.initWebGPU = async function() {
      this.webGPUInitializing = true;
      this.gpuBuffers = false;
      console.log('Init WebGPU physics');
      // Initialize GPU device
      this.adapter = await navigator.gpu.requestAdapter();
      this.device = await this.adapter.requestDevice();

      // FIXME - we're preallocating GPU memory for up to 100k items here. Clearly we should be
      //         allocating dynamically, presumably with a block allocator for efficiency
      this.gpuBuffers = this.createBuffers(new Array(100000), mapped=false, usage=GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST | GPUBufferUsage.COPY_SRC);

      let bindings = this.gpuBuffers.getBindings()
      this.bindings = bindings;
      // compile compute shader
      const shaderModule_forces = this.device.createShaderModule({
        code: `
          struct UBO {
            numobjects: u32,
            time: f32,
          };
          const G = 6.6743e-11;
          @group(0) @binding(0) var<storage, read_write> mass : array<f32>;
          @group(0) @binding(1) var<storage, read_write> position : array<f32>;
          @group(0) @binding(2) var<storage, read_write> velocity : array<f32>;
          @group(0) @binding(3) var<storage, read_write> force : array<f32>;
          @group(0) @binding(4) var<storage, read_write> uniforms : UBO;
          //@group(0) @binding(5) var<storage, read_write> time : array<f32>;

          @compute @workgroup_size(128) fn main( @builtin(global_invocation_id) global_id : vec3<u32>,
                                                @builtin(local_invocation_id) local_id : vec3<u32>,
                                              )
          {
            let nodecount = uniforms.numobjects;
            if (global_id.x > nodecount-1) {
              return;
            }

            var nodenum = u32(global_id.x);
            let t = uniforms.time;

            let offset = nodenum * 3;
            //force[offset  ] = 0;
            //force[offset+1] = 0;
            //force[offset+2] = 0;
            //var tforce = vec3<f32>(0,0,0);
            let p1 = vec3<f32>(position[offset], position[offset+1], position[offset+2]);

            for (var i = 0u; i < nodecount; i++) {
              let otherOffset = i * 3u;
              let p2 = vec3<f32>(position[otherOffset], position[otherOffset+1], position[otherOffset+2]);
              let dir = p1 - p2;
              let dist = length(dir);
              if (dist > 0) {
                let localforce = normalize(dir) * -(G * mass[nodenum] * mass[i]) / pow(dist, 2.0);
                //tforce += localforce;

                force[offset  ] += localforce.x;
                force[offset+1] += localforce.y;
                force[offset+2] += localforce.z;

                //force[otherOffset  ] -= localforce.x;
                //force[otherOffset+1] -= localforce.y;
                //force[otherOffset+2] -= localforce.z;
              }
            }
          }
        `
      });

      const shaderModule_positions = this.device.createShaderModule({
        code: `
          struct UBO {
            numobjects: u32,
            time: f32,
          };
          @group(0) @binding(0) var<storage, read_write> mass : array<f32>;
          @group(0) @binding(1) var<storage, read_write> position : array<f32>;
          @group(0) @binding(2) var<storage, read_write> velocity : array<f32>;
          @group(0) @binding(3) var<storage, read_write> force : array<f32>;
          @group(0) @binding(4) var<storage, read_write> uniforms : UBO;

          @compute @workgroup_size(128) fn main( @builtin(global_invocation_id) global_id : vec3<u32>,
                                                @builtin(local_invocation_id) local_id : vec3<u32>,
                                              )
          {
            let nodecount = uniforms.numobjects;
            if (global_id.x > nodecount) {
              return;
            }

            var nodenum = u32(global_id.x);
            let t = uniforms.time;


            let offset = nodenum * 3;
            let tforce = vec3<f32>(force[offset], force[offset+1], force[offset+2]);
            var newvel = (tforce / mass[nodenum]) * t + vec3<f32>(velocity[offset], velocity[offset+1], velocity[offset+2]);
/*
            velocity[offset] = velocity[offset] + (tforce.x / mass[idx]) * t;
            velocity[offset+1] = velocity[offset+1] + (tforce.y / mass[idx]) * t;
            velocity[offset+2] = velocity[offset+2] + (tforce.z / mass[idx]) * t;
*/
            velocity[offset] = newvel.x;
            velocity[offset+1] = newvel.y;
            velocity[offset+2] = newvel.z;
        

            position[offset] = position[offset] + newvel.x * t;
            position[offset+1] = position[offset+1] + newvel.y * t;
            position[offset+2] = position[offset+2] + newvel.z * t;
          }
        `
      });
      // setup pipeline
      if (true) {
        this.computePipeline_forces = this.device.createComputePipeline({
          layout: this.device.createPipelineLayout({
            bindGroupLayouts: [bindings.bindGroupLayout],
          }),
          compute: {
            module: shaderModule_forces,
            entryPoint: 'main'
          }
        });
        this.computePipeline_positions = this.device.createComputePipeline({
          layout: this.device.createPipelineLayout({
            bindGroupLayouts: [bindings.bindGroupLayout],
          }),
          compute: {
            module: shaderModule_positions,
            entryPoint: 'main'
          }
        });
      } else {
        // TODO - this is just me brainstorming what a more flexible API would look like. It still needs to be implemented.
        this.setPipeline([
          new elation.physics.solvers.nbodygravity(),
          new elation.physics.solvers.spring(),
          new elation.physics.solvers.buoyancy(),
          new elation.physics.solvers.electrostatic(),
          new elation.physics.solvers.collisions(),
          new elation.physics.solvers.projectile(),
        ]);
      }

console.log('my pipeline', this.computePipeline, bindings, shaderModule_forces, shaderModule_positions);

    }

    this.start = function() {
console.log('GO START IT');
      if (!this.webGPUInitializing) {
        this.initWebGPU();
      }
    }

    this.iterateVelocities = function(objects, t) {
      if (this.gpuBuffers) {
        if (this.readingBack) {
          // FIXME - hack, we're skipping frames if the readback is still occurring, we should just await the readback but that'll require additional work higher up in the library
          return;
        }
        //console.log('go!', objects);
        const commandEncoder = this.device.createCommandEncoder(),
              stagingBuffers = this.getStagingBuffers(objects),
              readBuffers = this.getReadBuffers(objects);

        stagingBuffers.fill(objects, t);
        stagingBuffers.release();
        stagingBuffers.copyTo(this.gpuBuffers, commandEncoder);

        const passEncoder = commandEncoder.beginComputePass();
        passEncoder.setPipeline(this.computePipeline_forces);
        passEncoder.setBindGroup(0, this.bindings.bindGroup);
        passEncoder.dispatchWorkgroups(Math.ceil(objects.length / 128));
        passEncoder.end();

        const passEncoder2 = commandEncoder.beginComputePass();
        passEncoder2.setPipeline(this.computePipeline_positions);
        passEncoder2.setBindGroup(0, this.bindings.bindGroup);
        passEncoder2.dispatchWorkgroups(Math.ceil(objects.length / 128));
        passEncoder2.end();

        this.gpuBuffers.copyTo(readBuffers, commandEncoder);

        const gpuCommands = commandEncoder.finish();
        this.device.queue.submit([gpuCommands]);

        // remap staging and read buffers and return to pool
        stagingBuffers.mapAsync().then(() => {
          this.stagingBuffers.push(stagingBuffers);
        })
        this.readingBack = true;
        readBuffers.mapAsync(GPUMapMode.READ).then(() => {
          this.readingBack = false;
          //readBuffers.dump();
          readBuffers.update(objects);
          readBuffers.release();
          this.readBuffers.push(readBuffers);
        });
      } else if (!this.webGPUInitializing) {
        this.initWebGPU();
      }
    }
    this.getStagingBuffers = function(objects) {
      let stagingBuffers = false;
      if (this.stagingBuffers.length > 0) {
        stagingBuffers = this.stagingBuffers.pop();
        if (stagingBuffers.length < objects.length) {
          //stagingBuffers.resize(objects.length);
          // FIXME - probably need to free the old buffer when this happens
          stagingBuffers = false;
        }
      }
      if (!stagingBuffers) {
        //stagingBuffers = this.createBuffers(objects, true);
        stagingBuffers = this.createBuffers(objects, mapped=true, usage=GPUBufferUsage.COPY_SRC | GPUBufferUsage.MAP_WRITE);
        console.log('allocate staging buffer');
      }
      //if (stagingBuffers) {
      //  this.fillBuffers(stagingBuffers, objects);
      //}
      return stagingBuffers;
    }
    this.getReadBuffers = function(objects) {
      let readBuffers = false;
      if (this.readBuffers.length > 0) {
        readBuffers = this.readBuffers.pop();
        if (readBuffers.length < objects.length) {
          //readBuffers.resize(objects.length);
          // FIXME - probably need to free the old buffer when this happens
          readBuffers = false;
        }
      }
      if (!readBuffers) {
        readBuffers = this.createBuffers(objects, mapped=false, usage=GPUBufferUsage.COPY_DST | GPUBufferUsage.MAP_READ);
      }
      return readBuffers;
    }
    this.createBuffers = function(objects, mapped=false, usage) {
      return new GPUPhysicsBufferGroup(this.device, objects.length, mapped, usage);
    }
    this.fillBuffers = function(buffers, objects) {
    }
    this.update = function(objects, t, active) {
      if (typeof active == 'undefined') active = [];
      for (var i = 0; i < objects.length; i++) {
        var obj = objects[i];
        //obj.updateState();
        //if (!obj.state.sleeping) {
          active.push(obj);
        //}
        if (obj.children.length > 0) {
          this.update(obj.children, t, active);
        }
      }
      return active;
    }

  }, false, elation.physics.processor.base);
});
