# Cyclone Javascript Physics Engine

A lightweight JavaScript physics engine that runs in browsers and in NodeJS. Pluggable architecture supports swapping out the processing engine using CPU, WebGPU, NVIDIA PhysX (WASM), and more.

## Usage:
```javascript
import cyclone from './cyclone.js'

const system = new cyclone.system(),
      framerate = 60,
      seconds = 10

let smallball = new cyclone.rigidbody({
  mass: 1,
  position: new cyclone.vector3(0, 0, 10),
  velocity: new cyclone.vector3(0, 0, -1)
})
smallball.setCollider("sphere", 2)
smallball.addEventListener('physics_collide', ev => console.log('small ball collided', ev.detail));
system.add(smallball)

let bigball = new cyclone.rigidbody({
  mass: 10,
  position: new cyclone.vector3(10, 0, 0),
  velocity: new cyclone.vector3(-1, 0, 0)
})
bigball.setCollider("sphere", 5)
bigball.addEventListener('physics_collide', ev => console.log('big ball collided', ev.detail));
system.add(bigball)

for (let i = 0; i < framerate * seconds; i++) {
  let dt = i * 1 / framerate
  system.step(dt)

  console.log(`===== Frame ${i} (${dt} seconds) =====`)
  console.log(`small ball position: (${smallball.position.toArray().join(',')})`)
  console.log(`big ball position: (${bigball.position.toArray().join(',')})`)
}
```

## History
Initially based on the Cyclone Physics Engine (https://github.com/idmillington/cyclone-physics) as discussed in the book <u>Game Physics Engine Design</u>, the engine has evolved significantly since its initial implementation. I wrote this physics engine to handle physics for Elation Engine back in 2011, after I outgrew the physics engine I'd written in 2008 to power a physically-simulated JavaScript UI toolkit for mobile phones of the time (eg, iPhone 3). At the time, there were few if any options for 3d physics on the web. It's now primarily used to provide physics to the JanusWeb virtual world client.
