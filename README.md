Cyclone Javascript Physics Engine

A manual port of the Cyclone Physics Engine (http://procyclone.com/) from C++ to JavaScript

Usage:


```javascript
import cyclone from './cyclone.js'

const system = new cyclone.system(),
      framerate = 10,
      seconds = 10

var smallball = new cyclone.rigidbody({
  mass: 1,
  position: new cyclone.vector3(0, 0, 1),
  velocity: new cyclone.vector3(0, 0, -1)
})
smallball.setCollider("sphere", 2)
//elation.events.add(smallball, "collide", function() { console.log('small boing!') })
system.add(smallball)

var bigball = new cyclone.rigidbody({
  mass: 10,
  position: new cyclone.vector3(1, 0, 0),
  velocity: new cyclone.vector3(-1, 0, 0)
})
bigball.setCollider("sphere", 5)

//elation.events.add(bigball, "collide", function() { console.log('big boing!') })
system.add(bigball)

for (let i = 0; i < framerate * seconds; i++) {
  let dt = i * 1 / framerate
  system.step(dt)

  console.log(`===== Frame ${i} (${dt} seconds) =====`)
  console.log(`small ball position: (${smallball.position.toArray().join(',')})`)
  console.log(`big ball position: (${bigball.position.toArray().join(',')})`)
}
```
