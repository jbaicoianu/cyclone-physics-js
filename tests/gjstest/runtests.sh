#!/bin/sh

ELATIONPATH=/home/bai/elation

gjstest --js_files=$ELATIONPATH/components/utils/scripts/elation.js,$ELATIONPATH/components/utils/scripts/events.js,../engine/scripts/external/three/three.js,scripts/cyclone.js,scripts/collisions.js,scripts/rigidbody.js,scripts/forces.js,scripts/processors.js,tests/elation-test.js 
