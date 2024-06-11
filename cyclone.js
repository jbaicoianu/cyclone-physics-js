import { PhysicsSystem } from './scripts/system.js';
import { RigidBody } from './scripts/rigidbody.js';
import { StaticContact, DynamicContact } from './scripts/contacts.js';
import { colliders } from './scripts/collisions.js';
import { forces } from './scripts/forces.js';
import { CPUPhysicsProcessor } from './scripts/processors/cpu.js';
import { CycloneVector3, CycloneQuaternion } from './scripts/common.js';

const cyclone = {
  system: PhysicsSystem,
  processors: {
    cpu: CPUPhysicsProcessor
  },
  rigidbody: RigidBody,
  contacts: {
    'static': StaticContact,
    'dynamic': DynamicContact
  },
  forces: forces,
  colliders: colliders,
  vector3: CycloneVector3,
  quaternion: CycloneQuaternion,
}

export default cyclone;

