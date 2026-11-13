# quaternion-rotation-interpolation-lib

Spherical linear interpolation between unit quaternions, plus the composition and vector-rotation helpers needed to build a 3D orientation pipeline.

```js
import { Quat, slerp, fromAxisAngle, multiply, rotateVector } from './src/index.js';

// Two orientations: 90 degrees about X and 90 degrees about Y.
const a = fromAxisAngle(1, 0, 0, Math.PI / 2);
const b = fromAxisAngle(0, 1, 0, Math.PI / 2);

// Halfway between them.
const mid = slerp(a, b, 0.5);

// Compose rotations: apply a, then b.
const composed = multiply(b, a);

// Rotate a vector.
const v = rotateVector(a, 0, 1, 0); // { x: 0, y: 0, z: 1 }
```

## Why this exists

Interpolating Euler angles between two orientations is ambiguous: the same
rotation has many Euler representations, and picking the wrong one makes the
object spin the long way around.  Unit quaternions remove that ambiguity, but
the naive linear blend of two quaternions is not a rotation — it shrinks
toward the origin and then has to be renormalized, which changes the angular
speed.  `slerp` walks the great-circle arc on the 4D unit sphere at constant
angular velocity, which is what animation and simulation actually want.

The trade-off: this library uses the shortest-arc convention.  When the dot
product between the two input quaternions is negative, one endpoint is
negated before interpolation.  That gives the shortest path, which is the
right default for animation, but it means `slerp(a, b, t)` and
`slerp(a, -b, t)` return the same rotation at every `t`.  If you need to
force a particular long path, negate one endpoint yourself before calling.

The awkward edge you will hit: when the two quaternions are nearly parallel,
the standard slerp formula divides by `sin(theta)`, which underflows.  This
implementation detects that case and falls back to normalized linear
interpolation, which is exact in the limit.  The quaternions are not
re-normalized on the way out of the general branch, so repeated composition
without an occasional `normalize` can drift; the helpers here assume you
keep your inputs unit length.

## API

- `Quat` — class with fields `w`, `x`, `y`, `z`, plus `toArray()` and `clone()`.
- `normalize(q)` — returns a unit quaternion; a zero input yields the identity.
- `dot(a, b)` — scalar dot product.
- `multiply(a, b)` — Hamilton product, i.e. compose rotations.
- `conjugate(q)` — inverse rotation for a unit quaternion.
- `slerp(a, b, t)` — shortest-arc spherical interpolation.
- `angleBetween(a, b)` — angle in radians in `[0, PI]`.
- `fromAxisAngle(ax, ay, az, angle)` — build a rotation from an axis and angle.
- `toAxisAngle(q)` — returns `{ ax, ay, az, angle }`; identity gives angle 0.
- `rotateVector(q, vx, vy, vz)` — returns `{ x, y, z }`.
