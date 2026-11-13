const EPS = 1e-12;

/**
 * A unit quaternion representing a 3D rotation.
 * Stored as [w, x, y, z] where w is the scalar part.
 * The constructor does not enforce unit length; call normalize() before use
 * if the input may be non-unit.  Most operations assume unit quaternions.
 */
export class Quat {
  /**
   * @param {number} w scalar part
   * @param {number} x
   * @param {number} y
   * @param {number} z
   */
  constructor(w, x, y, z) {
    this.w = w;
    this.x = x;
    this.y = y;
    this.z = z;
  }

  /** @returns {number[]} */
  toArray() {
    return [this.w, this.x, this.y, this.z];
  }

  /** @returns {Quat} */
  clone() {
    return new Quat(this.w, this.x, this.y, this.z);
  }
}

/**
 * Normalize a quaternion to unit length.
 * If the norm is below EPS, returns the identity quaternion rather than
 * dividing by zero.  This is the only sensible behaviour for a rotation
 * library: a zero quaternion has no orientation, but returning NaN would
 * poison downstream math.
 * @param {Quat} q
 * @returns {Quat}
 */
export function normalize(q) {
  const n = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < EPS) return new Quat(1, 0, 0, 0);
  return new Quat(q.w / n, q.x / n, q.y / n, q.z / n);
}

/**
 * Dot product of two quaternions.
 * @param {Quat} a
 * @param {Quat} b
 * @returns {number}
 */
export function dot(a, b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * Hamilton product of two quaternions (composition of rotations).
 * When a and b are unit quaternions, the result is unit up to floating
 * point error; callers that need exact unit length should call normalize.
 * @param {Quat} a
 * @param {Quat} b
 * @returns {Quat}
 */
export function multiply(a, b) {
  return new Quat(
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  );
}

/**
 * Conjugate of a quaternion.  For a unit quaternion this is the inverse
 * rotation, which is why the library does not expose a separate inverse().
 * @param {Quat} q
 * @returns {Quat}
 */
export function conjugate(q) {
  return new Quat(q.w, -q.x, -q.y, -q.z);
}

/**
 * Spherical linear interpolation between two unit quaternions.
 *
 * The shortest-arc convention is used: if the dot product is negative,
 * one endpoint is negated before interpolation so the path never takes the
 * long way around the 4D sphere.  This is the standard choice for animation
 * and is stated in the README.
 *
 * @param {Quat} a start unit quaternion
 * @param {Quat} b end unit quaternion
 * @param {number} t interpolation parameter, typically in [0, 1]
 * @returns {Quat}
 */
export function slerp(a, b, t) {
  let cosHalfTheta = dot(a, b);
  let bx = b.x, by = b.y, bz = b.z, bw = b.w;

  if (cosHalfTheta < 0) {
    cosHalfTheta = -cosHalfTheta;
    bx = -bx; by = -by; bz = -bz; bw = -bw;
  }

  // When the quaternions are nearly parallel, sin(theta) is tiny and the
  // standard formula is numerically unstable.  Linear interpolation followed
  // by normalization is exact in the limit and avoids the division by zero.
  if (cosHalfTheta > 1 - 1e-9) {
    const w = a.w + t * (bw - a.w);
    const x = a.x + t * (bx - a.x);
    const y = a.y + t * (by - a.y);
    const z = a.z + t * (bz - a.z);
    return normalize(new Quat(w, x, y, z));
  }

  const halfTheta = Math.acos(cosHalfTheta);
  const sinHalfTheta = Math.sqrt(1 - cosHalfTheta * cosHalfTheta);
  const wa = Math.sin((1 - t) * halfTheta) / sinHalfTheta;
  const wb = Math.sin(t * halfTheta) / sinHalfTheta;

  return new Quat(
    wa * a.w + wb * bw,
    wa * a.x + wb * bx,
    wa * a.y + wb * by,
    wa * a.z + wb * bz
  );
}

/**
 * Angle in radians between two rotations, in [0, PI].
 * Uses the absolute dot product so q and -q (which represent the same
 * rotation) give the same angle.
 * @param {Quat} a
 * @param {Quat} b
 * @returns {number}
 */
export function angleBetween(a, b) {
  const d = Math.abs(dot(a, b));
  const clamped = Math.min(1, Math.max(-1, d));
  return 2 * Math.acos(clamped);
}

/**
 * Build a unit quaternion from an axis and an angle in radians.
 * The axis is normalized internally; a zero axis yields the identity
 * quaternion because no rotation direction is defined.
 * @param {number} ax
 * @param {number} ay
 * @param {number} az
 * @param {number} angle
 * @returns {Quat}
 */
export function fromAxisAngle(ax, ay, az, angle) {
  const len = Math.sqrt(ax * ax + ay * ay + az * az);
  if (len < EPS) return new Quat(1, 0, 0, 0);
  const half = angle / 2;
  const s = Math.sin(half) / len;
  return new Quat(Math.cos(half), ax * s, ay * s, az * s);
}

/**
 * Extract axis and angle from a unit quaternion.
 * The returned axis is unit length.  For the identity rotation the angle is
 * zero and the axis is chosen as (1, 0, 0) so the return type stays uniform;
 * callers should treat a zero angle as "no rotation" regardless of axis.
 * @param {Quat} q
 * @returns {{ax: number, ay: number, az: number, angle: number}}
 */
export function toAxisAngle(q) {
  const w = Math.min(1, Math.max(-1, q.w));
  const angle = 2 * Math.acos(w);
  const s = Math.sqrt(1 - w * w);
  if (s < EPS) return { ax: 1, ay: 0, az: 0, angle: 0 };
  return { ax: q.x / s, ay: q.y / s, az: q.z / s, angle };
}

/**
 * Rotate a vector by a unit quaternion.
 * Uses the optimized q * v * q^-1 formula, which avoids constructing an
 * intermediate quaternion for the vector.
 * @param {Quat} q unit quaternion
 * @param {number} vx
 * @param {number} vy
 * @param {number} vz
 * @returns {{x: number, y: number, z: number}}
 */
export function rotateVector(q, vx, vy, vz) {
  const { w, x, y, z } = q;
  // t = 2 * cross(q.xyz, v)
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  // v' = v + w * t + cross(q.xyz, t)
  return {
    x: vx + w * tx + (y * tz - z * ty),
    y: vy + w * ty + (z * tx - x * tz),
    z: vz + w * tz + (x * ty - y * tx),
  };
}
