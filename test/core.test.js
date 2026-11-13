import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  Quat,
  slerp,
  normalize,
  multiply,
  conjugate,
  dot,
  angleBetween,
  fromAxisAngle,
  toAxisAngle,
  rotateVector,
} from '../src/index.js';

const EPS = 1e-9;

function assertQuatClose(got, want, msg) {
  assert.ok(Math.abs(got.w - want.w) < EPS, `${msg}: w ${got.w} vs ${want.w}`);
  assert.ok(Math.abs(got.x - want.x) < EPS, `${msg}: x ${got.x} vs ${want.x}`);
  assert.ok(Math.abs(got.y - want.y) < EPS, `${msg}: y ${got.y} vs ${want.y}`);
  assert.ok(Math.abs(got.z - want.z) < EPS, `${msg}: z ${got.z} vs ${want.z}`);
}

const identity = new Quat(1, 0, 0, 0);
const q90x = fromAxisAngle(1, 0, 0, Math.PI / 2);
const q90y = fromAxisAngle(0, 1, 0, Math.PI / 2);

test('normalize returns identity for zero quaternion', () => {
  assertQuatClose(normalize(new Quat(0, 0, 0, 0)), identity, 'zero');
});

test('normalize scales to unit length', () => {
  const q = normalize(new Quat(2, 0, 0, 0));
  assertQuatClose(q, new Quat(1, 0, 0, 0), 'scaled');
});

test('dot of orthogonal quaternions is zero', () => {
  assert.ok(Math.abs(dot(identity, new Quat(0, 1, 0, 0))) < EPS);
});

test('multiply composes rotations', () => {
  // 90 about x then 90 about y equals a known quaternion up to floating
  // point error and overall sign (q and -q represent the same rotation).
  const q = multiply(q90y, q90x);
  const expected = new Quat(0.5, 0.5, 0.5, -0.5);
  assertQuatClose(q, expected, 'compose');
});

test('conjugate inverts a rotation', () => {
  const q = multiply(q90x, conjugate(q90x));
  assertQuatClose(q, identity, 'inverse');
});

test('slerp at t=0 and t=1 returns endpoints', () => {
  assertQuatClose(slerp(q90x, q90y, 0), q90x, 't=0');
  assertQuatClose(slerp(q90x, q90y, 1), q90y, 't=1');
});

test('slerp midpoint is halfway in angle', () => {
  const mid = slerp(identity, q90x, 0.5);
  const expected = fromAxisAngle(1, 0, 0, Math.PI / 4);
  assertQuatClose(mid, expected, 'midpoint');
});

test('slerp takes the shortest path for opposite-sign quaternions', () => {
  // q and -q represent the same rotation; slerp must not go the long way.
  const a = new Quat(1, 0, 0, 0);
  const b = new Quat(-1, 0, 0, 0);
  const mid = slerp(a, b, 0.5);
  assertQuatClose(mid, identity, 'shortest');
});

test('slerp handles nearly parallel quaternions without NaN', () => {
  const a = fromAxisAngle(1, 0, 0, 0.1);
  const b = fromAxisAngle(1, 0, 0, 0.1 + 1e-12);
  const mid = slerp(a, b, 0.5);
  assert.ok(Number.isFinite(mid.w) && Number.isFinite(mid.x));
  assertQuatClose(mid, a, 'nearly parallel');
});

test('angleBetween is zero for identical rotations', () => {
  assert.ok(angleBetween(q90x, q90x) < EPS);
});

test('angleBetween is PI for opposite rotations', () => {
  assert.ok(Math.abs(angleBetween(identity, new Quat(0, 1, 0, 0)) - Math.PI) < EPS);
});

test('fromAxisAngle normalizes the axis', () => {
  const q = fromAxisAngle(0, 0, 2, Math.PI / 2);
  const expected = fromAxisAngle(0, 0, 1, Math.PI / 2);
  assertQuatClose(q, expected, 'axis length');
});

test('fromAxisAngle with zero axis returns identity', () => {
  assertQuatClose(fromAxisAngle(0, 0, 0, 1.23), identity, 'zero axis');
});

test('toAxisAngle round-trips a known rotation', () => {
  const { ax, ay, az, angle } = toAxisAngle(q90x);
  assert.ok(Math.abs(ax - 1) < EPS);
  assert.ok(Math.abs(ay) < EPS);
  assert.ok(Math.abs(az) < EPS);
  assert.ok(Math.abs(angle - Math.PI / 2) < EPS);
});

test('toAxisAngle on identity returns zero angle', () => {
  const { angle } = toAxisAngle(identity);
  assert.ok(Math.abs(angle) < EPS);
});

test('rotateVector rotates a basis vector', () => {
  const v = rotateVector(q90x, 0, 1, 0);
  assert.ok(Math.abs(v.x) < EPS);
  assert.ok(Math.abs(v.y) < EPS);
  assert.ok(Math.abs(v.z - 1) < EPS);
});

test('rotateVector with identity leaves vector unchanged', () => {
  const v = rotateVector(identity, 1, 2, 3);
  assert.ok(Math.abs(v.x - 1) < EPS);
  assert.ok(Math.abs(v.y - 2) < EPS);
  assert.ok(Math.abs(v.z - 3) < EPS);
});
