package internal

// The basic simulation object in the physics core. It represents a solid object with position, orientation
// and velocity that responds to forces and torques according to Newtonian mechanics.
type RigidBody struct {
	// Holds the inverse of the mass of the rigid body.
	// It is more useful to hold the inverse mass because integration is simpler, and because in real-time
	// simulation it is more useful to have bodies with infinite mass (immovable, inverseMass=0) than zero
	// mass (completely unstable in numerical simulation).
	InverseMass float32
	// Holds the amount of damping applied to linear motion. Damping is required to remove energy added through
	// numerical instability in the integrator, and can also simulate drag or other resistance forces.
	LinearDamping float32
	// Holds the linear position of the rigid body in world space. This represents the locations of the center
	// of mass of the body.
	Position *Vector
	// Holds the angular orientation of the rigid body in world space. This represents the rate of rotation of
	// the body from its reference orientation.
	Orientation *Quaternion
	// Holds the linear velocity of the rigid body in world space. This is the rate of change of position per unit time.
	Velocity *Vector
	// Holds the angular velocity of the rigid body in world space. This represents the rate of rotation around each
	// axis in radians per unit time.
	Rotation *Vector
	// Holds a transform matrix for converting body space into world space and vice-versa. This matrix combines the
	// position and orientation information into a single transformation that can be applied to points and vectors.
	TransformMatrix *Matrix4
	// Holds the inverse of the body's inertia tensor. The inertia tensor provided must not be degenerate (that would
	// mean the body had zero inertia for spinning along one axis). As long as the tensor is finite, it will be invertible.
	// The inverse tensor is used for similar reasons to the use of inverse mass.
	inverseInertiaTensor *Matrix3
	// Holds the accumulated force to be applied at the next integration step.
	ForceAccum *Vector
	// Holds the accumulated torque to be applied at the next integration step.
	TorqueAccum *Vector
	// A body can be put to sleep to avoid being updated by the integration functions or affected by collisions by the world.
	IsAwake bool
	// Holds the amount of damping applied to angular motion. Damping is required to remove energy added through numerical
	// instability in the integrator.
	AngularDamping Real
	// Holds the linear acceleration of the rigid body for the previous frame.
	LastFrameAcceleration *Vector
	// Holds the acceleration of the rigid body. This value can be used to set acceleration due to gravity or any constant acceleration.
	Acceleration *Vector
	// Holds the inverse inertia tensor of the body in world space.
	InverseInertiaTensorWorld *Matrix3
	// Holds the amount of motion of the body. This is a recency weighted mean that can be used to put a body to sleep.
	Motion Real
	// Some bodies may never be allowed to fall asleep. User controlled bodies, for example, should be always awake.
	CanSleep bool
}

// Updates internal data from the state data.
//
// This should be called after the body's state is altered directly (it is called automatically during integration).
// If you change the body's state and then intend to integrate before querying any data (such as the transform matrix),
// then you can omit this step.
//
// Currently this method:
// 1. Normalizes the orientation quaternion to ensure it represents a valid rotation.
// 2. Updates the transform matrix based on current position and orientation.
func (rb *RigidBody) CalculateDerivedData() {
	// Ensure the orientation quaternionn is normalized
	rb.Orientation.Normalize()

	calculateTransformMatrix(rb.TransformMatrix, rb.Position, rb.Orientation)
}

// Creates a transform matrix from a position and orientation.
// Builds a 4*4 transformation matrix that combines the rotation represented by the quaternion and
// the translation represented by the position.
// The resulting matrix can transform points from body space to world space.
func calculateTransformMatrix(transformMatrix *Matrix4, position *Vector, orientation *Quaternion) {
	// Set the rotation components
	transformMatrix.Data[0] = 1 - 2*orientation.J*orientation.J - 2*orientation.K*orientation.K
	transformMatrix.Data[1] = 2*orientation.I*orientation.J - 2*orientation.R*orientation.K
	transformMatrix.Data[2] = 2*orientation.I*orientation.K + 2*orientation.R*orientation.J

	transformMatrix.Data[4] = 2*orientation.I*orientation.J + 2*orientation.R*orientation.K
	transformMatrix.Data[5] = 1 - 2*orientation.I*orientation.I - 2*orientation.K*orientation.K
	transformMatrix.Data[6] = 2*orientation.J*orientation.K - 2*orientation.R*orientation.I

	transformMatrix.Data[8] = 2*orientation.I*orientation.K - 2*orientation.R*orientation.J
	transformMatrix.Data[9] = 2*orientation.J*orientation.K + 2*orientation.R*orientation.I
	transformMatrix.Data[10] = 1 - 2*orientation.I*orientation.I - 2*orientation.J*orientation.J

	// Set the translation components
	transformMatrix.Data[3] = float32(position.X)
	transformMatrix.Data[7] = float32(position.Y)
	transformMatrix.Data[11] = float32(position.Z)
}

func (rb *RigidBody) SetInertiaTensor(inertiaTensor *Matrix3) {
	rb.inverseInertiaTensor.SetInverse(inertiaTensor)
}

// Transforms an inertia tensor from body space to world space.
//
// The inertia tensor is initially defined in body space (local coordinates), where it remains contant regardless of the
// object's orientation. However, for physics calculations, we need the inertia tensor in world coordinates to properly
// calculate rotational dynamics.
//
// When a rigid body rotates, its resistance to angular acceleration (inertia) changes relative to the world axes. for e.g,
// a rod has different moments of inertia when spinning along its length versus spinning perpendicular to its length. As the
// rod rotates in space, these different moments of inertia need to be correctly mapped to the world coordinate axes.
//
// The implementation uses an optimized matrix multiplication sequence. The Formula is essentially:
// I_world = R * I_body * R^T
// where:
// I_world is the inertia tensor in world space
// I_body is the inertia tensor in body space
// R is the rotation matrix
// R^T is the transpose of the rotation matrix
//
// By updating this world-space inertia tensor each frame, we can correctly calculate how the object will rotate in response
// to torques applied in world coordinates, without having to recalculate the inertia tensor from scratch by summing masses.
//
// Parameters:
// - iitWorld: Output parameter for the transformed inertia tensor in world space.
// - q: The orientation quaternion
// - iitBody: The inertia tensor in body space
// - rotmat: The rotation matrix derived from the orientation
func transformInertiaTensor(iitWorld *Matrix3, q *Quaternion, iitBody *Matrix3, rotmat *Matrix4) {
	// First part: Calculate intermediate values representing I_body * R^T

	// Row 0 of I_body * R^T
	// First row of rotation matrix * first column of inertia tensor in body space
	t4 := rotmat.Data[0]*iitBody.Data[0] + // R[0,0] * I[0,0]
		rotmat.Data[1]*iitBody.Data[3] + // R[0,1] * I[1,0]
		rotmat.Data[2]*iitBody.Data[6] // R[0,2] * I[2,0]

	// First row of rotation matrix * second column of inertia tensor in body space
	t9 := rotmat.Data[0]*iitBody.Data[1] + // R[0,0] * I[0,1]
		rotmat.Data[1]*iitBody.Data[4] + // R[0,1] * I[1,1]
		rotmat.Data[2]*iitBody.Data[7] // R[0,2] * I[2,1]

	// First row of rotation matrix * third column of inertia tensor in body space
	t14 := rotmat.Data[0]*iitBody.Data[2] + // R[0,0] * I[0,2]
		rotmat.Data[1]*iitBody.Data[5] + // R[0,1] * I[1,2]
		rotmat.Data[2]*iitBody.Data[8] // R[0,2] * [2,2]

	// Row 1 of I_body * R^T
	// Second row of rotation matrix * first column of inertia tensor in body space
	t28 := rotmat.Data[4]*iitBody.Data[0] + // R[1,0] * I[0,0]
		rotmat.Data[5]*iitBody.Data[3] + // R[1,1] * I[1,0]
		rotmat.Data[6]*iitBody.Data[6] // R[1,2] * I[2,0]

	// Second row of rotation matrix * second column of inertia tensor in body space
	t33 := rotmat.Data[4]*iitBody.Data[1] + // R[1,0] * I[0,1]
		rotmat.Data[5]*iitBody.Data[4] + // R[1,1] * I[1,1]
		rotmat.Data[6]*iitBody.Data[7] // R[1,2] * I[2,1]

	// Second row of rotation matrix * third column of inertia tensor in body space
	t38 := rotmat.Data[4]*iitBody.Data[2] + // R[1,0] * I[0,2]
		rotmat.Data[5]*iitBody.Data[5] + // R[1,1] * I[1,2]
		rotmat.Data[6]*iitBody.Data[8] // R[1,2] * [2,2]

	// Row 2 of I_body * R^T
	// Third row of rotation matrix * first column of inertia tensor in body space
	t52 := rotmat.Data[8]*iitBody.Data[0] + // R[2,0] * I[0,0]
		rotmat.Data[9]*iitBody.Data[3] + // R[2,1] * I[1,0]
		rotmat.Data[10]*iitBody.Data[6] // R[2,2] * I[2,0]

	// Third row of rotation matrix * second column of inertia tensor in body space
	t57 := rotmat.Data[8]*iitBody.Data[1] + // R[2,0] * I[0,1]
		rotmat.Data[9]*iitBody.Data[4] + // R[2,1] * I[1,1]
		rotmat.Data[10]*iitBody.Data[7] // R[2,2] * I[2,1]

	// Third row of rotation matrix * third column of inertia tensor in body space
	t62 := rotmat.Data[8]*iitBody.Data[2] + // R[2,0] * I[0,2]
		rotmat.Data[9]*iitBody.Data[5] + // R[2,1] * I[1,2]
		rotmat.Data[10]*iitBody.Data[8] // R[2,2] * [2,2]

	// Second part: Calculate R * (I_body * R^T) to get final world space tensor

	// Row 0 of the world space inertia tensor

	// Dot product of temp row 0 with rotation matrix row 0
	iitWorld.Data[0] = t4*rotmat.Data[0] + t9*rotmat.Data[1] + t14*rotmat.Data[2]

	// Dot product of temp row 0 with rotation matrix row 1
	iitWorld.Data[1] = t4*rotmat.Data[4] + t9*rotmat.Data[5] + t14*rotmat.Data[6]

	// Dot product of temp row 0 with rotation matrix row 2
	iitWorld.Data[2] = t4*rotmat.Data[8] + t9*rotmat.Data[9] + t9*rotmat.Data[10]

	// Row 1 of the world space inertia tensor

	// Dot product of temp row 1 with rotation matrix row 0
	iitWorld.Data[3] = t28*rotmat.Data[0] + t33*rotmat.Data[1] + t38*rotmat.Data[2]

	// Dot product of temp row 1 with rotation matrix row 1
	iitWorld.Data[4] = t28*rotmat.Data[4] + t33*rotmat.Data[5] + t38*rotmat.Data[6]

	// Dot product of temp row 1 with rotation matrix row 2
	iitWorld.Data[5] = t28*rotmat.Data[8] + t33*rotmat.Data[9] + t38*rotmat.Data[10]

	// Row 2 of the world space inertia tensor

	// Dot product of temp row 2 with rotation matrix row 0
	iitWorld.Data[6] = t52*rotmat.Data[0] + t57*rotmat.Data[1] + t62*rotmat.Data[2]

	// Dot product of temp row 2 with rotation matrix row 1
	iitWorld.Data[7] = t52*rotmat.Data[4] + t57*rotmat.Data[5] + t62*rotmat.Data[6]

	// Dot product of temp row 2 with rotation matrix row 2
	iitWorld.Data[8] = t52*rotmat.Data[8] + t57*rotmat.Data[9] + t62*rotmat.Data[10]
}

// Adds the given force to center of mass of the rigid body. The force is expressed in world coordinates.
//
// This method only affects linear motion (translation) since forces applied at the center of mass generate no torque.
// This is useful for forces like gravity that act uniformly on the entire body.
//
// Based on D'Alembert's principle, we can accumulate multiple forces by simply adding them together. The effect of
// the accumulated force is identical to applying all the individual forces seperately.
func (rb *RigidBody) AddForce(force *Vector) {
	rb.ForceAccum = rb.ForceAccum.Add(force)
	rb.IsAwake = true
}

// Resets the force and torque accumulators to zero.
//
// This is called automatically after each integration step to ensure that forces don't accumulate b/w frames. Each frame should
// calculate and apply its own forces based on the current state of the simulation.
func (rb *RigidBody) ClearAccumulators() {
	rb.ForceAccum.Clear()
	rb.TorqueAccum.Clear()
}

// Adds the given force to the given point on the rigid body. Both the force and the application point are given in world space.
//
// When a force is applied away from the center of mass, it generates both a linear force and a torque(rotational force). According
// to D'Alembert's principle, we accumulate these seperately:
// 1. The full force is added to the force seperately.
// 2. The torque (calculated as point-relative-to-CM * force) is added to the torque accumulator.
//
// Note that two forces might cancel each other out as linear forces (sum to zero), but still generate significant torque if applied at
// different points (like fingers turning a dial)
func (rb *RigidBody) AddForceAtPoint(force *Vector, point *Vector) {
	// Convert to coordinates relative to center of mass
	pt := point.Subtract(rb.Position)

	// Add the force to force accumulator
	rb.ForceAccum = rb.ForceAccum.Add(force)

	// Calculate and add the torque (cross product of position and force)
	rb.TorqueAccum = rb.TorqueAccum.Add(pt.Cross(force))

	rb.IsAwake = true
}

// Adds the given force to the given point on the rigid body. The direction of the force is given in world coordinates, but the
// application point is given in body space.
//
// This is particularly useful for springs or other forces that are attached to a fixed point on the body. Since objects rotate,
// a point fixed in body space will change its world coordinates each frame. This method handles the conversion from body space
// to world space before calculating the resulting force and torque.
func (rb *RigidBody) AddForceAtBodyPoint(force *Vector, point *Vector) {
	// Convert body-space point to world-space point
	pt := rb.GetPointInWorldSpace(point)

	// Now add the force at this world point
	rb.AddForceAtPoint(force, pt)

	rb.IsAwake = true
}

func (rb *RigidBody) GetPointInWorldSpace(point *Vector) *Vector {
	return rb.TransformMatrix.Transform(point)
}

func (rb *RigidBody) HasFiniteMass() bool {
	return rb.InverseMass >= 0
}

func (rb *RigidBody) GetMass() Real {
	if rb.InverseMass == 0 {
		return MaxReal
	} else {
		return Real(1.0 / rb.InverseMass)
	}
}

func (rb *RigidBody) GetInverseMass() Real {
	return Real(rb.InverseMass)
}

func (rb *RigidBody) GetInverseInertiaTensorWorld() *Matrix3 {
	return rb.InverseInertiaTensorWorld
}

func (rb *RigidBody) GetInverseInertiaTensorWorldPtr(inverseInertiaTensor *Matrix3) {
	*inverseInertiaTensor = *rb.InverseInertiaTensorWorld
}

// Updates the rigid body's state over time.
//
// Performs the numerical integration of the equations of motion for a rigid body, advancing its position and orientation based
// on the forces and torques that have been applied. The implementation uses a semi-implicit Euler integration scheme, which
// provides a good balance b/w simplicity and stability for most real-time physics simulations.
//
// The integration process:
// 1. Calculates linear and angular accelerations from accumulated forces and torques
// 2. Updates linear and angular velocities based on these accelerations
// 3. Applies damping to both linear and angular velocities
// 4. Updates position and orientation based on the new velocities
// 5. Recalculates derived data (normalizing orientation, updating  transform matrices)
// 6. Clears force and torque accumulators for the next frame
func (rb *RigidBody) Integrate(duration float32) {
	if !rb.IsAwake {
		return
	}
	// Calculate linear acceleration from force inputs
	// a = F/m (Newton's second law)
	rb.LastFrameAcceleration = rb.Acceleration
	rb.LastFrameAcceleration.AddScaledVector(rb.ForceAccum, Real(rb.InverseMass))

	// Calculate angular acceleration from torque inputs, we use the inverse inertia tensor in the world space to transform the torque
	// α = I⁻¹τ (Rotational analog of Newton's Second Law)
	angularAcceleration := rb.InverseInertiaTensorWorld.Transform(rb.TorqueAccum)

	// Adjust velocities

	// Update linear velocity: v = v₀ + at
	rb.Velocity.AddScaledVector(rb.LastFrameAcceleration, Real(duration))

	// Update angular velocity, ω = ω₀ + αt
	rb.Rotation.AddScaledVector(angularAcceleration, Real(duration))

	// Impose Drag(damping)
	// This simulates the loss of energy due to various resistance forces. The exponential form (d^t) ensures physically correct behavior
	// across different time steps, where d is the damping coefficient(0-1).
	rb.Velocity = rb.Velocity.Scale(Pow(Real(rb.LinearDamping), Real(duration)))
	rb.Rotation = rb.Rotation.Scale(Pow(rb.AngularDamping, Real(duration)))

	// Adjust positions

	// Update linear position: p = p₀ + vt
	rb.Position.AddScaledVector(rb.Velocity, Real(duration))

	// Update angular position (orientation)
	// We use the angular velocity to update the orientation quaternion
	rb.Orientation.AddScaledVector(rb.Rotation, duration)

	// Normalize the orientation and update derived data
	// This ensures the orientation quaternion remains a valid rotation and updates the transformation matrics based on new position/orientation
	rb.CalculateDerivedData()

	// Clear accumulators for next frame. Forces and torques don't accumulate b/w frames - they are recalculated on each frame
	rb.ClearAccumulators()
}

func (rb *RigidBody) GetVelocity() *Vector {
	return rb.Velocity
}

func (rb *RigidBody) GetTransform() *Matrix4 {
	return rb.TransformMatrix
}

func (rb *RigidBody) SetMass(mass Real) {
	if mass == 0 {
		return
	}

	rb.InverseMass = float32(1.0 / mass)
}

func (rb *RigidBody) SetDamping(linearDamping Real, angularDamping Real) {
	rb.LinearDamping = float32(linearDamping)
	rb.AngularDamping = angularDamping
}

func (rb *RigidBody) SetAcceleration(acceleration *Vector) {
	rb.Acceleration = acceleration
}

// Sets the awake state of the body.
func (rb *RigidBody) SetAwake(awake bool) {
	if awake {
		rb.IsAwake = true

		// Add a bit of motion to avoid it falling asleep immediately
		rb.Motion = SleepEpsilon * 2.0
	} else {
		rb.IsAwake = false
		rb.Velocity.Clear()
		rb.Rotation.Clear()
	}
}

// Sets whether the body is ever allowed to go to sleep. Bodies under the player's control, or for which the set of transient forces
// applied each frame are not predictable, should be kept awake.
func (rb *RigidBody) SetCanSleep(canSleep bool) {
	rb.CanSleep = canSleep

	if !rb.CanSleep && !rb.IsAwake {
		rb.SetAwake(true)
	}
}

// Sets the position of the rigid body.
func (rb *RigidBody) SetPosition(x Real, y Real, z Real) {
	rb.Position.X = x
	rb.Position.Y = y
	rb.Position.Z = z
}

func (rb *RigidBody) SetPositionVector(position *Vector) {
	rb.Position = position
}

func (rb *RigidBody) SetOrientation(r, i, j, k Real) {
	rb.Orientation.R = float32(r)
	rb.Orientation.I = float32(i)
	rb.Orientation.J = float32(j)
	rb.Orientation.K = float32(k)

	rb.Orientation.Normalize()
}

func (rb *RigidBody) GetAwake() bool {
	return rb.IsAwake
}
func (rb *RigidBody) GetRotation() *Vector {
	return rb.Rotation
}

func (rb *RigidBody) GetRotationPtr(rotation *Vector) {
	*rotation = *rb.Rotation
}

func (rb *RigidBody) GetLastFrameAcceleration() *Vector {
	return rb.LastFrameAcceleration
}

func (rb *RigidBody) GetLastFrameAccelerationPtr(acc *Vector) {
	*acc = *rb.LastFrameAcceleration
}

func (rb *RigidBody) GetOrientation(q *Quaternion) {
	*q = *rb.Orientation
}

func (rb *RigidBody) SetOrientationQua(orientation *Quaternion) {
	rb.Orientation = orientation
	rb.Orientation.Normalize()
}

func (rb *RigidBody) SetVelocity(x, y, z Real) {
	rb.Velocity.X = x
	rb.Velocity.Y = y
	rb.Velocity.Z = z
}

func (rb *RigidBody) SetRotation(x, y, z Real) {
	rb.Rotation.X = x
	rb.Rotation.Y = y
	rb.Rotation.Z = z
}

func (rb *RigidBody) GetPosition() *Vector {
	return rb.Position
}

func (rb *RigidBody) GetPositionPtr(position *Vector) {
	*position = *rb.Position
}
