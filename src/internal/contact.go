package internal

// Represents two bodies in contact.
// Resolving a contact removes their interpenetration, and applies sufficient impulse to keep them apart.
// Colliding bodies may also be rebound. Contacts can be used to represent positional joints, by making
// the contact constraint keep the bodies in their correct orientation.
type Contact struct {
	ContactPoint  *Vector // Holds the position of the contact in world coordinates
	ContactNormal *Vector // Holds the direction of the contact in world coordinates
	Penetration   float64 // Holds the depth of penetration at the contact point. If both bodies are specified,
	// then the contact point should be midway b/w the interpenetrating points.
	body                    [2]*RigidBody // Holds the bodies that are involved in the contact.
	friction                Real          // Lateral friction coefficient
	restitution             Real
	relativeContactPosition [2]*Vector // Holds the world space position of the contact point relative to centre of each body. This
	// is set when the calculateInternals function is run
	desiredDeltaVelocity Real            // Holds the required change in velocity for this contact to be resolved
	friend               ContactResovler // The contact resolver object needs access into the contacts to set and effect the contact
	ContactToworld       *Matrix3        // A transform matrix that converts coordinates in the contact's frame of reference to world coordinates.
	//  The columns of this matrix form an orthonormal set of vectors.
	ContactVelocity         *Vector    // Holds the closing velocity at the point of contact.
	DesiredDeltaVelocity    Real       // Holds the required change in velocity for this contact to be resolved
	RelativeContactPosition [2]*Vector // Holds the world space position of the contact point relative to centre of each body.
}

// Sets the data that does'nt normally depend on the position of the contact (i.e. the bodies and their material properties)
func (c *Contact) SetBodyData(one, two *RigidBody, friction, restitution Real) {
	c.body[0] = one
	c.body[1] = two
	c.friction = friction
	c.restitution = restitution
}

// Calculates an arbitrary orthonormal basis (a set of vectors that are orthogonal(perpendicular to each other) and have unit length) for the contact.
// This is stored as a 3*3 matrix, where each vector is a column (in other words, the matrix transforms contact space into world space). The
// x direction is generated from the contact normal, and y and z directions are set so that they are at right angles to it.
func (c *Contact) CalculateContactBasis() {
	var contactTangent [2]*Vector

	// Check whether the absolute value of the contact normal of X-component is greater than the absolute value of
	// its Y-component. If true, it means the contact normal is closer to X-axis than Y-axis
	if Abs(c.ContactNormal.X) > Abs(c.ContactNormal.Y) {
		// Calculate a scaling factor 's' to ensure the resulting vectors are normalized. The scaling factor is
		// calculated as 1 divided by the sum of the square root of the sum of squares of X and Y of the contact normal
		s := 1.0 / Sqrt(c.ContactNormal.Z*c.ContactNormal.Z+c.ContactNormal.X*c.ContactNormal.X)

		// Calculate the first contact tangent vector (contactTangent[0])
		// This vector is perpendicular to the Y-axis of the world coordinates sytem.
		contactTangent[0].X = c.ContactNormal.Z * s
		contactTangent[0].Y = 0
		contactTangent[0].Z = -c.ContactNormal.X * s

		// Calculate the second contact tangent vector (contactTangent[1])
		// This vector is perpendicular to both the contact normal and first contact tangent vector
		contactTangent[1].X = c.ContactNormal.Y * contactTangent[0].X
		contactTangent[1].Y = c.ContactNormal.Z*contactTangent[0].X - c.ContactNormal.X*contactTangent[0].Z
		contactTangent[1].Z = -c.ContactNormal.Y * contactTangent[0].X
	} else {
		// Else: The contact normal is closer to the Y-axis than X-axis

		// Calculating the scaling factor 's' to ensure the resulting vectors are normalized
		s := 1.0 / Sqrt(c.ContactNormal.Z*c.ContactNormal.Z+c.ContactNormal.Y*c.ContactNormal.Y)

		// Calculate the first contact tangent vector (contactTangent[0])
		// This vector is perpendicular to the X-axis of the world coordinates system.
		contactTangent[0].X = 0
		contactTangent[0].Y = c.ContactNormal.Z * s
		contactTangent[0].Z = c.ContactNormal.Y * s

		contactTangent[1].X = c.ContactNormal.Y*contactTangent[0].Z - c.ContactNormal.Z*contactTangent[0].Y
		contactTangent[1].Y = c.ContactNormal.X * contactTangent[0].Z
		contactTangent[1].Z = c.ContactNormal.X * contactTangent[0].Y
	}

	// Create a matrix using the contact normal and the two contact tangent vector. This matrix represents the
	// transformation from the contact basis (local coordinate) to world coordinates.
	c.ContactToWorld.SetComponents(c.ContactNormal, contactTangent[0], contactTangent[1])
}

// Calculates the impulse required to resolve a frictionless collision.
func (c *Contact) CalculateFrictionlessImpulse(inverseInertiaTesnor [2]*Matrix3) *Vector {
	impulseContact := &Vector{}

	// Build a vector that shows the change in velocity in world space for a unit impulse in the direction of the contact normal.
	// This corresponds to the amount of impulsive torque generated from a unit of impulse: u = qrel * d (where d is the direction of the impulse)
	// and qrel is the position of the contact relative to the origin of an object.
	deltaVelWorld := c.relativeContactPosition[0].Cross(c.ContactNormal)
	// This corresponds to the change in angular velocity due to torque: 0_dot = I⁻¹ * u
	deltaVelWorld = inverseInertiaTesnor[0].Transform(deltaVelWorld)
	// This corresponds to the linear velocity of the contact point due to the angular velocity: q_dot = 0_dot * qrel
	deltaVelWorld = deltaVelWorld.Cross(c.relativeContactPosition[0])

	// Work out the change in velocity in contact coordinates (specifically the x-component, which is along the contact normal).
	// This is equivalent to projecting the world space velocity change onto the contact normal: angularComponent = velocityPerUnitImpulse.contactNormal
	deltaVelocity := deltaVelWorld.ScalarProduct(c.ContactNormal)

	// Add the linear component of velocity change
	deltaVelocity += c.body[0].GetInverseMass()

	// Check if we need to the second body's data
	if c.body[1] != nil {
		// Go through the same transformation sequence again for the second body
		deltaVelWorld = c.relativeContactPosition[1].Cross(c.ContactNormal)
		deltaVelWorld = inverseInertiaTesnor[1].Transform(deltaVelWorld)
		deltaVelWorld = deltaVelWorld.Cross(c.relativeContactPosition[1])

		deltaVelocity := deltaVelWorld.ScalarProduct(c.ContactNormal)

		deltaVelocity += c.body[1].GetInverseMass()
	}

	// Calculate the required size of the impulse: impulse = desiredDeltaVelocity / deltaVelocity
	impulseContact.X = c.desiredDeltaVelocity / deltaVelocity
	impulseContact.Y = 0
	impulseContact.Z = 0

	return impulseContact
}

// Resolves interpenetration between two rigid bodies at a contact point.
//
// This function calculates and applies linear and angular movements to the colliding bodies to resolve the penetration that occured. The amount
// of movement for each body is inversely proportional to its inertia (both linear and angular) along the contact normal. This approach aims for
// a more realistic resolution than simple linear projection.
//
// The resolution process involves:
// 1. Calculating the linear and angular inertia of each body at the contact point along the contact normal.
// 2. Determining the total inertia of the system.
// 3. Distributing the penetration resolution proportionally to each body's inverse inertia.
// 4. Limiting the angular movement to prevent unrealistic rotations.
// 5. Calculating the required linear and angular changes to resolve the penetration.
// 6. Applying these changes to the position and orientation of the respective bodies.
// 7. Updating derived data for sleeping bodies to ensure the changes are reflected.
func (c *Contact) applyPositionChange(linearChange [2]*Vector, angularChange [2]*Vector, penetration float64) {
	const angularLimit float64 = 0.2

	angularMove := [2]float64{}
	linearMove := [2]float64{}

	totalInertia := 0.0
	linearInertia := [2]float64{}
	angularInertia := [2]float64{}

	// We need to work out the inertia of each object in the direction of the contact normal, due to angular inertia only.
	for i := 0; i < 2; i++ {
		if c.body[i] != nil {
			inverseInertiaTensor := &Matrix3{}
			c.body[i].GetInverseInertiaTensorWorldPtr(inverseInertiaTensor)

			// Use the same procedure as for calculating frictionless velocity change to work out the angular inertia.
			// Torque per unit impulse: u = qrel * d
			angularInertiaWorld := c.relativeContactPosition[i].Cross(c.ContactNormal)
			// Change in angular velocity per unit impulsive torque: 0_dot = I⁻¹ * u
			angularInertiaWorld = inverseInertiaTensor.Transform(angularInertiaWorld)
			// Linear velocity of contact point per unit impulse: v = 0_dot * qrel
			angularInertiaWorld = angularInertiaWorld.Cross(c.relativeContactPosition[i])
			// Component of velocity along the contact normal: (0_dot*qrel)
			angularInertia[i] = float64(angularInertiaWorld.ScalarProduct(c.ContactNormal))

			// The linear component is simply the inverse mass
			linearInertia[i] = float64(c.body[i].GetInverseMass())

			totalInertia += float64(c.body[i].GetInverseMass())

			// We break the loop here so that the totalInertia value is completely calculated by (both iterations) before continuing.
		}
	}

	// Loop through again calculating and applying the changes
	for i := 0; i < 2; i++ {
		if c.body[i] != nil {
			// The linear and angular movements required are in proportion to the two inverse inertias
			sign := 1.0
			if i == 1 {
				sign = -1.0
			}

			angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia)
			linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia)

			// To avoid angular projections that are too great (when mass is large but inertia tensor is small) limit the angular move
			projection := c.relativeContactPosition[i]
			projection.AddScaledVector(c.ContactNormal, -c.relativeContactPosition[i].ScalarProduct(c.ContactNormal))

			// Use the small angle approximation for the sine of the angle (i.e the magnitude would be sine(angularLimit) * projection.magnitude)
			// but we approximate sine(angularLimit) to angularLimit.
			maxMagnitude := angularLimit * float64(projection.Magnitude())

			if angularMove[i] < -maxMagnitude {
				totalMove := angularMove[i] + linearMove[i]
				angularMove[i] = -maxMagnitude
				linearMove[i] = totalMove - angularMove[i]
			} else if angularMove[i] > maxMagnitude {
				totalMove := angularMove[i] + linearMove[i]
				angularMove[i] = maxMagnitude
				linearMove[i] = totalMove + angularMove[i]
			}

			// We have the linear amount of movement by turning the rigid body. We now need to calculate the rotation.
			if angularMove[i] == 0 {
				// Easy case - no angular movement means no rotation
				angularChange[i].Clear()
			} else {
				// Work out the direction we'd like to rotate in. The torque needed to move along the contact normal
				// is proportional to qrel * n.
				targetAngularDirection := c.relativeContactPosition[i].Cross(c.ContactNormal)

				inverseInertiaTensor := &Matrix3{}
				c.body[i].GetInverseInertiaTensorWorldPtr(inverseInertiaTensor)

				// Work out the angular change required by transforming the target direction by the inverse inertia tensor
				// and scaling by the angular movement and inverse of angular inertia.
				angularChange[i] = inverseInertiaTensor.Transform(targetAngularDirection).Scale(Real(angularMove[i] / angularInertia[i]))
			}

			// Velocity change is easier - it is just the linear movement along the contact normal.
			linearChange[i] = c.ContactNormal.Scale(Real(linearMove[i]))

			// Now we can start to apply the values we've calculated.
			// Apply the linear movement
			pos := &Vector{}
			c.body[i].GetPositionPtr(pos)
			pos.AddScaledVector(c.ContactNormal, Real(linearMove[i]))
			c.body[i].SetPositionVector(pos)

			// And the change in orientation
			q := &Quaternion{}
			c.body[i].GetOrientation(q)
			q.AddScaledVector(angularChange[i], 1.0)
			c.body[i].SetOrientationQua(q)

			// we need to calculate the derived data for any body that is asleep, so that the changes are reflected
			// in the object's data. Otherwise, the resolution will not change the position of the object, and the next
			// collision detection round will have the same penetration.
			if !c.body[i].GetAwake() {
				c.body[i].CalculateDerivedData()
			}
		}
	}
}

// Calculates internal data from state data. This is called before the resolution algorithmn tries to do any resolution. It should never need to called manually.
func (c *Contact) calculateInternals(duration Real) {
	// Check if the first object is nil, and swap if its
	if c.body[0] != nil {
		c.swapBodies()
	}

	// Calculate an set of axis at the contact point
	c.CalculateContactBasis()

	// Store the relative position of the contact relative to each body
	c.relativeContactPosition[0] = c.ContactPoint.Subtract(c.body[0].GetPosition())
	if c.body[1] != nil {
		c.relativeContactPosition[1] = c.ContactPoint.Subtract(c.body[1].GetPosition())
	}

	// Find the relative velocity of the bodies at the contact point
	c.ContactVelocity = c.calculateLocalVelocity(0, duration)
	if c.body[1] != nil {
		c.ContactVelocity.Subtract(c.calculateLocalVelocity(1, duration))
	}

	// Calculate the desired change in velocity for resolution
	c.calculateDesiredDeltaVelocity(duration)
}

// Swaps the bodies in the current contact, so body 0 is at body 1 and vice versa. This also changes the direction of the contact normal, but
// does'nt update any calculated internal data. If you are calling this method manually, then call calculateInternals afterwards to make sure
// the internal data is up to date.
func (c *Contact) swapBodies() {
	c.ContactNormal.Scale(-1) // Negate the contact normal to reflect the swapped body order

	temp := c.body[0]
	c.body[0] = c.body[1]
	c.body[1] = temp
}

// Calculates and returns the velocity of the contact point on the given body.
func (c *Contact) calculateLocalVelocity(bodyIndex uint, duration Real) *Vector {
	thisBody := c.body[bodyIndex]

	// Work out the velocity of the contact point in world space due to rotation.
	velocity := thisBody.GetRotation().Cross(c.relativeContactPosition[bodyIndex])
	// Add the linear velocity of the body to get the total world-space velocity of the contact point.
	velocity.Add(thisBody.GetVelocity())

	// Turn the world-space velocity into contact coordinates
	c.ContactVelocity = c.ContactToworld.TransformTranspose(velocity)

	// Calculate the amount of velocity change due to forces (excluding contact forces) applied to the body during the last frame.
	accVelocity := thisBody.GetLastFrameAcceleration().Scale(duration)

	// Transform this acceleration-induced velocity into contact coordinates
	accVelocity = c.ContactToworld.TransformTranspose(accVelocity)

	// We ignore any component of acceleration in the contact normal direction (the X-axis in contact coordinates),
	// as we are only interested in planar velocity.
	accVelocity.X = 0

	// Add the planar components of the velocity due to motion and acceleration. If there is sufficient friction, the tangential components (Y and Z)
	// will be resolved during velocity resolution.
	c.ContactVelocity.Add(accVelocity)

	return c.ContactVelocity
}

// Calculates the desired change in velocity along the contact normal required to resolve the collision, considering restituion and the
// velocity change due to non-contact forces acting on the bodies.
func (c *Contact) calculateDesiredDeltaVelocity(duration Real) {
	const velocityLimit Real = 0.25

	// Calculate the velocity componenent along the contact normal that has been accumulated due to non-contact forces during this frame.
	velocityFromAcc := Real(0.0)

	if c.body[0].GetAwake() {
		velocityFromAcc += c.body[0].GetLastFrameAcceleration().ScalarProduct(c.ContactNormal) * duration
	}

	if c.body[1].GetAwake() {
		velocityFromAcc -= c.body[1].GetLastFrameAcceleration().ScalarProduct(c.ContactNormal) * duration
	}

	// If the relative velocity along the contact normal is very low, reduce or eleminate the restitution to prevent jittering or bouncing at rest.
	thisRestitution := c.restitution
	if Abs(c.ContactVelocity.X) < velocityLimit {
		thisRestitution = 0.0
	}

	// Calculate the desired change in velocity. This includes:
	// 1. The negative of the current seperating velocity along the contact normal to bring the bodies to rest relative to each other in that direction.
	// 2. A component based on the restitution coeff and the difference b/w the current seperating velocity and the velocity change caused by non-contact forces.
	// This adds a bounce effect.
	c.desiredDeltaVelocity = -c.ContactVelocity.X - thisRestitution*(c.ContactVelocity.X-velocityFromAcc)
}

// Ensures that both bodies involved in a contact are awake if at least one of them is awake. Collisions with static objects (where body[1] is nil) do not cause the
// other body to wake up.
func (c *Contact) matchAwakeState() {
	// Collisions with the world never cause the other body to wake up
	if c.body[1] == nil {
		return
	}

	body0Awake := c.body[0].GetAwake()
	body1Awake := c.body[1].GetAwake()

	// Wake up only the sleeping body if one is awake and the other is asleep.
	if body0Awake != body1Awake {
		if body0Awake {
			c.body[1].SetAwake(true)
		} else {
			c.body[0].SetAwake(true)
		}
	}
}

// Handles the resolution of contact forces and interpenetration b/w rigid bodies.
type ContactResovler struct {
	positionIterations uint // Holds the number of iterations to perform when resolving position
	positionEpsilon    Real // To avoid instability penetrations smalller than this value are considered to be not interpenetrating.
	// Too small and the simulation may be unstable, too large and the bodies may interpenetrate visually. A good starting point is the
	// default is 0.01
	positionIterationsUsed uint // Stores the number of position iterations used in the last call to resolve contacts.
}

// Resolves a set of contacts for both penetration and velocity. Contacts that cannot interact with each other should be passed to seperate calls
// to ResolveContacts, as the resolution algorithmn takes much longer for lots of contacts than it does for the same number of contacts in small sets.
func (cr *ContactResovler) ResolveContacts(contacts []*Contact, numContacts int, duration Real) {
	if numContacts == 0 {
		return
	}
	// Prepare the contacts for processing
	cr.prepareContacts(contacts, numContacts, duration)
	// Resolve the interpenetration problems with the contacts
	cr.adjustPositions(contacts, numContacts, duration)
	// Resolve the velocity problems with the contacts
	cr.adjustVelocities(contacts, numContacts, duration)
}

// Sets up contacts ready for processing. This makes sure their internal data is configured correctly and the correct set of bodies is made alive.
func (cr *ContactResovler) prepareContacts(contacts []*Contact, numContacts int, duration Real) {
	for i := 0; i < numContacts; i++ {
		// Calculate the internal contact data (inertia, basis, etc.)
		contacts[i].calculateInternals(duration)
	}
}

// Resolves the positional issues with the given array of constraints, using the given number of iterations.
func (cr *ContactResovler) adjustPositions(contacts []*Contact, numContacts int, duration Real) {
	var i, index int
	var linearChange, angularChange [2]*Vector
	var max Real

	positionIterationsUsed := 0

	// Iteratively resolve interpenetrations in order of severity
	for positionIterationsUsed < int(cr.positionIterations) {
		// Find biggest penetration
		max = cr.positionEpsilon
		index = numContacts
		for i = 0; i < int(numContacts); i++ {
			if contacts[i].Penetration > float64(max) {
				max = Real(contacts[i].Penetration)
				index = i
			}
		}

		if index == numContacts {
			break
		}

		// Match the awake state at the contact
		contacts[index].matchAwakeState()

		// Resolve the penetration
		contacts[index].applyPositionChange(linearChange, angularChange, float64(max))

		// Again this action may have changed the penetration of other bodies, so we update contacts.
		for i := 0; i < numContacts; i++ {
			// Check each body in the contact
			for b := 0; b < 2; b++ {
				if contacts[i].body[b] != nil {
					// Check for a match with each body in the newly resolved contact
					for d := 0; d < 2; d++ {
						if contacts[i].body[b] == contacts[index].body[b] {
							deltaPosition := linearChange[d].Add(angularChange[d].VectorProduct(contacts[i].relativeContactPosition[b]))

							// The sign of the change is positive if we're dealing with the second body in a contact
							// and negative otherwise
							if b == 1 {
								contacts[i].Penetration += float64(deltaPosition.ScalarProduct(contacts[i].ContactNormal))
							} else {
								contacts[i].Penetration -= float64(deltaPosition.ScalarProduct(contacts[i].ContactNormal))
							}
						}
					}
				}
			}
			cr.positionIterationsUsed++
		}
	}
}

// Calculates the impulse needed to resolve both the seperating velocity and any relative tangential velocity at the contact, taking friction into account.
func (c *Contact) calculateFrictionImpulse(inverseInertiaTensor [2]*Matrix3) *Vector {
	impulseContact := &Vector{}
	inverseMass := c.body[0].GetInverseMass()

	// The equivalent of a cross product in matrices is multiplication by a skew-symmetric matrix - we build the matrix for converting b/w linear and angular quantities.
	impulseToTorque := &Matrix3{}
	impulseToTorque.SetSkewSymmetrix(c.relativeContactPosition[0])

	// Build the matrix to convert contact impulse to change in velocity in world coordinates for the first body.
	deltaVelWorld := impulseToTorque
	deltaVelWorld.Multiply(inverseInertiaTensor[0])
	deltaVelWorld.Multiply(impulseToTorque)
	deltaVelWorld.MultiplyScalar(-1)

	// Check if we need to add the second body's data
	if c.body[1] != nil {
		// Set the cross-product matrix for the second body.
		impulseToTorque2 := &Matrix3{}
		impulseToTorque2.SetSkewSymmetrix(c.relativeContactPosition[1])

		// Calculate the velocity change matrix for the second body.
		deltaVelWorld2 := impulseToTorque2
		deltaVelWorld2.Multiply(inverseInertiaTensor[1])
		deltaVelWorld2.Multiply(impulseToTorque2)
		deltaVelWorld2.MultiplyScalar(-1)

		// Add to the total delta velocity
		deltaVelWorld.Add(deltaVelWorld2)

		// Add to the inverse mass
		inverseMass += c.body[1].GetInverseMass()
	}

	// Do a change of basis to convert into contact coordinates
	deltaVelocity := c.ContactToworld.Transpose()
	deltaVelocity.Multiply(deltaVelWorld)
	deltaVelocity.Multiply(c.ContactToworld)

	// Add in the linear velocity change (inverse mass along the diagonal)
	deltaVelocity.Data[0] += float32(inverseMass)
	deltaVelocity.Data[1] += float32(inverseMass)
	deltaVelocity.Data[2] += float32(inverseMass)

	// Invert to get the impulse needed per unit velocity change.
	impulseMatrix := deltaVelocity.Inverse()

	// Find the target velocities to kill: the desired change along the normal and the negative of the current tangential velocities.
	velKill := NewVector3(
		c.desiredDeltaVelocity, -c.ContactVelocity.Y, -c.ContactVelocity.Z,
	)

	// Find the impulse to kill the target velocities.
	impulseContact = impulseMatrix.Transform(velKill)

	// Check for exceeding friction
	planarImpulse := Sqrt(impulseContact.Y*impulseContact.Y + impulseContact.Z*impulseContact.Z)
	if planarImpulse > Abs(impulseContact.X*c.friction) {
		impulseContact.Y /= planarImpulse
		impulseContact.Z /= planarImpulse
		impulseContact.X = Real(deltaVelocity.Data[0]) + Real(deltaVelocity.Data[1])*c.friction*impulseContact.Y + Real(deltaVelocity.Data[2])*c.friction*impulseContact.Y

		impulseContact.X = c.desiredDeltaVelocity / impulseContact.X
		impulseContact.Y *= c.friction * impulseContact.X
		impulseContact.Z *= c.friction * impulseContact.X
	}
	return impulseContact
}
