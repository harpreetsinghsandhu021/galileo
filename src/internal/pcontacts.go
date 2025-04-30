package internal

// Represents two objects in contact (specifically two particles).
// Resolving a contact:
// 1. Removes their interpenetration
// 2. Applies suffiicient impulse to keep them apart
// 3. Handles rebound for colliding bodies
// The contact has no methods, it just holds the contact details.
// To resolve contacts, use the particle contact resolver.
type ParticleContact struct {
	// Particles involved in the contact.
	// The second particle can be nil for contacts with scenery.
	// If we are dealing with a collision between an object and the scenery, then there is only one object involved
	Particles [2]*Particle
	// Coefficient at the contact point.
	// Determines how "bouncy" the collision is.
	Restitution Real
	// Direction of the contact in world coordinates.
	// This is typically the normal vector at the contact point.
	ContactNormal *Vector
	// Holds the depth of penetration at the contact.
	Penetration Real
}

// Handles the complete contact resolution between two particles.
// If follows a two-phase resolution process:
// 1. Velocity resolution - handles the rebounding effect
// 2. Penetration resolution - ensures particles don't overlap
func (c *ParticleContact) Resolve(duration Real) {
	c.resolveVelocity(duration)
	c.resolveInterpenetration(duration)
}

// Determines the closing velocity between particles. A positive value indicates particles are moving apart.
// A negative value indicates particles are moving towards each other. For single particle contacts (with scenery),
// only first particle's velocity is used
func (c *ParticleContact) calculateSeperatingVelocity() Real {
	relativeVelocity := c.Particles[0].GetVelocity()

	if c.Particles[1] != nil {
		relativeVelocity.SubtractInPlace(c.Particles[1].GetVelocity())
	}

	return relativeVelocity.ScalarProduct(c.ContactNormal)
}

// Handles the velocity changes during collision using the impulse-momentum equation, with additional handling for acceleration
// based velocity buildup from classical mechancis:
// Formula: v_new = v_old + (J * n) / m, where:
// - v_new: new velocity after collision
// - v_old: original velocity before collision
// - J: impulse magnitude (calculated as deltaVel/totalInverseMass)
// - n: contact normal direction
// - m: particle mass (we use 1/m for efficiency)
// The formula ensures:
// 1. Conservation of linear momentum
// 2. Proper velocity distribution based on mass
// 3. Correct post-collision trajectory
func (c *ParticleContact) resolveVelocity(duration Real) {
	// Get the velocity in the direction of the contact normal. This tells us if the particles are moving together or apart.
	seperatingVelocity := c.calculateSeperatingVelocity()

	// If seperating velocity is positive, particles are already moving apart. No resilution needed in this case.
	if seperatingVelocity > 0 {
		return
	}

	// Calculate new seperating velocity using coefficient of restituion
	// restitution = 1.0 means perfect bounce
	// restitution = 0.0 means complete energy loss
	newSepVelocity := -seperatingVelocity * c.Restitution

	// Check velocity buildup due to acceleration only
	accCausedVelocity := c.Particles[0].GetAcceleration()
	if c.Particles[1] != nil {
		accCausedVelocity.SubtractInPlace(c.Particles[1].GetAcceleration())
	}

	accCausedSepVelocity := accCausedVelocity.ScalarProduct(c.ContactNormal)

	// If we've got a closing velocity due to acceleration buildup, remove it from the new separating velocity
	if accCausedSepVelocity < 0 {
		newSepVelocity += c.Restitution * accCausedSepVelocity

		// Ensyre we have'nt removed more than was there to remove.
		if newSepVelocity < 0 {
			newSepVelocity = 0
		}
	}

	deltaVelocity := newSepVelocity - seperatingVelocity

	// Calculate inverse mass sum to distribute impulse properly.
	// Objects with higher mass (lower inverse mass) recieve less velocity change.
	totalInverseMass := c.Particles[0].GetInverseMass()
	if c.Particles[1] != nil {
		totalInverseMass += c.Particles[1].GetInverseMass()
	}

	// If total inverse mass is 0, both particles have infinite mass.
	// No velocity changes possible in this case.
	if totalInverseMass <= 0 {
		return
	}

	// Calculate impulse magnitude using velocity change and mass distribution.
	impulse := deltaVelocity / totalInverseMass

	// Convert impulse into a vector in the contact normal direction.
	impulsePerMass := c.ContactNormal.Scale(impulse)

	// Apply impulse to first particle using the impulse-momemtum equation: v_new = v_old + (J * n) / m
	// Velocity change = impulse * inverse mass.
	p0NewVelocity := c.Particles[0].GetVelocity().
		Add(impulsePerMass.Scale(c.Particles[0].GetInverseMass()))
	c.Particles[0].SetVelocity(p0NewVelocity)

	// Apply opposite impulse to second particle if it exists.
	// Note the negative scale to reverse the impulse direction.
	if c.Particles[1] != nil {
		p1NewVelocity := c.Particles[1].GetVelocity().
			Add(impulsePerMass.Scale(-c.Particles[1].GetInverseMass()))
		c.Particles[1].SetVelocity(p1NewVelocity)
	}
}

// Handles overlapping particles by adjusting their positions.
// Uses the formula: △x = (d * n) * (1 / m) / (1 / m_total), where:
// - △x: position correction for each particle
// - d: penetration depth
// - n: contact normal
// - m: particle mass
// - m_total: sum of both particle's masses
// The correction is distributed inversely propotional to the masses:
// - Heavier objects (smaller 1/m) move less
// - Lighter objects (larger 1/m) move more
func (c *ParticleContact) resolveInterpenetration(duration Real) {
	// Skip if no penetration existss
	if c.Penetration <= 0 {
		return
	}

	// Calculate total inverse mass for distribution of movement
	totalInverseMass := c.Particles[0].GetInverseMass()
	if c.Particles[1] != nil {
		totalInverseMass += c.Particles[1].GetInverseMass()
	}

	// Skip resolution if both particles have infinite mass
	if totalInverseMass <= 0 {
		return
	}

	// Calculate move per unit inverse mass
	// movePerMass = n * (d / totalInverseMass)
	movePerMass := c.ContactNormal.Scale(c.Penetration / totalInverseMass)

	// Calculate and store movement amounts
	// movement = movePerMass * (1/m)
	particleMovement := [2]*Vector{
		movePerMass.Scale(c.Particles[0].GetInverseMass()),
		NewZeroVector(),
	}

	if c.Particles[1] != nil {
		particleMovement[1] = movePerMass.Scale(-c.Particles[1].GetInverseMass())
	}

	// Apply position corrections
	p0NewPos := c.Particles[0].GetPosition().Add(particleMovement[0])
	c.Particles[0].SetPosition(p0NewPos)

	if c.Particles[1] != nil {
		p1newPos := c.Particles[1].GetPosition().Add(particleMovement[0])
		c.Particles[0].SetPosition(p1newPos)
	}

}

// Handles resolution of multiple particle contacts.
// A single resolver instance can be shared for the entire simulation.
type ParticleContactResolver struct {
	iterations     uint // Maximum number of iterations allowed for contact resolution
	iterationsUsed uint // Performance tracking value - records actual iterations used
}

func NewParticleContactResolver(iterations uint) *ParticleContactResolver {
	return &ParticleContactResolver{
		iterations: iterations,
	}
}

// Updates the maximum number of iterations that can be used
func (r *ParticleContactResolver) SetIterations(iterations uint) {
	r.iterations = iterations
}

// Resolves a set of particle contacts for both penetration and velocity.
// It processes contacts in order of severity (largest closing velocity first) to ensure the most
// significant collisions are handles within the iteration limit.
func (r *ParticleContactResolver) ResolveContacts(contacts []*ParticleContact, duration Real) {
	r.iterationsUsed = 0

	for r.iterationsUsed < r.iterations {
		// Find the contact with the largest closing velocity
		max := MaxReal
		maxIndex := len(contacts)

		// Iterate through contacts to find the most severe collision
		for i := 0; i < len(contacts); i++ {
			sepVel := contacts[i].calculateSeperatingVelocity()
			if sepVel < max && (sepVel < 0 || contacts[i].Penetration > 0) {
				max = sepVel
				maxIndex = i
			}
		}

		// Break if no contacts need resolving
		if maxIndex == len(contacts) {
			break
		}

		// Resolve the most severe contact
		contacts[maxIndex].Resolve(duration)
		r.iterationsUsed++
	}
}
