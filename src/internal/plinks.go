package internal

// Represents a connection b/w two particles that can generate contact constraints. It serves as a base for implementations
// like cables and rods. Links monitor the connected particles and create contacts when their constraints.
type ParticleLink struct {
	// Holds the two particles connected by this link.
	// Both particles must be valid (non-nil) for the link to function.
	Particles [2]*Particle
}

// Calculates and returns the current distance b/w the linked particles using their positions.
func (l *ParticleLink) currentLength() Real {
	relativePos := l.Particles[0].GetPosition().Subtract(l.Particles[1].GetPosition())

	return relativePos.Magnitude()
}

// Interface for link types
type ParticleContactGenerator interface {
	// Generates any needed contacts to maintain link constraints.
	// Returns the number of contacts generated (0 or 1).
	// contact: pointer to store the generated contact
	// limit: maximum number of contacts that can be written (must be >= 1)
	AddContact(contact *ParticleContact, limit uint) uint
}

// Implements a cable constraint b/w two particles. A cable has a max length and generates a contact when particles move too apart.
// It acts like a rope - preventing extension beyond its max length but allowing any shorter distance.
type ParticleCable struct {
	ParticleLink
	MaxLength   Real // Embed base link functionality
	Restitution Real // Bounciness coefficient for cable constraint
}

// Creates a new cable constraint b/w two particles
func NewParticleCable(p1, p2 *Particle, maxLength, restitution Real) *ParticleCable {
	return &ParticleCable{
		ParticleLink: ParticleLink{
			Particles: [2]*Particle{p1, p2},
		},
		MaxLength:   maxLength,
		Restitution: restitution,
	}
}

// Implements ParticleContactGenerator for cables. Generates a contact when the cable is extended
// beyond its maximum length. The contact will have:
// - Normal in the direction from particle[0] to particle[1]
// - Penetration as the amount of overextension
// - Specified restitution coefficient
// Returns 1 if contact was generated, 0 if cable is not overextended.
func (c *ParticleCable) AddContact(contact *ParticleContact, limit uint) uint {
	// Check current cable length
	length := c.currentLength()

	// No contact needed if cable is'nt stretched
	if length < c.MaxLength {
		return 0
	}

	// Generate contact for overextended cable
	contact.Particles[0] = c.Particles[0]
	contact.Particles[1] = c.Particles[1]

	// Calculate normalized direction vector (contact normal)
	normal := c.Particles[1].GetPosition().Subtract(c.Particles[0].GetPosition()).Normalize()

	contact.ContactNormal = normal
	contact.Penetration = length - c.MaxLength
	contact.Restitution = c.Restitution

	return 1
}

// Implements a rigid rod constraint b/w two particles.
// A rod is a bidirectional constraint that:
// 1. Maintains an exact fixed distance b/w particles
// 2. Prevents both extension (particles moving too far apart)
// 3. Prevents compression (particles moving too close together)
// 4. Has zero elasticity (no bounce) due to its rigid nature
// This makes it behave like a perfectly rigid connecting rod or beam.
type ParticleRod struct {
	ParticleLink
	Length Real // Fixed length the rod maintains b/w particles
}

func NewParticleRod(p1, p2 *Particle, length Real) *ParticleRod {
	return &ParticleRod{
		ParticleLink: ParticleLink{
			Particles: [2]*Particle{p1, p2},
		},
		Length: length,
	}
}

// Implements ParticleContactGenerator for rods by generating appropriate contact constraints when the rod's length is violated.
// The contact generation process:
// 1. Measures current distance b/w particles
// 2. Compares against rod's fixed length requirement
// 3. Generates a contact if length deviates from required length
// 4. Sets contact properties based on violation type:
//   - For extension: normal points from p0 to p1
//   - For compression: normal points from p1 to p0
//   - Penetration is the magnitude of length deviation
//   - Zero restituion ensures rigid behavior
//
// Parameters:
// - contact: Pointer to contact structure to be filled
// - limit: Maximum number of contacts that can be generated (unused as rods generate at most 1)
// Returns:
// - 1 if a contact was generated (length violation found)
// - 0 if no contact needed (particles at correct distance)
func (r *ParticleRod) AddContact(contact *ParticleContact, limit uint) uint {
	currentLen := r.currentLength()

	// No contact needed if particles are exactly at the required distance
	if currentLen == r.Length {
		return 0
	}

	// Copy particle references to contact structure
	// These particles will be affected by the contact resolution
	contact.Particles[0] = r.Particles[0]
	contact.Particles[1] = r.Particles[1]

	// Calculate normalized direction vector
	normal := r.Particles[1].GetPosition().Subtract(r.Particles[0].GetPosition()).Normalize()

	// Set contact properties based on whether rod is extended or compressed
	if currentLen > r.Length {
		// Rod is too long - generate extension contact
		// Normal points from p0 to p1 to bring particles together
		contact.ContactNormal = normal
		contact.Penetration = currentLen - r.Length
	} else {
		// Rod is too short: particles too close together
		// Normal points from p1 to p0 to push particles apart
		contact.ContactNormal = normal.Scale(-1)
		contact.Penetration = r.Length - currentLen
	}

	// Rods are perfectly rigid - no bouncing or elastic behavior
	contact.Restitution = 0

	return 1
}
