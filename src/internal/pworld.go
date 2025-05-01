package internal

// Manages and updates a collection of particles in a physics simulation.
// It handles:
// 1. Particle storage and lifecycle management
// 2. Contact detection and resolution b/w particles
// 3. Force application and integration updates
// 4. Overall simulation stepping
type ParticleWorld struct {
	Particles           []*Particle
	maxContacts         uint
	resolverIterations  uint                       // Number of iterations to use in contact resolution, More iterations = more accurate but slower simulation
	registry            *ParticleForceRegistry     // Holds the force generators for particles
	resolver            *ParticleContactResolver   // Handles contact resolution b/w particles
	contactGenerators   []ParticleContactGenerator // Holds the list of objects that can generate contacts
	contacts            []*ParticleContact         // Stores the array of current contacts
	calculateIterations bool
}

// Creates a new particle physics simulator
func NewParticleWorld(maxContacts uint, iterations uint) *ParticleWorld {
	world := &ParticleWorld{
		Particles:           make([]*Particle, 0),
		maxContacts:         maxContacts,
		resolverIterations:  iterations,
		contacts:            make([]*ParticleContact, maxContacts),
		contactGenerators:   make([]ParticleContactGenerator, 0),
		calculateIterations: iterations == 0,
	}

	world.resolver = NewParticleContactResolver(iterations)
	return world
}

// Initializes the world state for a new simulation frame.
// It should be called at the beginning of each frame, before applying any forces or running physics calculations.
func (w *ParticleWorld) StartFrame() {
	// Remove all forces from the accumulator
	for _, p := range w.Particles {
		p.ClearAccumulator()
	}
}

func (w *ParticleWorld) GetParticles() []*Particle {
	return w.Particles
}

func (w *ParticleWorld) SetParticle(particle *Particle) {
	w.Particles = append(w.Particles, particle)
}

func (w *ParticleWorld) GetContactGenerators() []ParticleContactGenerator {
	return w.contactGenerators
}

func (w *ParticleWorld) SetContactGenerator(gen ParticleContactGenerator) {
	w.contactGenerators = append(w.contactGenerators, gen)
}

func (w *ParticleWorld) GetForceRegistry() *ParticleForceRegistry {
	return w.registry
}

// Calls each of the registered contact generators to report their contacts.
// Returns the number of generated contacts.
func (w *ParticleWorld) generateContacts() uint {
	limit := w.maxContacts
	nextContact := w.contacts

	for _, generator := range w.contactGenerators {
		// Get contacts from this generator
		used := generator.AddContact(nextContact[0], limit)

		// Update our counters
		limit -= used
		nextContact = nextContact[used:]

		// Break if we've run out of contacts
		if limit <= 0 {
			break
		}
	}

	return w.maxContacts - limit
}

// Moves all particles in this world forward in time by the given duration.
func (w *ParticleWorld) integrate(duration Real) {
	for _, particle := range w.Particles {
		particle.Integrate(duration)
	}
}

// Processes all the physics calculations for the particle world.
// This is the main entry point for physics simulation each frame.
func (w *ParticleWorld) RunPhysics(duration Real) {
	// First apply the force generators
	w.registry.UpdateForces(duration)
	// Then integrate the objects
	w.integrate(duration)
	// Generate contacts
	usedContacts := w.generateContacts()
	// Process contacts if we have any
	if usedContacts > 0 {
		if w.calculateIterations {
			w.resolver.SetIterations(usedContacts * 2)
		}
		w.resolver.ResolveContacts(w.contacts[:usedContacts], duration)
	}
}

// Generates contacts b/w particles and the ground plane. It implements the ParticleContactGenerator interface to
// integrate with the physics engine's contact resolution system.
type GroundContacts struct {
	ParticleContactGenerator
	particles []*Particle // refs to particles that need ground collision checking
}

func (g *GroundContacts) Init(particles []*Particle) {
	g.particles = particles
}

// Implements ParticleContactGenerator interface.
// It generates contacts for any particles that have penetrated the ground plane (y < 0).
// Parameters:
//   - contact: Pointer to first available contact in contact array.
//   - limit:   Max number of contacts that can be generated.
func (g *GroundContacts) AddContact(contact *ParticleContact, limit uint) uint {
	count := uint(0)

	for _, p := range g.particles {
		y := p.GetPosition().Y
		if y < 0.0 {
			if count >= limit {
				return count
			}

			contact.ContactNormal = UP
			contact.Particles[0] = p
			contact.Particles[1] = nil
			contact.Penetration = -y
			contact.Restitution = 0.2

			count++
		}

	}

	return count
}
