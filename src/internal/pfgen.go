package internal

import "math"

/*
Implementation of the particle force generators.
*/

// Defines the interface for objects that can apply forces to particles.
// Force generators can be used to implement various types of forces like gravity, drag, spring etc.
type ParticleForceGenerator interface {
	UpdateForce(particle *Particle, duration Real)
}

// Keeps track of one force generator and the particle it applies to.
type ParticleForceRegistration struct {
	Particle  *Particle
	Generator ParticleForceGenerator
}

// Holds all force generators and their associated particles.
// It manages the application of forces to particles in the simulation.
type ParticleForceRegistry struct {
	registrations []ParticleForceRegistration
}

func NewParticleForceRegistry() *ParticleForceRegistry {
	return &ParticleForceRegistry{
		registrations: make([]ParticleForceRegistration, 0),
	}
}

func (r *ParticleForceRegistry) Add(particle *Particle, generator ParticleForceGenerator) {
	r.registrations = append(r.registrations, ParticleForceRegistration{
		Particle:  particle,
		Generator: generator,
	})
}

func (r *ParticleForceRegistry) Remove(particle *Particle, generator ParticleForceGenerator) {
	for i := 0; i < len(r.registrations); i++ {
		reg := &r.registrations[i]
		if reg.Particle == particle && reg.Generator == generator {
			// Remove by swapping with last element and truncating
			r.registrations[i] = r.registrations[len(r.registrations)-1]
			r.registrations = r.registrations[:len(r.registrations)-1]
			return
		}
	}
}

func (r *ParticleForceRegistry) Clear() {
	r.registrations = r.registrations[:0]
}

// Updates all particles` forces using their registered generators.
func (r *ParticleForceRegistry) UpdateForces(duration Real) {
	for i := range r.registrations {
		reg := r.registrations[i]
		reg.Generator.UpdateForce(reg.Particle, duration)
	}
}

// GRAVITY FORCE GENERATOR

// Generates a constant gravitational force.
// A single instance can be used for multiple particles to apply the same gravitational acceleration.
type ParticleGravity struct {
	Gravity Vector
}

// Creates a gravity force generator with the given acceleration.
func NewParticleGravity(gravity Vector) *ParticleGravity {
	return &ParticleGravity{
		Gravity: gravity,
	}
}

// Applies gravitational force to the given particle.
// The force is calculalted as F = mg, where:
// - m is the mass of the particle
// - g is the gravitational acceleration
func (g *ParticleGravity) UpdateForce(particle *Particle, duration Real) {
	// Check that we do not have infinite mass
	if !particle.HasFiniteMass() {
		return
	}

	// Apply the mass-scaled force to the particle
	particle.AddForce(g.Gravity.Scale(particle.GetMass()))
}

// DRAG FORCE GENERATOR

// Generates a drag force that is proportional to the particle's velocity.
// The force is calculated as F = -(K1*v + K2*v²)v̂, where:
// - K1 is the velocity drag coefficient
// - K2 is the velocity squared drag coefficient
// - v is the velocity magnitude
// - v̂ is the velocity unit vector
type ParticleDrag struct {
	K1 Real
	K2 Real
}

func NewParticleDrag(k1, k2 Real) *ParticleDrag {
	return &ParticleDrag{
		K1: k1,
		K2: k2,
	}
}

// Applies the drag force to the given particle
func (d *ParticleDrag) UpdateForce(particle *Particle, duration Real) {
	// Get the particle's velocity
	velocity := particle.GetVelocity()

	// Calculate the total drag coefficient
	dragCoeff := velocity.Magnitude()
	dragCoeff = d.K1*dragCoeff + d.K2*dragCoeff*dragCoeff

	// Calculate final force and apply it
	velocity.Normalize()
	velocity.Scale(-dragCoeff)
	particle.AddForce(velocity)
}

// SPRING FORCE GENERATOR

// Generates a spring force between two particles.
// The force follows Hooke's law: F = -k(|d| - r)(d/|d|), where:
// - k is the spring constant
// - d is the vector between the particles
// - |d| is the current length of the spring
// - r is the rest length
type ParticleSpring struct {
	Other          *Particle
	SpringConstant Real
	RestLength     Real
}

func NewParticleSpring(other *Particle, springConstant, restLength Real) *ParticleSpring {
	return &ParticleSpring{
		Other:          other,
		SpringConstant: springConstant,
		RestLength:     restLength,
	}
}

// Applies the spring force to the given particle
func (s *ParticleSpring) UpdateForce(particle *Particle, duration Real) {
	// Calculate the vector of the spring by getting the vector from other to particle (p - o)
	force := particle.GetPosition()
	force.SubtractInPlace(s.Other.GetPosition())

	// Calculate the magintude of the force using Hooke's law
	magnitude := force.Magnitude()
	magnitude = Real(math.Abs(float64(magnitude - s.RestLength)))
	magnitude *= s.SpringConstant

	// Calculate the final force and apply it
	force.NormalizeInPlace()
	force.ScaleInPlace(-magnitude)
	particle.AddForce(force)
}

// ANCHORED SPRING FORCE GENERATOR

// Generates a spring force between a particle and a fixed anchor point.
// The force follows Hooke's law
type ParticleAnchoredSpring struct {
	Anchor         Vector // The fixed location that the spring is anchored to
	SpringConstant Real   // Defines the stiffness
	RestLength     Real   // Natural length of the spring when no force is applied
}

func NewParticleAnchoredSpring(anchor Vector, springConstant, restLength Real) *ParticleAnchoredSpring {
	return &ParticleAnchoredSpring{
		Anchor:         anchor,
		SpringConstant: springConstant,
		RestLength:     restLength,
	}
}

func (s *ParticleAnchoredSpring) UpdateForce(particle *Particle, duration Real) {
	// Calculate the vector of the spring
	force := particle.GetPosition()
	force.SubtractInPlace(s.Anchor)

	// Calculate the magnitude of the force
	magnitude := force.Magnitude()
	magnitude = (s.RestLength - magnitude) * s.SpringConstant

	// Calculate final force and apply it
	force.NormalizeInPlace()
	force.ScaleInPlace(-magnitude)
	particle.AddForce(force)
}

// BUNGEE SPRING FORCE GENERATOR

// Generates a spring force only when extended
type ParticleBungee struct {
	Other          *Particle
	SpringConstant Real
	RestLength     Real
}

func NewParticleBungee(other *Particle, springConstant, restLength Real) *ParticleBungee {
	return &ParticleBungee{
		Other:          other,
		SpringConstant: springConstant,
		RestLength:     restLength,
	}
}

// Applies the spring force to the given particle.
func (b *ParticleBungee) UpdateForce(particle *Particle, duration Real) {
	// Calculate the vector of the spring
	force := particle.GetPosition()
	force.SubtractInPlace(b.Other.GetPosition())

	// Check if bungee is compressed
	magnitude := force.Magnitude()
	if magnitude <= b.RestLength {
		return
	}

	// Calculate the magnitude of the force
	magnitude = b.SpringConstant * (b.RestLength - magnitude)

	// Calculate the final force and apply it
	force.NormalizeInPlace()
	force.Scale(-magnitude)
	particle.AddForce(force)
}
