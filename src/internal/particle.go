package internal

// Represents the simplest object that can be simulated in the physics system.
// It encapsulates basic motion properties like position, velocity, and acceleration, making it suitable
// for particle systems, projectile motion, and other simple physics simulations where rotation and complex
// shapes are not needed.
type Particle struct {
	// Holds the linear position of the particle in world space.
	// It represents the absolute location of the particle in the simulation.
	Position *Vector
	// Holds the linear velocity of the particle in world space.
	// It represents the rate of change of position over time.
	Velocity *Vector
	// Holds the acceleration of the particle. This value is typically used to set acceleration
	// due to gravity (its primary use), or any other constant acceleration.
	Acceleration *Vector
	// Holds the amount of damping applied to linear motion. Damping is required to remove energy added
	// through numerical instability in the integrator.
	Damping Real
	// Holds the inverse of the mass of the particle. It is more useful to hold the inverse mass than mass because
	// integration is simpler, and because in real-time simulation, it is more useful to have objects with infinite
	// masss (immovable) than zero mass (completely unstable in numerical simulation).
	InverseMass Real
	// Holds the accumulated force to applied at the next simulation iteration/ This value is zeroed at each integration
	// step.
	ForceAccum *Vector
}

func NewParticle(initialPosition *Vector) *Particle {
	return &Particle{
		Position:     initialPosition,
		Velocity:     NewZeroVector(),
		Acceleration: NewZeroVector(),
	}
}

func NewParticleWithProperties(position, velocity, acceleration *Vector) *Particle {
	return &Particle{
		Position:     position,
		Velocity:     velocity,
		Acceleration: acceleration,
	}
}

// Sets the inverse mass (may be 0 for infinite mass).
func (p *Particle) SetInverseMass(inverseMass Real) {
	p.InverseMass = inverseMass
}

func (p *Particle) GetInverseMass() Real {
	return p.InverseMass
}

// Sets the mass of the particle (must be > 0).
func (p *Particle) SetMass(mass Real) {
	if mass <= 0 {
		panic("mass must be greater than 0")
	}
	p.InverseMass = 1.0 / mass
}

// Gets the mass of the particle
func (p *Particle) GetMass() Real {
	if p.InverseMass == 0 {
		return MaxReal
	}

	return 1.0 / p.InverseMass
}

func (p *Particle) HasFiniteMass() bool {
	return p.InverseMass > 0.0
}

// Integrates the particle forward in time by the given amount.
// Uses Newton-Euler integration method, which is a linear approximation.
// The integration only occurs if the particle has finite mass (inverseMass > 0).
// The integration process:
// 1. Updates position based on velocity
// 2. Calculates resulting acceleration
// 3. Updates velocity based on acceleration
// 4. Applies velocity damping
// 5. Clears force accumulator
func (p *Particle) Integrate(duration Real) {
	// Don`t integrate particles with infinite mass
	if p.InverseMass <= 0.0 {
		return
	}

	// Ensure valid duration
	if duration <= 0.0 {
		panic("integration duration must be positive")
	}

	// Update linear position
	// pos = pos + vel * dt
	p.Position.AddScaledVector(p.Velocity, duration)

	// Calculating resulting acceleration
	// For now, just use the constant acceleration
	resultingAcc := p.Acceleration

	// Update linear velocity from acceleration
	// vel = vel + acc * dt
	p.Velocity.AddScaledVector(resultingAcc, duration)

	// Applying drag (damping)
	// vel = vel * damping^dt
	dampingVector := Pow(p.Damping, duration)
	p.Velocity.ScaleInPlace(dampingVector)

	// Clear accumulated forces
	p.ClearAccumulator()
}

func (p *Particle) ClearAccumulator() {
	p.ForceAccum = NewZeroVector()
}

// Adds a force to the accumulator.
func (p *Particle) AddForce(force *Vector) {
	p.ForceAccum.AddInPlace(force)
}

func (p *Particle) SetDamping(damping Real) {
	p.Damping = damping
}

func (p *Particle) GetDamping() Real {
	return p.Damping
}

func (p *Particle) SetPosition(position *Vector) {
	p.Position = position
}

func (p *Particle) SetPositionComponents(x, y, z Real) {
	p.Position.X = x
	p.Position.Y = y
	p.Position.Z = z
}

func (p *Particle) GetPosition() *Vector {
	return p.Position
}

func (p *Particle) SetVelocity(velocity *Vector) {
	p.Velocity = velocity
}

func (p *Particle) GetVelocity() *Vector {
	return p.Velocity
}

func (p *Particle) SetVelocityComponents(x, y, z Real) {
	p.Velocity.X = x
	p.Velocity.Y = y
	p.Velocity.Y = z
}

func (p *Particle) SetAcceleration(acceleration *Vector) {
	p.Acceleration = acceleration
}

func (p *Particle) GetAcceleration() *Vector {
	return p.Acceleration
}

func (p *Particle) SetAccelerationComponents(x, y, z Real) {
	p.Acceleration.X = x
	p.Acceleration.Y = y
	p.Acceleration.Z = z
}

// Returns the kinetic energy of the particle.
// Kinetic energy is calculated as KE = ½mv², where:
// - m is the mass of the particle (1 / inverseMass)
// - v² is the square of the velocity magnitude
func (p *Particle) CalculateKineticEnergy() Real {
	// Return 0 for infinite mass particles
	if p.InverseMass <= 0 {
		return 0
	}
	// Calculate mass from inverse mass
	mass := Real(1.0) / p.InverseMass
	// Get square of velocity magnitude(v²)
	velocitySquared := p.Velocity.SquareMagnitude()

	return Real(0.5) * mass * velocitySquared
}
