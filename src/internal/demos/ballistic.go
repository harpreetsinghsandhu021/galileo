package demos

import (
	"time"

	"github.com/harpreetsingh/galileo/src/internal"
)

// Represents different types of ammunition that can be fired
type ShotType int

const (
	// Pistol fires a small mass at high velocity
	Pistol ShotType = iota
	// Artillery fires a large mass with arcing trajectory
	Artillery
	// Fireball has small mass, floats upward
	Fireball
	// Laser has minimal mass, moves at high speed without gravity
	Laser
)

// Represents a projectile in the simulation
type Shot struct {
	Particle  *internal.Particle
	StartTime float64
	Type      ShotType
}

func NewShot(shotType ShotType) *Shot {
	// Create particle at default starting position
	startPos := internal.NewVector3(0.0, 1.5, 0.0)
	particle := internal.NewParticle(startPos)

	// Configure particle based on shot type
	switch shotType {
	case Pistol:
		// 2.0kg projectile
		particle.SetMass(2.0)
		// 35 m/s along z-axis
		particle.SetVelocity(internal.NewVector3(0.0, 0.0, 35.0))
		// Normal gravity
		particle.SetAcceleration(internal.NewVector3(0.0, -1.0, 0.0))
		particle.SetDamping(0.99)

	case Artillery:
		// 200kg projectile
		particle.SetMass(200.0)
		// 50 m/s at 30 degree elevation
		particle.SetVelocity(internal.NewVector3(0.0, 30.0, 40.0))
		// higher gravity for gameplay
		particle.SetAcceleration(internal.NewVector3(0.0, -20.0, 0.0))
		particle.SetDamping(0.99)

	case Fireball:
		// 0.1kg energy bolt
		particle.SetMass(1.0)
		// 10 m/s along z-axis
		particle.SetVelocity(internal.NewVector3(0.0, 0.0, 10.0))
		// Floats upward
		particle.SetAcceleration(internal.NewVector3(0.0, 0.6, 0.0))
		particle.SetDamping(0.9)

	case Laser:
		// 0.1kg energy bolt
		particle.SetMass(0.1)
		// 100 m/s along z-axis
		particle.SetVelocity(internal.NewVector3(0.0, 0.0, 100.0))
		// No gravity effect
		particle.SetAcceleration(internal.NewVector3(0.0, 0.0, 0.0))
		particle.SetDamping(0.99)
	}

	// Clear any accumulated forces
	particle.ClearAccumulator()

	return &Shot{
		Particle:  particle,
		StartTime: float64(time.Now().UnixNano()),
		Type:      shotType,
	}
}
