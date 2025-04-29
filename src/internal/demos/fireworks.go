package demos

import (
	"math/rand"
	"time"

	"github.com/harpreetsingh/galileo/src/internal"
)

var random = rand.New(rand.NewSource(time.Now().UnixNano()))

// Represents a particle with additional properties for firework simulation.
// It extends the basic Particle type with rendering and evolution data specific to firework behavior.
type Firework struct {
	*internal.Particle
	Type uint          // Represents the firework category, used for determining payload and detonation rules
	Age  internal.Real // Represents the remaining fuse time before detonation. When age reaches 0, the firework delivers its payload.
}

// Represents a firework's detonation configuration.
// It defines what type and how many new fireworks should be created when the parent firework detonates.
type Payload struct {
	Type  int  // Category of firework to create
	Count uint // Specifies how many fireworks of this type to generate
}

// Controls how a specifc type of firework behaves, including its fuse duration, velociry ranges, and
// what is produces when it detonates.
type FireWorkRule struct {
	Type uint

	// Defines the possible range for fuse duration
	MinAge internal.Real
	MaxAge internal.Real

	// Defines the velocity bounds for the firework
	MinVelocity *internal.Vector
	MaxVelocity *internal.Vector

	// Controls how much the firework slows down over time
	Damping internal.Real

	// Defines what new fireworks are created upon detonation
	Payloads []*Payload
}

func NewFirework(position *internal.Vector, fireworkType int) *Firework {
	return &Firework{
		Particle: internal.NewParticle(position),
		Type:     uint(fireworkType),
		Age:      internal.Real(0),
	}
}

func NewPayload(payloadType, count int) *Payload {
	return &Payload{
		Type:  payloadType,
		Count: uint(count),
	}
}

func NewFireworkRule(fireworkType int,
	minAge,
	maxAge internal.Real,
	minVelocity,
	maxVelocity *internal.Vector,
	damping internal.Real,
	payloads []*Payload) *FireWorkRule {
	return &FireWorkRule{
		Type:        uint(fireworkType),
		MinAge:      minAge,
		MaxAge:      maxAge,
		MinVelocity: minVelocity,
		MaxVelocity: maxVelocity,
		Damping:     damping,
		Payloads:    payloads,
	}
}

// Creates and returns a slice of predefined FireworkRules that control the behavior of different firework
// types in the simulation.
func InitFireworkRules() []*FireWorkRule {
	// Create rules slice with initial capacity
	rules := make([]*FireWorkRule, 3)

	// Rule 0: Creates a multi-stage explosion
	rules[0] = NewFireworkRule(
		1,
		internal.Real(0.5),
		internal.Real(1.4),
		internal.NewVector3(-5, 25, -5),
		internal.NewVector3(5, 28, 5),
		internal.Real(0.1),
		[]*Payload{
			NewPayload(3, 5),
			NewPayload(5, 5),
		},
	)

	// Rule 1: Creates a medium-height burst
	rules[1] = NewFireworkRule(
		2,
		internal.Real(0.5),
		internal.Real(1.0),
		internal.NewVector3(-5, 10, -5),
		internal.NewVector3(5, 20, 5),
		internal.Real(0.8),
		[]*Payload{
			NewPayload(4, 2),
		},
	)

	rules[2] = NewFireworkRule(
		3,
		internal.Real(0.5),
		internal.Real(1.5),
		internal.NewVector3(-5, -5, -5),
		internal.NewVector3(5, 5, 5),
		internal.Real(0.1),
		[]*Payload{},
	)

	return rules
}

// Moves the firework forward in time by the given duration.
// It handles both physical simulation and life-cycle management.
func (f *Firework) Update(duration internal.Real) bool {
	// Update physical state using parent particle implementation
	f.Integrate(duration)

	// Decrease remaining fuse time
	f.Age -= duration

	// Return true if firework should be removed:
	// - Age expired (fuse burnt out)
	// - Below ground level (y < 0)
	return f.Age < 0 || f.Position.Y < 0
}

// Generates a new firework based on this rule's parameters.
// If parent is provided, the new firework inherits some properties from it.
func (r *FireWorkRule) Create(parent *Firework) *Firework {
	// Create firework with initial position
	var position *internal.Vector
	if parent != nil {
		// Inherit parent's position
		position = parent.GetPosition()
	} else {
		// Create at random ground position
		x := internal.Real(random.Float64()*10 - 5)
		position = internal.NewVector3(x, 0, 0)
	}

	firework := NewFirework(position, int(r.Type))

	// Set random age within rule's range
	ageRange := r.MaxAge - r.MinAge
	firework.Age = r.MinAge + internal.Real(random.Float64())*ageRange

	// Calculate velocity
	velocity := randomVector(r.MinVelocity, r.MaxVelocity)
	if parent != nil {
		// Add parent's velocity to inheritance
		parentVel := parent.GetVelocity()
		velocity.AddInPlace(parentVel)
	}

	// Configure physical properties
	firework.SetVelocity(velocity)
	firework.SetMass(1.0) // Consistent mass for all fireworks
	firework.SetDamping(r.Damping)
	firework.SetAcceleration(internal.GRAVITY)
	firework.ClearAccumulator()

	return firework
}

// Generates a random vector within the given bounds
func randomVector(min, max *internal.Vector) *internal.Vector {
	return &internal.Vector{
		X: min.X + internal.Real(rand.Float64())*(max.X-min.Y),
		Y: min.Y + internal.Real(rand.Float64())*(max.Y-min.Y),
		Z: min.Z + internal.Real(rand.Float64())*(max.Z-min.Z),
	}
}

// Handles creation and tracking of fireworks
type FireworkManager struct {
	Rules     []*FireWorkRule
	Fireworks []*Firework
	NextIndex int
}

// Generates a new firework of the specified type
func (fm *FireworkManager) CreateFirework(fireworkType uint, parent *Firework) {
	if int(fireworkType) > len(fm.Rules) {
		return
	}

	// Get the rule for this type
	rule := fm.Rules[fireworkType-1]

	// Create new firework
	firework := rule.Create(parent)

	// Add to fireworks array
	if fm.NextIndex >= len(fm.Fireworks) {
		// Grow array if needed
		fm.Fireworks = append(fm.Fireworks, firework)
	} else {
		// Reuse existing slot
		fm.Fireworks[fm.NextIndex] = firework
	}

	fm.NextIndex = (fm.NextIndex + 1) % cap(fm.Fireworks)
}

// Processe all active fireworks, handling detonations and creating payloads.
// It updates the physics of each firework and handles any that need any that need to detonate.
func (fm *FireworkManager) UpdateFireworks(duration internal.Real) {
	// Process all fireworks
	for i := 0; i < len(fm.Fireworks); i++ {
		firework := fm.Fireworks[i]
		if firework == nil {
			continue
		}

		// Update physics and check if firework should be removed
		if firework.Update(duration) {
			// Get the rule for this firework type
			rule := fm.Rules[firework.Type-1]

			// Create payloads before removing the firework
			for _, payload := range rule.Payloads {
				// Create specified number of new fireworks
				for j := uint(0); j < payload.Count; j++ {
					fm.CreateFirework(uint(payload.Type), firework)
				}
			}

			fm.Fireworks[i] = nil
		}
	}
}
