package demos

import "github.com/harpreetsingh/galileo/src/internal"

const (
	RodCount     = 6
	CableCount   = 10
	SupportCount = 12
	BaseMass     = 1.0
	ExtraMass    = 10.0
)

type BridgeDemo struct {
	*MassAggregateApplication
	supports       []*internal.ParticleCableConstraint
	cables         []*internal.ParticleCable
	rods           []*internal.ParticleRod
	massPos        *internal.Vector
	massDisplayPos *internal.Vector
}

func NewBridgeDemo() *BridgeDemo {
	baseApp := NewMassAggregateApplication(12)

	demo := &BridgeDemo{
		MassAggregateApplication: baseApp,
		supports:                 make([]*internal.ParticleCableConstraint, SupportCount),
		cables:                   make([]*internal.ParticleCable, CableCount),
		rods:                     make([]*internal.ParticleRod, RodCount),
		massPos:                  internal.NewVector3(0, 0, 0.5),
	}

	// Create the masses and connections
	for i := 0; i < 12; i++ {
		x := (i % 12) / 2
		demo.ParticleArray[i].SetPosition(internal.NewVector3(internal.Real((i/2)*2.0-5.0), 4, internal.Real(i%2)*2.0-1.0))
		demo.ParticleArray[i].SetVelocity(internal.NewVector3(0, 0, 0))
		demo.ParticleArray[i].SetDamping(0.9)
		demo.ParticleArray[i].SetAcceleration(internal.GRAVITY)
		demo.ParticleArray[i].ClearAccumulator()
	}

	// Add the cable links b/w particles
	for i := 0; i < CableCount; i++ {
		demo.cables[i].Particles[0] = demo.ParticleArray[i]
		demo.cables[i].Particles[1] = demo.ParticleArray[i+2]
		demo.cables[i].MaxLength = 1.9
		demo.cables[i].Restitution = 0.3
		demo.World.SetContactGenerator(demo.cables[i])
	}

	// Add support cables that connect to fixed anchor points
	for i := 0; i < SupportCount; i++ {
		demo.supports[i].Particle = demo.ParticleArray[i]
		demo.supports[i].Anchor = internal.NewVector3(internal.Real(i/2)*2.2-5.5, 6, internal.Real(i%2)*1.6-0.8)

		if i < 6 {
			demo.supports[i].MaxLength = float32(i / 2)
		}
	}

	return demo
}
