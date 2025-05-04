package internal

// Defines an interface for objects that can apply forces to rigid bodies.
//
// The force generator pattern allows for a clean seperation b/w the physics engine and the
// specific forces that act on bodies. By implementing different force generators, we can model
// various physical phenomena like gravity, springs, drag, buoyancy, etc. without modifying the
// core integration code.
//
// Each force generator is responsible for calculating the appropriate force based on the current
// state of the body and applying it using the body's force accumulation methods.
type ForceGenerator interface {
	// Calculates and applies a force to the given rigid body. This method is called once per frame.
	UpdateForce(body *RigidBody, duration float32)
}

// A Force generator that applies a gravitational force.
//
// This generator applies a simple gravitational acceleration to objects, scaled by their mass (F = mg). This follows
// Newton's law of gravitation for objects near a massive body like the Earth, where the gravitational field can be
// approximated as uniform.
//
// A Single gravity generator can be used for multiple rigid bodies to effeciently model gravitational forces in a scene.
type Gravity struct {
	// Holds the acceleration due to gravity in world coordinates. Typically this would be (0, -9,81, 0) for Earth-like gravity,
	// assuming Y is the up axis.
	Gravity *Vector
}

// Applies the gravitational force to the given rigid body. The force applied is F = mg, where m is the body's mass and g is
// the gravitational acceleration. This models the fact that gravity applies a force proportional to an object's mass.
//
// Bodies with infinite mass (mass=0, inverseMass=0) are not affected by gravity, which allows for immovable objects like terrain.
//
// Since the gravitational force acts at the center of mass, it produces no torque and only affects linear motion.
func (g *Gravity) UpdateForce(body *RigidBody, duration float32) {
	// Check that we do not have infinite mass
	if !body.HasFiniteMass() {
		return
	}

	// Apply F = mg (force = mass * acceleration)
	body.AddForce(g.Gravity.Scale(body.GetMass()))
}

// A Force generator that applies a spring force b/w two rigid bodies.
//
// A spring connects two points - one on each body - and generates forces proportional to the extension or compression
// of the spring from its rest length. This follows Hooke's Law: F = -k(x - x0), where:
// - k is the spring constant (stiffness)
// - x is the current length
// - x0 is the rest length
//
// Springs are useful for creating a wide variety of physical effects including:
// - Suspensions in vehicles
// - Elastic connections b/w objects
// - Constraints that allow some flexibility
// - Soft body physics approximations
type Spring struct {
	// The attachment point of the spring in the first body's local coordinates. This represents where the spring connects
	// to the body, relative to its center of mass.
	ConnectionPoint *Vector
	// The attachment point of the spring in the second body's local coordinates. This represents where the spring connects
	// to the other body, relative to its center of mass.
	OtherConnectionPoint *Vector
	// The rigid body at the other end of the spring. The spring will generate equal and opposite forces on both this body
	// and the other body.
	Other *RigidBody
	// Holds the spring constant (k).
	SpringConstant float32
	// Holds the natural length of the spring when no forces are applied. The spring generates forces to try to return to this length
	// when stretched or compressed.
	RestLength float32
}

// Applies the spring force to the given rigid body.
//
// The method implements Hooke's law (F = -k(x - x0)) by:
// 1. Calculating the world positions of both spring connection points.
// 2. Finding the vector b/w them (representing the spring's current direction)
// 3. Determining how far the spring is from its rest length
// 4. Calculating the magnitude of the force using the spring constant
// 5. Applying the force at the appropriate connection point
//
// The force is applied at the connection point rather than the center of mass, which typically produces both linear and angular acceleration.
// The negative force direction ensures the spring pulls bodies together when extended and pushes them apart when compressed.
func (s *Spring) UpdateForce(body *RigidBody, duration float32) {
	// Calculate the two ends in world space. This transforms the local connection points to their current position in world coordinates
	lws := body.GetPointInWorldSpace(s.ConnectionPoint)
	ows := s.Other.GetPointInWorldSpace(s.OtherConnectionPoint)

	// Calculate the vector of the spring. This represents both the direction and current length of the spring.
	force := lws.Subtract(ows)

	magnitude := force.Magnitude()

	// Find how far it is from the rest length
	magnitude = Abs(magnitude - Real(s.RestLength))

	// Scale by the spring constant to get the force magnitude
	magnitude *= Real(s.SpringConstant)

	// Calculate the final force and apply it, First normalize it
	force.Normalize()

	// Then scale by the negative magnitude (because all springs pull when extended)
	force = force.Scale(-magnitude)

	// Apply the force at the connection point on the body
	body.AddForceAtPoint(force, lws)
}
