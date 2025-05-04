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

// A Force generator that applies aerodynamic forces to rigid bodies.
//
// Aerodynamic forces depend on the velocity of the body relative to the fluid (typically air), the orientation of the body,
// and the shape of the surfaces. This generator models these forces using an aerodynamic tensor that relates velocity to force
// for a particular surface shape.
//
// The aerodynamic force is calculated as:
// F = -v` * T * v
// where:
// F is the resulting force
// v is the relative velocity of the body through the fluid
// v` is the tranpose of v
// T is the aerodynamic tensor
//
// This model can represent various aerodynamic effects including:
// - Drag: resistance to motion through the fluid
// - Lift: perpendicular to force from assymetric airflow
// - Torque: rotational force from off-cennter aerodynamic effects
type Aero struct {
	// Holds the aerodynamic tensor for the surface in body space. This 3*3 matrix transforms the relative velocity into a force,
	// representing how the surface interacts with the fluid from different directions.
	Tensor *Matrix3
	// Holds the relative position of the aerodynamic surface in body coordinates. This is the point at which the aerodynamic force
	// is applied, which may not be at the center of mass, allowing aerodynamic forces to create torque.
	Position *Vector
	// Holds the wind speed of the enviroment.
	Windspeed *Vector
}

// Applies the aerodynamic force to the given rigid body.
func (a *Aero) UpdateForce(body *RigidBody, duration float32) {
	a.updateForceFromTensor(body, duration, a.Tensor)
}

// Uses an explicit tensor matrix to update the force on the given rigid body.
//
// Calculates and applies the aerodynamic force based on the body's velocity, the wind speed, and the provided aerodynamic tensor.
// The process is:
// 1. Calculate the velocity of the body relative to the fluid(air) by combining the body's velocity with the wind speed.
// 2. Transform this velocity into the body's local coordinate system, since the aerodynamic tensor is defined in body space.
// 3. Apply the tensor to the velocity to get the resulting force in body space F = T * v (where T is the tensor and v is the relative velocity)
// 4. Transform the force back to world space coordinates
// 5. Apply the force at the specified position in the body
// Params:
//   - body: The rigid body to apply force to
//   - duration: The time step of the simulation
//   - tensor: The aerodynamic tensor to use for this calculation
func (a *Aero) updateForceFromTensor(body *RigidBody, duration float32, tensor *Matrix3) {
	// Calculate total velocity (wind speed and body's velocity)
	// This gives us the velocity of the body relative to the fluid
	velocity := body.GetVelocity()
	velocity = velocity.Add(a.Windspeed)

	// Calculate the velocity in body coordinates
	// This is needed because the tensor is defined in body space
	bodyVel := body.GetTransform().TransformInverseDirection(velocity)

	// Calculate the force in body corrdinates by applying the tensor
	// This transforms the velocity into a force according to the aerodynamic properties of the surface
	bodyForce := tensor.Transform(bodyVel)

	// Transform the force back to world coordinates
	force := body.GetTransform().TransformDirection(bodyForce)

	body.AddForceAtBodyPoint(force, a.Position)
}

// A Force generator with a controllable aerodynamic surface.
//
// This generator extends the basic Aero force generator by allowing the aerodynamic properties to be adjusted dynamically. This
// simulates control surfaces like flaps, ailerons, rudders, or elevators on aircraft, which change the airflow and resulting forces by adjusting their angle.
//
// The control is represented by a setting value b/w -1 and +1, which interpolates b/w three tensors:
// At -1: The minimum tensor is used (e.g., control surface fully down)
// At 0: The base tensor is used (e.g., control surface neutral)
// At +1: The maximum tensor is used (e.g., control surface fully up)
//
// The interpolation allows for smooth transitions b/w different aerodynamic configurations.
type AeroControl struct {
	Aero
	// Holds the aerodynamic tensor for the surface when the control is at its maximum value(+1).
	MaxTensor *Matrix3
	// Holds the aerodynamic tensor for the surface when the control is at its minimum value(-1).
	MinTensor *Matrix3
	// Represents the current position of the control surface. It ranges from -1 through 0 to +1.
	// Values outside this range may give undefined results.
	ControlSetting float32
}

// Calculates the final aerodynamic tensor for the current control setting.
func (ac *AeroControl) GetTensor() *Matrix3 {
	if ac.ControlSetting <= -1.0 {
		return ac.MinTensor
	} else if ac.ControlSetting >= 1.0 {
		return ac.MaxTensor
	} else if ac.ControlSetting < 0 {
		return Matrix3LinearInterpolate(ac.MinTensor, ac.Tensor, ac.ControlSetting+1.0)
	} else if ac.ControlSetting > 0 {
		return Matrix3LinearInterpolate(ac.Tensor, ac.MaxTensor, ac.ControlSetting)
	} else {
		return ac.Tensor
	}
}

// Applies the aerodynamic force to the given rigid body.
func (ac *AeroControl) UpdateForce(body *RigidBody, duration float32) {
	tensor := ac.GetTensor()
	ac.updateForceFromTensor(body, duration, tensor)
}

func (ac *AeroControl) SetControl(value float32) {
	ac.ControlSetting = value
}

// Holds all the force generators and the bodies they apply to.
type ForceRegistry struct {
	registrations []*ForceRegistration
}

// Keeps track of one force generator and the body it applies to.
//
// This structure represents a connection b/w a specific force generator and a rigid body. The force
// generator will apply its force to the specified body when the forces are updated.
type ForceRegistration struct {
	Body *RigidBody
	Fg   ForceGenerator
}

// Registers the given force generator to apply to the given body
func (fr *ForceRegistry) Add(body *RigidBody, fg ForceGenerator) {
	fr.registrations = append(fr.registrations, &ForceRegistration{
		Body: body,
		Fg:   fg,
	})
}

// Calls all the force generators to update the forces of their corresponding bodies.
func (fr *ForceRegistry) UpdateForces(duration float32) {
	for _, reg := range fr.registrations {
		reg.Fg.UpdateForce(reg.Body, duration)
	}
}
