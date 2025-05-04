package internal

// Manages a collection of rigid bodies and provides the infrastructure to simulate their physical behavior over time.
type World struct {
	Bodies   []*RigidBody // Holds the rigid bodies being simulated
	Registry *ForceRegistry
}

// Initializes the world for a simulation frame.
//
// This prepares each rigid body for the upcoming physics calculations by:
// 1. Clearing the force and torque accumulators (resetting applied forces)
// 2. Recalculating derived data (transform matrices, world-space inertia tensors)
//
// This method should be called at the beginning of each simulation step, before any forces are applied
// to the bodies. After calling StartFrame, you can add forces to the bodies for the current frame before
// calling RunPhysics to advance the simulation.
func (w *World) StartFrame() {
	for _, body := range w.Bodies {
		body.ClearAccumulators()
		body.CalculateDerivedData()
	}
}

// Advances the state of all rigid bodies by the specified duration.
//
// This method performs numerical integration on each body in the simulation, updating positions and orientations based on
// their current velocities and the forces/torques that have been applied to them.
func (w *World) Integrate(duration float32) {
	for _, body := range w.Bodies {
		// Integrate the body by the given duration
		body.Integrate(duration)
	}
}

// Processes all the physics for the world for one time step.
//
// This method advances the simulation by the specified duration by;
// 1. Applying forces from all registered force generators
// 2. Integrating the motion of all bodies
func (w *World) RunPhysics(duration float32) {
	w.Registry.UpdateForces(duration)

	w.Integrate(duration)
}
