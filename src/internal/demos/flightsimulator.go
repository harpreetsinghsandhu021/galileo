package demos

import (
	"fmt"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/go-gl/glu"
	"github.com/harpreetsingh/galileo/src/internal"
)

// Main demo class. Simulates an aircraft using aerodynamic foces and allows user control of the aircraft's control surfaces.
type FlightSimDemo struct {
	Application
	LeftWing         *internal.AeroControl
	RightWing        *internal.AeroControl
	Rudder           *internal.AeroControl
	Tail             *internal.Aero
	Aircraft         *internal.RigidBody
	Registry         *internal.ForceRegistry
	Windspeed        *internal.Vector
	LeftWingControl  float32
	RightWingControl float32
	RudderControl    float32
}

func NewFlightSimDemo() *FlightSimDemo {
	demo := &FlightSimDemo{
		Windspeed: internal.NewVector3(0, 0, 0),
	}

	demo.RightWing = &internal.AeroControl{
		Aero: internal.Aero{
			Tensor:    &internal.Matrix3{Data: [9]float32{0, 0, 0, -1, -0.5, 0, 0, 0, 0}},
			Position:  internal.NewVector3(-1.0, 0.0, 2.0),
			Windspeed: demo.Windspeed,
		},
		MinTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, -0.995, -0.5, 0, 0, 0, 0}},
		MaxTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, -1.005, -0.5, 0, 0, 0, 0}},
		ControlSetting: 0,
	}

	demo.LeftWing = &internal.AeroControl{
		Aero: internal.Aero{
			Tensor:    &internal.Matrix3{Data: [9]float32{0, 0, 0, -1, -0.5, 0, 0, 0, 0}},
			Position:  internal.NewVector3(-1.0, 0.0, -2.0),
			Windspeed: demo.Windspeed,
		},
		MinTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, -0.995, -0.5, 0, 0, 0, 0}},
		MaxTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, -1.005, -0.5, 0, 0, 0, 0}},
		ControlSetting: 0,
	}

	demo.Rudder = &internal.AeroControl{
		Aero: internal.Aero{
			Tensor:    &internal.Matrix3{Data: [9]float32{0, 0, 0, 0, 0, 0, 0, 0, 0}},
			Position:  internal.NewVector3(2.0, 0.5, 0),
			Windspeed: demo.Windspeed,
		},
		MinTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, 0, 0, 0, 0.01, 0, 0}},
		MaxTensor:      &internal.Matrix3{Data: [9]float32{0, 0, 0, 0, 0, 0, -0.01, 0, 0}},
		ControlSetting: 0,
	}

	demo.Tail = &internal.Aero{
		Tensor:    &internal.Matrix3{Data: [9]float32{0, 0, 0, -1, -0.5, 0, 0, 0, -0.1}},
		Position:  internal.NewVector3(2.0, 0, 0),
		Windspeed: demo.Windspeed,
	}

	// Set up the aircraft rigid body
	demo.ResetPlane()

	demo.Aircraft.SetMass(2.5)

	// Create and set the inertia tensor
	var inertiaTensor internal.Matrix3
	inertiaTensor.SetBlockInertiaTensor(internal.NewVector3(2, 1, 1), 1)
	demo.Aircraft.SetInertiaTensor(&inertiaTensor)

	demo.Aircraft.SetDamping(0.8, 0.8)

	demo.Aircraft.SetAcceleration(internal.GRAVITY)
	demo.Aircraft.CalculateDerivedData()

	demo.Aircraft.SetAwake(true)
	demo.Aircraft.SetCanSleep(false)

	// Add force generators to registry
	demo.Registry.Add(demo.Aircraft, demo.LeftWing)
	demo.Registry.Add(demo.Aircraft, demo.RightWing)
	demo.Registry.Add(demo.Aircraft, demo.Rudder)
	demo.Registry.Add(demo.Aircraft, demo.Tail)

	return demo
}

// Resets the aircraft to its initial position and orientation.
func (demo *FlightSimDemo) ResetPlane() {
	demo.Aircraft.SetPosition(0, 0, 0)
	demo.Aircraft.SetOrientation(1, 0, 0, 0)

	demo.Aircraft.SetVelocity(0, 0, 0)
	demo.Aircraft.SetRotation(0, 0, 0)
}

// Draws the aircraft model
func DrawAircraft() {
	// Fuselage
	gl.PushMatrix()
	gl.Translatef(-0.5, 0, 0)
	gl.Scalef(2.0, 0.8, 1.0)
	DrawSolidCube(1.0)
	gl.PopMatrix()

	// Rear Fuselage
	gl.PushMatrix()
	gl.Translatef(1.0, 0.15, 0)
	gl.Scalef(2.75, 0.5, 0.5)
	DrawSolidCube(1.0)
	gl.PopMatrix()

	// Wings
	gl.PushMatrix()
	gl.Translatef(0, 0.3, 0)
	gl.Scalef(0.8, 0.1, 6.0)
	DrawSolidCube(1.0)
	gl.PopMatrix()

	// Tail-plane
	gl.PushMatrix()
	gl.Translatef(1.9, 0, 0)
	gl.Scalef(0.85, 0.1, 2.0)
	DrawSolidCube(1.0)
	gl.PopMatrix()
}

func DrawSolidCube(size float32) {
	halfSize := size / 2

	gl.Begin(gl.QUADS)

	// Front face
	gl.Vertex3f(-halfSize, -halfSize, halfSize)
	gl.Vertex3f(halfSize, -halfSize, halfSize)
	gl.Vertex3f(halfSize, halfSize, halfSize)
	gl.Vertex3f(-halfSize, halfSize, halfSize)

	// Back face
	gl.Vertex3f(-halfSize, -halfSize, -halfSize)
	gl.Vertex3f(-halfSize, halfSize, -halfSize)
	gl.Vertex3f(halfSize, halfSize, -halfSize)
	gl.Vertex3f(halfSize, -halfSize, -halfSize)

	// Top face
	gl.Vertex3f(-halfSize, halfSize, -halfSize)
	gl.Vertex3f(-halfSize, halfSize, halfSize)
	gl.Vertex3f(halfSize, halfSize, halfSize)
	gl.Vertex3f(halfSize, halfSize, -halfSize)

	// Bottom face
	gl.Vertex3f(-halfSize, -halfSize, -halfSize)
	gl.Vertex3f(halfSize, -halfSize, -halfSize)
	gl.Vertex3f(halfSize, -halfSize, halfSize)
	gl.Vertex3f(-halfSize, -halfSize, halfSize)

	// Right face
	gl.Vertex3f(halfSize, -halfSize, -halfSize)
	gl.Vertex3f(halfSize, halfSize, -halfSize)
	gl.Vertex3f(halfSize, halfSize, halfSize)
	gl.Vertex3f(halfSize, -halfSize, halfSize)

	// Left face
	gl.Vertex3f(-halfSize, -halfSize, -halfSize)
	gl.Vertex3f(-halfSize, -halfSize, halfSize)
	gl.Vertex3f(-halfSize, halfSize, halfSize)
	gl.Vertex3f(-halfSize, halfSize, -halfSize)
	gl.End()
}

func (demo *FlightSimDemo) GetTitle() string {
	return "Galileo > Flight Sim Demo"
}

// Display renders the scene.
func (demo *FlightSimDemo) Display() {
	// Clear the view port and set the camera direction
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
	gl.LoadIdentity()

	pos := demo.Aircraft.GetPosition()
	offset := internal.NewVector3(4.0+demo.Aircraft.GetVelocity().Magnitude(), 0, 0)
	offset = demo.Aircraft.GetTransform().TransformDirection(offset)

	glu.LookAt(
		float64(pos.X+offset.X), float64(pos.Y+5.0), float64(pos.Z+offset.Z),
		float64(pos.X), float64(pos.Y), float64(pos.Z),
		0.0, 1.0, 0.0,
	)

	// Draw the ground grid
	gl.Color3f(0.6, 0.6, 0.6)
	bx := int(pos.X)
	bz := int(pos.Z)

	gl.Begin(gl.QUADS)
	for x := -20; x <= 20; x++ {
		for z := -20; z <= 20; z++ {
			gl.Vertex3f(float32(bx)+float32(x)-0.1, 0, float32(bz)+float32(z)-0.1)
			gl.Vertex3f(float32(bx)+float32(x)-0.1, 0, float32(bz)+float32(z)+0.1)
			gl.Vertex3f(float32(bx)+float32(x)+0.1, 0, float32(bz)+float32(z)+0.1)
			gl.Vertex3f(float32(bx)+float32(x)+0.1, 0, float32(bz)+float32(z)-0.1)
		}
	}
	gl.End()

	// Set the transform matrix for the aircraft
	transform := demo.Aircraft.GetTransform()
	var glTransform [16]float32
	transform.FillGLArray(glTransform[:])

	gl.PushMatrix()
	gl.MultMatrixf(&glTransform[0])

	// Draw the aircraft
	gl.Color3f(0, 0, 0)
	DrawAircraft()
	gl.PopMatrix()

	// Draw shadow
	gl.Color3f(0.8, 0.8, 0.8)
	gl.PushMatrix()
	gl.Translatef(0, -1.0-pos.Y, 0)
	gl.Scalef(1.0, 0.001, 1.0)
	gl.MultMatrixf(&glTransform[0])
	DrawAircraft()
	gl.PopMatrix()

	// Display text information
	buffer1 := fmt.Sprintf("Altitude: %.1f | Speed %.1f",
		demo.Aircraft.GetPosition().Y,
		demo.Aircraft.GetVelocity().Magnitude())

	buffer2 := fmt.Sprintf("Left Wing: %.1f | Right Wing: %.1f | Rudder %.1f",
		demo.LeftWingControl, demo.RightWingControl, demo.RudderControl)

	gl.Color3f(0, 0, 0)
	demo.RenderText(10.0, 24.0, buffer1)
	demo.RenderText(10.0, 10.0, buffer2)
}

// Update updates the simulation state.
func (demo *FlightSimDemo) Update() {
	// Find the duration of the last frame in seconds
	duration := float32(GetTimingData().lastFrameDuration.Milliseconds()) * 0.001
	if duration <= 0.0 {
		return
	}

	// Start with no forces or acceleration
	demo.Aircraft.ClearAccumulators()

	// Add the propeller force
	propulsion := internal.NewVector3(-10.0, 0, 0)
	propulsion = demo.Aircraft.GetTransform().TransformDirection(propulsion)
	demo.Aircraft.AddForce(propulsion)

	// Add the forces acting on the aircraft
	demo.Registry.UpdateForces(duration)

	// Update the aircraft's physics
	demo.Aircraft.Integrate(duration)

	// Do a very basic collision detection and response with the ground
	pos := demo.Aircraft.GetPosition()
	if pos.Y < 0.0 {
		pos.Y = 0.0
		demo.Aircraft.SetPosition(pos.X, pos.Y, pos.Z)

		if demo.Aircraft.GetVelocity().Y < -10.0 {
			demo.ResetPlane()
		}
	}

	// Call the base class update
	demo.Application.Update()
}

// Key handles keyboard input.
func (demo *FlightSimDemo) Key(key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	// Only handle key press events
	if action != glfw.Press && action != glfw.Repeat {
		return
	}

	switch key {
	case glfw.KeyQ:
		demo.RudderControl += 0.1

	case glfw.KeyE:
		demo.RudderControl -= 0.1

	case glfw.KeyW:
		demo.LeftWingControl -= 0.1
		demo.RightWingControl -= 0.1

	case glfw.KeyS:
		demo.LeftWingControl += 0.1
		demo.RightWingControl += 0.1

	case glfw.KeyD:
		demo.LeftWingControl -= 0.1
		demo.RightWingControl += 0.1

	case glfw.KeyA:
		demo.LeftWingControl += 0.1
		demo.RightWingControl -= 0.1

	case glfw.KeyX:
		demo.LeftWingControl = 0.0
		demo.RightWingControl = 0.0
		demo.RudderControl = 0.0

	case glfw.KeyR:
		demo.ResetPlane()

	default:
		// Let the base application handle other keys
		demo.Application.Key(key, scancode, action, mods)
	}

	// Make sure the controls are in range
	if demo.LeftWingControl < -1.0 {
		demo.LeftWingControl = -1.0
	} else if demo.LeftWingControl > 1.0 {
		demo.LeftWingControl = 1.0
	}

	if demo.RightWingControl < -1.0 {
		demo.RightWingControl = -1.0
	} else if demo.RightWingControl > 1.0 {
		demo.RightWingControl = 1.0
	}

	if demo.RudderControl < -1.0 {
		demo.RudderControl = -1.0
	} else if demo.RudderControl > 1.0 {
		demo.RudderControl = 1.0
	}

	// Update the control surfaces
	demo.LeftWing.SetControl(demo.LeftWingControl)
	demo.RightWing.SetControl(demo.RightWingControl)
	demo.Rudder.SetControl(demo.RudderControl)
}

// Main entry point
func main() {
	app := NewFlightSimDemo()
	RunApplication(&app.Application, 800, 600)
}
