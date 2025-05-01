package demos

import (
	"fmt"
	"image"
	"image/color"
	"os"
	"runtime"
	"time"

	"github.com/go-gl/gl/v2.1/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/golang/freetype"
	"github.com/golang/freetype/truetype"
	"github.com/harpreetsingh/galileo/src/internal"
)

// Initialize GLFW
func init() {
	// This necessary for Opengl to work on OSX
	// Go's OpenGL bindings required all OpenGL calls to be made on the same thread
	// that created the OpenGL context. This is important because OpenGL is inherently single threaded.
	runtime.LockOSThread()
}

// The Base class for all demonstration programs.
// It provides common functionality and a standard interface for all demos.
type Application struct {
	// Window Dimensions
	Height int
	Width  int

	// Reference to GLFW window - replaces GLUT window handling
	Window *glfw.Window

	font     *truetype.Font
	fontSize float64
}

func (app *Application) GetTitle() string {
	return "Galileo Demo"
}

// Sets up the graphics, and allows the application to acquire graphical resources. Guaranteed
// to be called afrer OpenGL is set up.
func (app *Application) InitGraphics() {
	// Set the background color (light blue)
	gl.ClearColor(0.9, 0.95, 1.0, 1.0)
	// Enable depth testing for proper 3d rendering
	gl.Enable(gl.DEPTH_TEST)
	// Set the shading model to smooth
	gl.ShadeModel(gl.SMOOTH)
	// Set up the camera view
	app.SetView()
}

// Sets up the camera projection characterstics. This configures the perspective projection matrix
// for the 3D scene.
func (app *Application) SetView() {
	// Switch to projection matrix mode
	gl.MatrixMode(gl.PROJECTION)
	gl.LoadIdentity()

	// Set up a perspective projection with 60-degree field of view. The near clipping plane is at
	// 1.0 and far plane at 500.0
	// glu.Perspective(60.0, float64(app.Width)/float64(app.Height), 1.0, 500.0)

	// Switch back to modelview matrix for normal rendering
	gl.MatrixMode(gl.MODELVIEW)
}

// Called each frame to display the current scene. This simple implementation just draws a diagonal
// line.
func (app *Application) Display() {
	// Clear the color buffer
	gl.Clear(gl.COLOR_BUFFER_BIT)

	// Draw a simple diagonal line as a sanity check
	gl.Begin(gl.LINES)
	gl.Vertex2i(1, 1)
	gl.Vertex2i(639, 319)
	gl.End()
}

// Update is called each frame to update the current state of the scene.
// Unlike in C++ where we need to call glutPostRedisplay(), in Go with GLFW
// the rendering happens in our controlled main loop.
func (app *Application) Update() {
	// Default implementation is empty
	// In GLUT/C++ this would post a redisplay request
}

func (app *Application) Key(key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {

}

// Notifies the application that the window has changed size. Updates the viewport and projection
// matrix accordingly.
func (app *Application) Resize(width, height int) {
	// Avoid divide by zero errors
	if height <= 0 {
		height = 1
	}

	// Store the new dimensions
	app.Width = width
	app.Height = height

	// Update the OpenGL viewport to match the new window size
	gl.Viewport(0, 0, int32(width), int32(height))

	// Update the projection matrix for the new aspect ratio
	app.SetView()
}

func (app *Application) MouseButton(button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
}

func (app *Application) MouseDrag(x, y float64) {}

// Renders the given text to the given x, y location on the window. This is a utility function
// to display status information.
func (app *Application) RenderText(x, y float32, text string) {
	// Disable depth testing temporarily so text appears on top
	gl.Disable(gl.DEPTH_TEST)

	// Set up orthographic projection for 2D text rendering
	gl.MatrixMode(gl.PROJECTION)
	gl.PushMatrix()
	gl.LoadIdentity()
	gl.Ortho(0.0, float64(app.Width), 0.0, float64(app.Height), -1.0, 1.0)

	// Switch to modelview matrix for positioning the text
	gl.MatrixMode(gl.MODELVIEW)
	gl.PushMatrix()
	gl.LoadIdentity()

	// Set raster position for bitmap rendering
	currentX := x
	currentY := y

	// Create bitmap for rendering
	textImage := app.CreateTextBitmap(text)
	if textImage == nil {
		// Pop the matrices and return if text bitmap creation failed
		gl.PopMatrix()
		gl.MatrixMode(gl.PROJECTION)
		gl.PopMatrix()
		gl.MatrixMode(gl.MODELVIEW)
		gl.Enable(gl.DEPTH_TEST)
		return
	}

	// Create and bind texture
	var texture uint32
	gl.GenTextures(1, &texture)
	gl.BindTexture(gl.TEXTURE_2D, texture)

	// Set texture parameters
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)

	// Upload image data to texture
	gl.TexImage2D(gl.TEXTURE_2D, 0, gl.RGBA, int32(textImage.Bounds().Dx()), int32(textImage.Bounds().Dy()), 0, gl.RGBA, gl.UNSIGNED_BYTE, gl.Ptr(textImage.Pix))

	// Enable texturing and blending
	gl.Enable(gl.TEXTURE_2D)
	gl.Enable(gl.BLEND)
	gl.BlendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

	// Draw textured quad
	width := float32(textImage.Bounds().Dx())
	height := float32(textImage.Bounds().Dy())

	gl.Begin(gl.QUADS)
	gl.TexCoord2f(0, 0)
	gl.Vertex2f(currentX, currentY)

	gl.TexCoord2f(1, 0)
	gl.Vertex2f(currentX+width, currentY)

	gl.TexCoord2f(1, 1)
	gl.Vertex2f(currentX+width, currentY+height)

	gl.TexCoord2f(0, 1)
	gl.Vertex2f(currentX, currentY+height)

	gl.End()

	// Disable texturing and blending
	gl.Disable(gl.BLEND)
	gl.Disable(gl.TEXTURE_2D)

	// Clean up
	gl.DeleteTextures(1, &texture)

	// Pop the matrices to return to how we were before
	gl.PopMatrix()
	gl.MatrixMode(gl.PROJECTION)
	gl.PopMatrix()
	gl.MatrixMode(gl.MODELVIEW)

	gl.Enable(gl.DEPTH_TEST)
}

func (app *Application) CreateTextBitmap(text string) *image.RGBA {
	// Calculate size needed for text
	// This is a simplification - in a real app you'd calculate exact bounds
	width := len(text) * int(app.fontSize) * 3 / 4
	height := int(app.fontSize * 1.5)

	// Create image for text
	img := image.NewRGBA(image.Rect(0, 0, width, height))

	// Set background transparent
	for y := 0; y < height; y++ {
		for x := 0; x < width; x++ {
			img.Set(x, y, color.RGBA{0, 0, 0, 0})
		}
	}

	// Create freetype context
	c := freetype.NewContext()
	c.SetDPI(72)
	c.SetFont(app.font)
	c.SetFontSize(app.fontSize)
	c.SetClip(img.Bounds())
	c.SetDst(img)
	c.SetSrc(image.White)

	// Process text line by line
	pt := freetype.Pt(0, int(c.PointToFixed(app.fontSize)>>6))
	lineHeight := int(app.fontSize * 1.2)

	// Split text by newline
	lines := []string{text}
	for i, line := range lines {
		if i > 0 {
			pt.Y += c.PointToFixed(float64(lineHeight))
		}

		_, err := c.DrawString(line, pt)
		if err != nil {
			fmt.Fprintf(os.Stderr, "Error drawing text: %v\n", err)
			return nil
		}
	}

	return img
}

// Manages timing information for the application. This is a singleton clss that tracks frame for animation and physics.
type TimingData struct {
	lastFrameTimestamp time.Time
	lastFrameDuration  time.Duration
}

// Returns the singleton instance of TimingData. This implements singleton pattern in Go.
var timingInstance TimingData

func GetTimingData() *TimingData {
	return &timingInstance
}

func (t *TimingData) Update() {
	now := time.Now()
	t.lastFrameDuration = now.Sub(t.lastFrameTimestamp)
	t.lastFrameTimestamp = now
}

func (t *TimingData) Init() {
	t.lastFrameTimestamp = time.Now()
	t.lastFrameDuration = 0
}

// Adds functionality for particle-based demos. This extends the base application with particle physics capabilities.
type MassAggregateApplication struct {
	Application
	World                  *internal.ParticleWorld
	ParticleArray          []*internal.Particle
	GroundContactGenerator *internal.GroundContacts
}

func NewMassAggregateApplication(particleCount int) *MassAggregateApplication {
	app := &MassAggregateApplication{
		World:         internal.NewParticleWorld(uint(particleCount*10), 100),
		ParticleArray: make([]*internal.Particle, particleCount),
	}

	// Add all particles to the world
	for i := range app.ParticleArray {
		app.World.SetParticle(app.ParticleArray[i])
	}

	// Initialize the ground contact generator and add it to the world
	app.GroundContactGenerator.Init(app.World.GetParticles())
	app.World.SetContactGenerator(app.GroundContactGenerator)

	return app
}

// Updates the particle positions.
// This overides the base application.Update method
func (app *MassAggregateApplication) Update() {
	// Clear accumulators to start the physics calculations fresh
	app.World.StartFrame()

	// Calculate frame duration in seconds
	duration := float32(GetTimingData().lastFrameDuration.Milliseconds()) * 0.001
	if duration <= 0.0 {
		return // Skip physics if time has'nt advanced
	}

	// Run the physics simulation for this frame
	app.World.RunPhysics(internal.Real(duration))

	// Call the base class Update
	RunApplication(&app.Application, app.Width, app.Height)
}

// Displays the particle. This overides the base Application.Display method.
func (app *MassAggregateApplication) Display() {
	// Clear the screen and depth buffer
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	// Set up the camera
	gl.LoadIdentity()
	// glu.LookAt(0.0, 3.5, 8.0, 0.0, 3.5, 0.0, 0.0, 1.0, 0.0)

	// Set drawing color to black
	gl.Color3f(0, 0, 0)

	// Draw each particle in the world
	particles := app.World.GetParticles()
	for _, particle := range particles {
		pos := particle.GetPosition()
		gl.PushMatrix()
		gl.Translatef(float32(pos.X), float32(pos.Y), float32(pos.Z))

		drawCube(0.1)
		gl.PopMatrix()

	}

}

func drawCube(size float32) {
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

func RunApplication(app *Application, width, height int) {
	// Initialize GLFW
	if err := glfw.Init(); err != nil {
		panic(fmt.Errorf("failed to initialize glfw: %v", err))
	}
	defer glfw.Terminate() // Ensure we clean up

	// Configure OpenGL context
	glfw.WindowHint(glfw.ContextVersionMajor, 2)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)

	// Create the window
	window, err := glfw.CreateWindow(width, height, app.GetTitle(), nil, nil)
	if err != nil {
		panic(fmt.Errorf("failed to create window: %v", err))
	}

	// Store window reference and make its context current
	app.Window = window
	window.MakeContextCurrent()

	// Initialize OpenGL
	if err := gl.Init(); err != nil {
		panic(fmt.Errorf("failed to initialize OpenGL: %v", err))
	}

	// Initialize timing system
	GetTimingData().Init()

	// Set up event callbacks

	// Window resize event
	window.SetFramebufferSizeCallback(func(w *glfw.Window, width, height int) {
		app.Resize(width, height)
	})

	// Keyboard event
	window.SetKeyCallback(func(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
		app.Key(key, scancode, action, mods)
	})

	// Mouse button event
	window.SetMouseButtonCallback(func(w *glfw.Window, button glfw.MouseButton, action glfw.Action, mods glfw.ModifierKey) {
		app.MouseButton(button, action, mods)
	})

	// Mouse movement event
	window.SetCursorPosCallback(func(w *glfw.Window, xpos, ypos float64) {
		// Only consider it dragging if a mouse button is pressed
		// This mimics the behavior of GLUT's mouseDrag
		leftState := window.GetMouseButton(glfw.MouseButtonLeft)
		rightState := window.GetMouseButton(glfw.MouseButtonRight)
		if leftState == glfw.Press || rightState == glfw.Press {
			app.MouseDrag(xpos, ypos)
		}
	})

	// Initialize application graphics
	app.InitGraphics()

	// Main application loop
	for !window.ShouldClose() {
		// Update timing information
		GetTimingData().Update()

		// Update physics and state
		app.Update()

		// Render the scene
		app.Display()

		// Swap buffers and process events
		window.SwapBuffers()
		glfw.PollEvents()
	}

	// Clean up
	// app.Deinit()
}
