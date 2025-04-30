package internal

import "math"

var GRAVITY = NewVector3(0, -9.81, 0)

// Represents a 3D vector with x, y, z components.
// It can be used for representing positions, directions, velocities, and other
// physical quantities in 3D space. All components are stored using the Real type
// for consistent precision throughout calculations.
type Vector struct {
	X Real // Holds the value along the x axis
	Y Real // Holds the value along the y axis
	Z Real // Holds the value along the z axis
	// pad Real // Padding to ensure four word alignment
}

func NewVector3(x, y, z Real) *Vector {
	return &Vector{
		X: x,
		Y: y,
		Z: z,
	}
}

// Creates a new Vector with all components set to zero.
func NewZeroVector() *Vector {
	return &Vector{}
}

// Flips all componentts of the vector to their negative values.
// This modifies the vector in-place.
func (v *Vector) Invert() {
	v.X = -v.X
	v.Y = -v.Y
	v.Z = -v.Z
}

// Calculates the length (magnitude) of the vector using the Euclidean norm: sqrt(x² + y² + z²)
func (v *Vector) Magnitude() Real {
	return Real(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}

// Returns the squared length of this vector.
// This is faster than magnitude as it avoids the square root
func (v *Vector) SquareMagnitude() Real {
	return v.X*v.X + v.Y*v.Y + v.Z*v.Z
}

// Returns a new normalized vector (a unit vector in the same direction) as this vector.
// A normalized vector has a magnitude of 1.0. If the vector has zero magnitude, it returns a zero vector
func (v *Vector) Normalize() *Vector {
	mag := v.Magnitude()

	if mag > 0 {
		return &Vector{
			X: v.X / mag,
			Y: v.Y / mag,
			Z: v.Z / mag,
		}
	}

	return v
}

// Modifies this vector to have a magnitide of 1.0.
// Modifoes the vector in-place for better performance when creating a new vector
// instance is not desired
func (v *Vector) NormalizeInPlace() {
	mag := v.Magnitude()
	if mag > 0 {
		v.X /= mag
		v.Y /= mag
		v.Z /= mag
	}
}

// Multiplies all the components of this vector by the given scalar and returns a new Vector.
// This operation does not modify the original vector. Geomatrically, multiplication of a vector
// by a scalar changes the length of the vector, but not the direction.
func (v *Vector) Scale(scalar Real) *Vector {
	return &Vector{
		X: v.X * scalar,
		Y: v.Y * scalar,
		Z: v.Z * scalar,
	}
}

// Multiplies all components of this vector by the given scalar value, modifying the vector
// in-place.
func (v *Vector) ScaleInPlace(scalar Real) {
	v.X *= scalar
	v.Y *= scalar
	v.Z *= scalar
}

// Returns a new vector that is the sum of this vector and the given vector.
// Vector addition is performed component-wise and creates a new Vector instance.
func (v *Vector) Add(other *Vector) *Vector {
	return &Vector{
		X: v.X + other.X,
		Y: v.Y + other.Y,
		Z: v.Z + other.Z,
	}
}

// Adds the components of the given vector to this vector, modifying the vector in-place.
func (v *Vector) AddInPlace(other *Vector) {
	v.X += other.X
	v.Y += other.Y
	v.Z += other.Z
}

// Returns a new vector that is the result of subtracting the given vector from this vector.
// Vector subtraction is performed component-wise and creates a new Vector instance.
func (v *Vector) Subtract(other *Vector) *Vector {
	return &Vector{
		X: v.X - other.X,
		Y: v.Y - other.Y,
		Z: v.Z - other.Z,
	}
}

// Subtracts the components of the given vector from this vector, modifying the vector in-place.
func (v *Vector) SubtractInPlace(other *Vector) {
	v.X -= other.X
	v.Y -= other.Y
	v.Z -= other.Z
}

// Adds another vector to this vector after scaling it by the given scale.
// This operation performs the addition in-place, modifying the current vector.
func (v *Vector) AddScaledVector(vector *Vector, scale Real) {
	v.X += vector.X * scale
	v.Y += vector.Y * scale
	v.Z += vector.Z * scale
}

// Calculates and returns a new vector that is the component-wise product of this vector with another
// vector. Each component in the resulting vector is the product of the corresponding components from both vectors.
func (v *Vector) ComponentProduct(other Vector) *Vector {
	return &Vector{
		X: v.X * other.X,
		Y: v.Y * other.Y,
		Z: v.Z * other.Z,
	}
}

// Performs a component-wise product with another vector and stores the result in this vector. Each component of this
// vector is multiplied by the corresponding component of the other vector.
func (v *Vector) ComponentProductInPlace(other Vector) {
	v.X *= other.X
	v.Y *= other.Y
	v.Z *= other.Z
}

// Calculates and returns the scalar(dot) product of this vector with another vector. THe dot product is defined as the
// sum of the products of corresponding components: (x₁·x₂ + y₁·y₂ + z₁·z₂).
// The dot product has several geometric interpretations:
// - It represents the cosine of the angle between vectors(when normalized)
// - It can be used to find the angle b/w vectors.
// - It helps determine if vectors are perpendicular (dot product = 0)
// - It projects one vector into another
func (v *Vector) ScalarProduct(other *Vector) Real {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Calculates and returns the vector (cross) product of this vector with another vector.
// The cross product is a vector perpendicular to both input vectors.
// The resultant vector follows the right-hand rule, where:
//   - X = y₁·z₂ - z₁·y₂
//   - Y = z₁·x₂ - x₁·z₂
//   - Z = x₁·y₂ - y₁·x₂
//
// The cross product has several geomatric interpretations:
// - The resulting vector is perpendiculat to both input vectors.
// - Its magnitude is the area of the parallelogram formed by the vectors.
// - The direction follows the right-hand rule
func (v *Vector) VectorProduct(other *Vector) *Vector {
	return &Vector{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}
}

func (v *Vector) VectorProductInPlace(other Vector) {
	temp := Vector{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}

	*v = temp
}

// An alias for VectorProduct that provides a shorter name for this commonly used operation.
func (v *Vector) Cross(other *Vector) *Vector {
	return v.VectorProduct(other)
}
