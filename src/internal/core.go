package internal

import "math"

var GRAVITY = NewVector3(0, -9.81, 0)
var UP = NewVector3(0, 1, 0)
var SleepEpsilon = Real(0.3)

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

// Zero all components of the vector.
func (v *Vector) Clear() {
	v.X = 0
	v.Y = 0
	v.Z = 0
}

// Holds a 3*3 row major matrix representing a transformation in 3D space that does not include
// a translational component. This matrix is not padded to produce an aligned structure.
type Matrix3 struct {
	Data [9]float32
}

// Transforms the given vector by this matrix. The transformation applies the matrix to the vector using
// standard matrix-vector multiplication rules, where each component of the resulting vector is the dot
// product of a row of the matrix with the input vector.
func (m *Matrix3) MultiplyVector(vector *Vector) *Vector {
	return NewVector3(
		vector.X*Real(m.Data[0])+vector.Y*Real(m.Data[1])+vector.Z*Real(m.Data[2]),
		vector.X*Real(m.Data[3])+vector.Y*Real(m.Data[4])+vector.Z*Real(m.Data[5]),
		vector.X*Real(m.Data[6])+vector.Y*Real(m.Data[7])+vector.Z*Real(m.Data[8]),
	)
}

// Transforms the given vector by this matrix. This method is a convenience wrapper around MultiplyVector
// and provides a more descriptive name for the operation being performed.
func (m *Matrix3) Transform(vector *Vector) *Vector {
	return m.MultiplyVector(vector)
}

func (m *Matrix3) TransformTranspose(vector *Vector) *Vector {
	return NewVector3(
		vector.X*Real(m.Data[0])+vector.Y*Real(m.Data[3])+vector.Z*Real(m.Data[6]),
		vector.X*Real(m.Data[1])+vector.Y*Real(m.Data[4])+vector.Z*Real(m.Data[7]),
		vector.X*Real(m.Data[2])+vector.Y*Real(m.Data[5])+vector.Z*Real(m.Data[8]),
	)
}

// Returns a new matrix that is the result of multiplying this matrix by another matrix. Matrix multiplication is
// performed using the standard mathematical definition: For each element (i, j) in the result matrix, we compute
// the dot product of the i-th row of this matrix with the j-th column of the other matrix.
func (m *Matrix3) Multiply(other *Matrix3) *Matrix3 {
	return &Matrix3{
		Data: [9]float32{
			m.Data[0]*other.Data[0] + m.Data[1]*other.Data[3] + m.Data[2]*other.Data[6],
			m.Data[0]*other.Data[1] + m.Data[1]*other.Data[4] + m.Data[2]*other.Data[7],
			m.Data[0]*other.Data[2] + m.Data[1]*other.Data[5] + m.Data[2]*other.Data[8],

			m.Data[3]*other.Data[0] + m.Data[4]*other.Data[3] + m.Data[5]*other.Data[6],
			m.Data[3]*other.Data[1] + m.Data[4]*other.Data[4] + m.Data[5]*other.Data[7],
			m.Data[3]*other.Data[2] + m.Data[4]*other.Data[5] + m.Data[5]*other.Data[8],

			m.Data[6]*other.Data[0] + m.Data[7]*other.Data[3] + m.Data[8]*other.Data[6],
			m.Data[6]*other.Data[1] + m.Data[7]*other.Data[4] + m.Data[8]*other.Data[7],
			m.Data[6]*other.Data[2] + m.Data[7]*other.Data[5] + m.Data[8]*other.Data[8],
		},
	}
}

// Multiplies this matrix by another matrix and stores the result in this matrix.
func (m *Matrix3) MultiplyInPlace(other *Matrix3) {
	var t1, t2, t3 float32

	t1 = m.Data[0]*other.Data[0] + m.Data[1]*other.Data[3] + m.Data[2]*other.Data[6]
	t2 = m.Data[0]*other.Data[1] + m.Data[1]*other.Data[4] + m.Data[2]*other.Data[7]
	t3 = m.Data[0]*other.Data[2] + m.Data[1]*other.Data[5] + m.Data[2]*other.Data[8]

	m.Data[0] = t1
	m.Data[1] = t2
	m.Data[2] = t3

	t1 = m.Data[3]*other.Data[0] + m.Data[4]*other.Data[3] + m.Data[5]*other.Data[6]
	t2 = m.Data[3]*other.Data[1] + m.Data[4]*other.Data[4] + m.Data[5]*other.Data[7]
	t3 = m.Data[3]*other.Data[2] + m.Data[4]*other.Data[5] + m.Data[5]*other.Data[8]

	m.Data[3] = t1
	m.Data[4] = t2
	m.Data[5] = t3

	t1 = m.Data[6]*other.Data[0] + m.Data[7]*other.Data[3] + m.Data[8]*other.Data[6]
	t2 = m.Data[6]*other.Data[1] + m.Data[7]*other.Data[4] + m.Data[8]*other.Data[7]
	t3 = m.Data[6]*other.Data[2] + m.Data[7]*other.Data[5] + m.Data[8]*other.Data[8]

	m.Data[6] = t1
	m.Data[7] = t2
	m.Data[8] = t3
}

// Sets this matrix to be the inverse of the given matrix. The inverse of a matrix M is another M' such that M * M' = I,
// where I is the identity matrix. The inverse is calculated using the classical adjoint formula:
// M' = adj(M) / det(M) where adj(M) is the adjoint matrix and det(M) is the determinant.
// For a 3*3 matrix, the calculation involves computing factors and the determinant. Several temporary variables
// are used to avoid redundant calculations and improve performance. If the determinant is zero, the matrix is singular (non-invertible),
// and this method will return without changing the matrix.
func (mat *Matrix3) SetInverse(m *Matrix3) {
	// Calculate intermediate terms used multiple times in the calculation
	t1 := m.Data[0] * m.Data[4]
	t2 := m.Data[0] * m.Data[5]
	t3 := m.Data[1] * m.Data[3]
	t4 := m.Data[2] * m.Data[3]
	t5 := m.Data[1] * m.Data[6]
	t6 := m.Data[2] * m.Data[6]

	// Calculate the determinant
	det := (t1*m.Data[8] - t2*m.Data[7] - t3*m.Data[8] + t4*m.Data[7] + t5*m.Data[5] - t6*m.Data[4])

	// Make sure the determinant is non-zero (matrix is invertible)
	if det == 0.0 {
		return
	}

	// Calculate the inverse determinant once to avoid division in each element
	invd := 1.0 / det

	// Calculate each element of the inverse matrix using cofactors, transposing as we go
	mat.Data[0] = (m.Data[4]*m.Data[8] - m.Data[5]*m.Data[7]) * invd
	mat.Data[1] = -(m.Data[1]*m.Data[8] - m.Data[2]*m.Data[7]) * invd
	mat.Data[2] = (m.Data[1]*m.Data[5] - m.Data[2]*m.Data[4]) * invd

	mat.Data[3] = -(m.Data[3]*m.Data[8] - m.Data[5]*m.Data[6]) * invd
	mat.Data[4] = (m.Data[0]*m.Data[8] - t6) * invd
	mat.Data[5] = -(t2 - t4) * invd

	mat.Data[6] = -(m.Data[3]*m.Data[7] - m.Data[4]*m.Data[6]) * invd
	mat.Data[7] = (m.Data[0]*m.Data[7] - t5) * invd
	mat.Data[8] = (t1 - t3) * invd
}

// Returns a new matrix containing the inverse of this matrix.
func (mat *Matrix3) Inverse() *Matrix3 {
	var result *Matrix3
	result.SetInverse(mat)

	return result
}

// Sets this matrix to be the transpose of the given matrix.
// The transpose of a matrix is obtained by flipping the matrix over its main diagonal, switching the row and column
// indices of each element. For a 3*3 matrix, the element at position (i,j) in the original matrix becomes the element
// at position (j,i) in the transpose.
func (mat *Matrix3) SetTranspose(m *Matrix3) {
	mat.Data[0] = m.Data[0]
	mat.Data[1] = m.Data[3]
	mat.Data[2] = m.Data[6]
	mat.Data[3] = m.Data[1]
	mat.Data[4] = m.Data[4]
	mat.Data[5] = m.Data[7]
	mat.Data[6] = m.Data[2]
	mat.Data[7] = m.Data[5]
	mat.Data[8] = m.Data[8]
}

func (mat *Matrix3) Transpose() *Matrix3 {
	var result *Matrix3
	result.SetTranspose(mat)
	return result
}

// Sets this matrix to be the rotation matrix corresponding to the given quaternion. A Quaternion represents a rotation in 3D space.
// Converting a quaternion to a rotation matrix allows that rotation allows that rotation to be applied to vectors using matrix multiplication.
// This is often more efficient than using quarternions directly for repeated transformations of multiple points.
//
// The conversion uses the following formula for a unit quaternion q = [r, i, j ,k]:
//
// [ 1-2(j² + k²) 2(ij + kr)   2(ik - jr)   ]
// [ 2(ij - kr)   1-2(i² + k²) 2(jk + ir)   ]
// [ 2(ik + jr)   2(jk - ir)   1-2(i² + j²) ]
//
// This formula assumes the quaternion is normalized (has unit length). If working with non-normalized quaternions, you should normalize them
// before calling this method for correct results.
//
// The resulting matrix can be used to transform vectors by multiplying them by this matrix. The transformation will rotate the vectors according
// to the rotation represented by the quaternion.
func (mat *Matrix3) SetOrientation(q *Quaternion) {
	// First row
	mat.Data[0] = 1 - 2*(q.J*q.J+q.K*q.K)
	mat.Data[1] = 2 * (q.I*q.J + q.K*q.R)
	mat.Data[2] = 2 * (q.I*q.K - q.J*q.R)

	mat.Data[3] = 2 * (q.I*q.J - q.K*q.R)
	mat.Data[4] = 1 - 2*(q.I*q.I+q.K*q.K)
	mat.Data[5] = 2 * (q.J*q.K + q.I*q.R)

	mat.Data[6] = 2 * (q.I*q.K + q.J*q.R)
	mat.Data[7] = 2 * (q.J*q.K - q.I*q.R)
	mat.Data[8] = 1 - 2*(q.I*q.I+q.J*q.J)
}

// Sets the matrix values to represent the inertia tensor of a rectangular block aligned with the body's coordinates system.
//
// This method calculates the inertia tensor of a rectangular cubiod using the standard formulas for a uniform density block:
// Ixx = m/3 * (y² + z²)
// Ixx = m/3 * (x² + z²)
// Ixx = m/3 * (x² + y²)
// Params:
//   - halfSizes: The half-extents of the block along each axis
//   - mass: The total mass of the block
func (m *Matrix3) SetBlockInertiaTensor(halfSizes *Vector, mass float32) {
	// Calculate the squared components
	squares := halfSizes.ComponentProduct(*halfSizes)

	// Set the diagonal elements of the inertia tensor. The 0.3 factor comes from mass/3 in the standard formulas
	m.SetInertiaTensorCoeffs(
		0.3*mass*float32(squares.Y+squares.Z),
		0.3*mass*float32(squares.X+squares.Z),
		0.3*mass*float32(squares.X+squares.Y),
	)

}

func (m *Matrix3) SetInertiaTensorCoeffs(ix, iy, iz float32) {
	m.Data[0] = ix
	m.Data[1] = -0
	m.Data[3] = -0
	m.Data[2] = -0
	m.Data[6] = -0
	m.Data[4] = iy
	m.Data[5] = -0
	m.Data[7] = -0
	m.Data[8] = iz
}

func (m *Matrix3) SetComponents(compOne *Vector, compTwo *Vector, compThree *Vector) {
	m.Data[0] = float32(compOne.X)
	m.Data[1] = float32(compTwo.X)
	m.Data[2] = float32(compThree.X)
	m.Data[3] = float32(compOne.Y)
	m.Data[4] = float32(compTwo.Y)
	m.Data[5] = float32(compThree.Y)
	m.Data[6] = float32(compOne.Z)
	m.Data[7] = float32(compTwo.Z)
	m.Data[8] = float32(compThree.Z)
}

// Performs a linear interpolation b/w two matrices.
//
// Creates a new matrx by linearly interpolating b/w each corresponding element of the source matrices. The interpolation
// is controlled by the prop parameter:
// - When prop = 0: Returns matrix a
// - When prop = 1: Returns matrix b
// - When 0 < prop < 1: Returns a weighted blend of a and b
func Matrix3LinearInterpolate(a, b *Matrix3, prop float32) *Matrix3 {
	var result *Matrix3
	for i := 0; i < 9; i++ {
		result.Data[i] = a.Data[i]*(1-prop) + b.Data[i]*prop
	}

	return result
}

// Holds a transform matrix, consisting of a rotation matrix and a position. The matrix has 12
// elements, and it is assumed that the remaining four are (0,0,0,1) producing a homogeneous matrix.
type Matrix4 struct {
	Data [12]float32
}

// Transforms the given vector by this matrix. The transformation applies the matrix to the vector
// using homogeneous coordinates, which allows the matrix to represent translations in addition to linear
// transformations. This essentially a 4*4 matrix multiplication where the input vector is treated as a 4D vector
// with w=1( a point in 3D space), and the result is projected back to 3D. The matrix is assumed to be in row-major
// order with the last row being [0,0,0,1], which is not explicitly stored. This method can be useed to apply any
// affine transformation (rotation, scaling, translation, shearing, etc.) to a 3D point.
func (m *Matrix4) MultiplyVector(vector *Vector) *Vector {
	return NewVector3(
		vector.X*Real(m.Data[0])+vector.Y*Real(m.Data[1])+vector.Z*Real(m.Data[2])+Real(m.Data[3]),
		vector.X*Real(m.Data[4])+vector.Y*Real(m.Data[5])+vector.Z*Real(m.Data[6])+Real(m.Data[7]),
		vector.X*Real(m.Data[8])+vector.Y*Real(m.Data[9])+vector.Z*Real(m.Data[10])+Real(m.Data[11]),
	)
}

// This method is a convenience wrapper around MultiplyVector and provides
// a more descriptive name for the operation being performed.
func (m *Matrix4) Transform(vector *Vector) *Vector {
	return m.MultiplyVector(vector)
}

// Returns a new matrix that is the result of multiplying this matrix with another matrix.
// This method performs matrix multiplication for 4*4 transformation matrices, where the matrices
// are stored in a compact 12-element array (omitting the last row is assumed to be [0,0,0,1]).
// Matrix multiplication combines two transformations: if matrix A represents a rotation a translation. Note that
// matrix multiplication is not commutative - the order matters.
//
// The calculation handles the special case of homogenous coordinates, where:
// - The top-left 3*3 submatrix represents linear transformations (rotation, scale, shear)
// - The last column represents translation
// - The implicit last row [0,0,0,1] allows for proper concatenation of transformations.
//
// The matrices are conceptually arranged as:
//
//	 This matrix (m):           Other matrix (o):
//		[ m[0]  m[1]  m[2]  m[3] ] [ o[0]  o[1]  o[2]  o[3] ]
//		[ m[4]  m[5]  m[6]  m[7] ] [ o[4]  o[5]  o[6]  o[7] ]
//		[ m[8]  m[9]  m[10] m[11]] [ o[8]  o[9]  o[10] o[11]]
//		[ 0     0     0     1    ] [ 0     0     0     1    ]
func (m *Matrix4) Multiply(other *Matrix4) *Matrix4 {
	result := &Matrix4{}

	// First column of result matrix
	result.Data[0] = other.Data[0]*m.Data[0] + other.Data[4]*m.Data[1] + other.Data[8]*m.Data[2]
	result.Data[4] = other.Data[0]*m.Data[4] + other.Data[4]*m.Data[5] + other.Data[6]*m.Data[6]
	result.Data[8] = other.Data[0]*m.Data[8] + other.Data[4]*m.Data[9] + other.Data[8]*m.Data[10]

	// Second column of result matrix
	result.Data[1] = other.Data[1]*m.Data[0] + other.Data[5]*m.Data[1] + other.Data[9]*m.Data[2]
	result.Data[5] = other.Data[1]*m.Data[4] + other.Data[5]*m.Data[5] + other.Data[9]*m.Data[6]
	result.Data[9] = other.Data[1]*m.Data[8] + other.Data[5]*m.Data[9] + other.Data[9]*m.Data[10]

	// Third column of result matrix
	result.Data[2] = other.Data[2]*m.Data[0] + other.Data[6]*m.Data[1] + other.Data[10]*m.Data[2]
	result.Data[6] = other.Data[2]*m.Data[4] + other.Data[6]*m.Data[5] + other.Data[10]*m.Data[6]
	result.Data[10] = other.Data[2]*m.Data[8] + other.Data[6]*m.Data[9] + other.Data[10]*m.Data[10]

	// Fourth column of result matrix (translation component)
	result.Data[3] = other.Data[3]*m.Data[0] + other.Data[7]*m.Data[1] + other.Data[11]*m.Data[2] + m.Data[3]
	result.Data[7] = other.Data[3]*m.Data[4] + other.Data[7]*m.Data[5] + other.Data[11]*m.Data[6] + m.Data[7]
	result.Data[11] = other.Data[3]*m.Data[8] + other.Data[7]*m.Data[9] + other.Data[11]*m.Data[10] + m.Data[11]

	return result
}

// Returns the determinant of the matrix. The determinant is a scalar value that can be computed from the elements
// of a square matrix and encodes certain properties of the linear transformation described by the matrix. A matrix
// is invertible if and only if its determinant is not zero. For this 4*4 matrix, we're actually calculating a simplified
// determinant that works specifically for the structure of our Matrix4 class, which affine transformations with an implicit bottom
// row of [0,0,0,1].
func (mat *Matrix4) GetDeterminant() float32 {
	return mat.Data[8]*mat.Data[5]*mat.Data[2] +
		mat.Data[4]*mat.Data[9]*mat.Data[2] +
		mat.Data[8]*mat.Data[1]*mat.Data[6] -
		mat.Data[0]*mat.Data[9]*mat.Data[6] -
		mat.Data[4]*mat.Data[1]*mat.Data[10] +
		mat.Data[0]*mat.Data[5]*mat.Data[10]
}

// Sets this matrix to be the inverse of the given matrix. The inverse of a matrix M is another matrix
func (mat *Matrix4) SetInverse(m *Matrix4) {
	// Make sure the determinant is non-zero
	det := m.GetDeterminant()
	if det == 0 {
		return
	}

	// Calculate the inverse determinant once to avoid division in each element.
	det = 1.0 / det

	// First row
	mat.Data[0] = (-m.Data[9]*m.Data[6] + m.Data[5]*m.Data[10]) * det
	mat.Data[4] = (m.Data[8]*m.Data[6] - m.Data[4]*m.Data[10]) * det
	mat.Data[8] = (-m.Data[8]*m.Data[5] + m.Data[4]*m.Data[9]) * det

	// Second row
	mat.Data[1] = (m.Data[9]*m.Data[2] - m.Data[1]*m.Data[10]) * det
	mat.Data[5] = (-m.Data[8]*m.Data[2] + m.Data[0]*m.Data[10]) * det
	mat.Data[9] = (m.Data[8]*m.Data[1] - m.Data[0]*m.Data[9]) * det

	// Third row
	mat.Data[2] = (-m.Data[5]*m.Data[2] + m.Data[1]*m.Data[6]) * det
	mat.Data[6] = (+m.Data[4]*m.Data[2] - m.Data[0]*m.Data[6]) * det
	mat.Data[10] = (-m.Data[4]*m.Data[1] + m.Data[0]*m.Data[5]) * det

	// Fourth row (translation components)
	// These have more complex formulas due to the affine transformation structure
	mat.Data[3] = (m.Data[9]*m.Data[6]*m.Data[3] -
		m.Data[5]*m.Data[10]*m.Data[3] -
		m.Data[9]*m.Data[2]*m.Data[7] +
		m.Data[1]*m.Data[10]*m.Data[7] +
		m.Data[5]*m.Data[2]*m.Data[11] -
		m.Data[1]*m.Data[6]*m.Data[11]) * det

	mat.Data[7] = (-m.Data[8]*m.Data[6]*m.Data[3] +
		m.Data[4]*m.Data[10]*m.Data[3] +
		m.Data[8]*m.Data[2]*m.Data[7] -
		m.Data[0]*m.Data[10]*m.Data[7] -
		m.Data[4]*m.Data[2]*m.Data[11] +
		m.Data[0]*m.Data[6]*m.Data[11]) * det

	mat.Data[11] = (m.Data[8]*m.Data[5]*m.Data[3] -
		m.Data[4]*m.Data[9]*m.Data[3] -
		m.Data[8]*m.Data[1]*m.Data[7] +
		m.Data[0]*m.Data[9]*m.Data[7] +
		m.Data[4]*m.Data[1]*m.Data[11] -
		m.Data[0]*m.Data[5]*m.Data[11]) * det
}

// Returns a new matrix containing the inverse of this matrix.
func (mat *Matrix4) Inverse() *Matrix4 {
	var result *Matrix4
	result.SetInverse(mat)
	return result
}

// Inverts the matrix in place.
func (mat *Matrix4) Invert() {
	mat.SetInverse(mat)
}

// Sets this matrix to be a combined rotation and translation matrix. The rotation is specified by a quaternion and
// the translation by a position vector.
func (mat *Matrix4) SetOrientationAndPos(q *Quaternion, pos *Vector) {
	mat.Data[0] = 1 - 2*(q.J*q.J+q.K*q.K)
	mat.Data[1] = 2*(q.I*q.J) + 2*q.K*q.R
	mat.Data[2] = 2*q.I*q.K - 2*q.J*q.R

	mat.Data[4] = 2*q.I*q.J - 2*q.K*q.R
	mat.Data[5] = 1 - 2*(q.I*q.I+q.K*q.K)
	mat.Data[6] = 2*q.J*q.K + 2*q.I*q.R

	mat.Data[8] = 2*q.I*q.K + 2*q.J*q.R
	mat.Data[9] = 2*q.J*q.K - 2*q.I*q.R
	mat.Data[10] = 1 - 2*(q.I*q.I+q.J*q.J)

	mat.Data[3] = float32(pos.X)
	mat.Data[7] = float32(pos.Y)
	mat.Data[11] = float32(pos.Z)
}

// Transforms a vector by the inverse of this matrix without actually without actually calculating the full inverse matrix.
// This is more efficient when you only need to transform a single vector by the inverse transformation.
// This method works by:
// 1. Subtracting the translation component (undoing the translation)
// 2. Applying the inverse of the rotation component (which is the transpose of the rotation matrix, asssuming it's orthogonal)
func (mat *Matrix4) TransformInverse(vector *Vector) *Vector {
	// First undo the translation
	tmp := Vector{
		X: vector.X - Real(mat.Data[3]),
		Y: vector.Y - Real(mat.Data[7]),
		Z: vector.Z - Real(mat.Data[11]),
	}

	return &Vector{
		X: tmp.X*Real(mat.Data[0]) + tmp.Y*Real(mat.Data[4]) + tmp.Z*Real(mat.Data[8]),
		Y: tmp.X*Real(mat.Data[1]) + tmp.Y*Real(mat.Data[5]) + tmp.Z*Real(mat.Data[9]),
		Z: tmp.X*Real(mat.Data[2]) + tmp.Y*Real(mat.Data[6]) + tmp.Z*Real(mat.Data[10]),
	}
}

// Transforms a direction vector by this matrix, ignoring the translation component.
// Direction vectors represent orientation rather than position, so translation should not be
// applied to them. This method uses only the rotational portion of the matrix to transform the vector.
// Unlike TransformInverse, this method applies the matrix  directly  to the vector without first undoing
// any translation, because direction vectors are invariant to translation.
func (mat *Matrix4) TransformDirection(vector *Vector) *Vector {
	return &Vector{
		X: vector.X*Real(mat.Data[0]) + vector.Y*Real(mat.Data[1]) + vector.Z*Real(mat.Data[2]),
		Y: vector.X*Real(mat.Data[4]) + vector.Y*Real(mat.Data[5]) + vector.Z*Real(mat.Data[6]),
		Z: vector.X*Real(mat.Data[8]) + vector.Y*Real(mat.Data[9]) + vector.Z*Real(mat.Data[10]),
	}
}

// Transforms a direction vector by the inverse of the rotational part of this matrix. For an orthogonal rotation
// matrix, the inverse is the same as the transpose. This method effectively multiplies the vector by the transpose of the 3*3
// rotation portion of the matrix.
// This is useful for transforming normals or directions from world space to local space without having to calculate the full inverse matrix.
func (mat *Matrix4) TransformInverseDirection(vector *Vector) *Vector {
	return &Vector{
		X: vector.X*Real(mat.Data[0]) + vector.Y*Real(mat.Data[4]) + vector.Z*Real(mat.Data[8]),
		Y: vector.X*Real(mat.Data[1]) + vector.Y*Real(mat.Data[5]) + vector.Z*Real(mat.Data[9]),
		Z: vector.X*Real(mat.Data[2]) + vector.Y*Real(mat.Data[6]) + vector.Z*Real(mat.Data[10]),
	}
}

// Gets a vector representing one axis (i.e one column) in the matrix.
func (mat *Matrix4) GetAxisVector(i int) *Vector {
	return NewVector3(Real(mat.Data[i]), Real(mat.Data[i+4]), Real(mat.Data[i+8]))
}

// Transforms a point from local space to world space using the given transformation matrix.
func LocalToWorld(local *Vector, transform *Matrix4) *Vector {
	return transform.Transform(local)
}

// Transforms a point from world space to local space using the inverse of the given transformation matrix.
func WorldToLocal(world *Vector, transform *Matrix4) *Vector {
	return transform.TransformInverse(world)
}

// Holds a three-degrees-of-freedom orientation. It represents rotations in 3D space using a four-dimensional
// complex number with a real part and three imaginary parts.
type Quaternion struct {
	R float32 // The real component of the quaternion
	I float32
	J float32
	K float32
}

// Returns the quaternion components as a slice, allowing array-like access to the quaternion elements.
func (q *Quaternion) Data() [4]float32 {
	return [4]float32{q.R, q.I, q.J, q.K}
}

// Adjusts the quaternion to have unit length, making it a valid orientation quaternion.
// A unit quaternion is required for proper rotation operations. This method scales the quaternion components
// so that the sum of their squares equals 1.
// If the quaternion has zero length (all components are zero), it's replaced with the identity quaternion (1,0,0,0) representing no rotation.
func (q *Quaternion) Normalize() {
	// Calculate the squared magnitude of the quaternion
	d := q.R*q.R + q.I*q.I + q.J*q.J + q.K*q.K

	// Check for zero-length quaternion, and use the no-rotation quaternion in that case.
	if d == 0 {
		q.R = 1
		return
	}

	// Scale by inverse of the magitude to normalize
	d = float32(1.0 / Sqrt(Real(d)))
	q.R *= float32(d)
	q.I *= float32(d)
	q.J *= float32(d)
	q.K *= float32(d)
}

// Represents the combination of two rotations. When two quaternions are multiplied, the resulting quaternion represents a rotation
// that is equivalent to applying the second rotation followed by the first.
//
// The multiplication follows the Hamilton product formula for quaternions:
// (q1 * q2).r = q1.r*q2.r - q1.i*q2.i - q1.j*q2.j - q1.k*q2.k
// (q1 * q2).i = q1.r*q2.i + q1.i*q2.r + q1.j*q2.k - q1.k*q2.j
// (q1 * q2).j = q1.r*q2.j + q1.j*q2.r + q1.k*q2.i - q1.i*q2.k
// (q1 * q2).k = q1.r*q2.k + q1.k*q2.r + q1.i*q2.j - q1.j*q2.i
func (q *Quaternion) MultiplyWith(multiplier *Quaternion) {
	// Store original values to avoid interference during calculation
	qr, qi, qj, qk := q.R, q.I, q.J, q.K

	q.R = qr*multiplier.R - qi*multiplier.I - qj*multiplier.J - qk*multiplier.K
	q.I = qr*multiplier.I + qi*multiplier.R + qj*multiplier.K - qk*multiplier.J
	q.J = qr*multiplier.J + qj*multiplier.R + qk*multiplier.I - qi*multiplier.K
	q.K = qr*multiplier.K + qk*multiplier.R + qi*multiplier.J - qj*multiplier.I
}

// Rotates this quaternion by a vector.
//
// This method creates a rotation quaternion from the given vector and multiplies this quaternion by it. The vector's components are used as the imaginary
// parts of a quaternion with a rela part of zero.
//
// This operation can be used to apply an angular velocity to an orientation quaternion, where the vector represents the axis and magnitude of rotation.
func (q *Quaternion) RotateByVector(vector *Vector) {
	// Create a quaternion with the vector components as the imaginary parts
	rotationQ := &Quaternion{
		R: 0,
		I: float32(vector.X),
		J: float32(vector.Y),
		K: float32(vector.Z),
	}

	q.MultiplyWith(rotationQ)
}

// Adds a scaled vector to this quaternion.
//
// This method is commonly used to update an orientation quaternion by an angular velocity over time. The vector represents
// the angular velocity axis and magnitude, and the scale parameter is typically the time delta.
//
// The operation creates a rotation quaternion from the scaled vector, multiplies it with this quaternion, and then adds
// half of the result of this quaternion. This approximates the integration of angular velocity to update orientation.
func (q *Quaternion) AddScaledVector(vector *Vector, scale float32) {
	rotationQ := &Quaternion{
		R: 0,
		I: float32(vector.X) * scale,
		J: float32(vector.Y) * scale,
		K: float32(vector.Z) * scale,
	}

	// Multiply by the current orientation
	rotationQ.MultiplyWith(q)

	// Add half of the resulting quaternion to the current orientation
	q.R += rotationQ.R * 0.5
	q.I += rotationQ.I * 0.5
	q.J += rotationQ.J * 0.5
	q.K += rotationQ.K * 0.5
}
