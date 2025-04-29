package internal

import "math"

// Represents a floating-point precision type.
// Can be changed to float64 if double precision is needed.
type Real float32

const MaxReal Real = math.MaxFloat32

// Returns x**y, the base-x exponential of y.
func Pow(x, y Real) Real {
	return Real(math.Pow(float64(x), float64(y)))
}
