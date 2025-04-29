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

func Sin(x Real) Real {
	return Real(math.Sin(float64(x)))
}

func Cos(x Real) Real {
	return Real(math.Cos(float64(x)))
}

func Exp(x Real) Real {
	return Real(math.Exp(float64(x)))
}

func Abs(x Real) Real {
	return Real(math.Abs(float64(x)))
}

func Sqrt(x Real) Real {
	return Real(math.Sqrt(float64(x)))
}
