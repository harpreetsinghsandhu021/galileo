package internal

import "math"

// A Helper structure that contains information for the detector to use in building its contact data.
type CollisionData struct {
	Contacts     []*Contact
	ContactsLeft int
	Friction     Real
	Restitution  Real
	ContactCount uint
}

// Notifies the data that the given number of contacts have been added
func (cd *CollisionData) AddContacts(count uint) {
	cd.ContactsLeft -= int(count)
	cd.ContactCount += count

	cd.Contacts = cd.Contacts[count:]
}

// Represents a primitive to detect collisions against
type CollisionPrimitive struct {
	Body      *RigidBody
	Offset    *Matrix4 // Offset of the primitve from the given rigid body
	Transform *Matrix4 // The resultant transform of the primitive, calculated from the combined offset of the primitive and the transform of the rigid body to which it was attached.
}

func (cp *CollisionPrimitive) calculateInternals() {}

// Represents a rigid body that can be treated as sphere for collision detection.
type CollisionSphere struct {
	CollisionPrimitive
	Radius float64
	Axis   *Vector
}

func NewCollisionSphere(radius float64, axis *Vector) *CollisionSphere {
	return &CollisionSphere{
		Axis:   axis,
		Radius: radius,
	}
}

// Plane is not a primitive nor a rigid body. It is used for contacts with the immovable world geometry.
type CollisionPlane struct {
	Direction *Vector
	Offset    Real
}

// Represents a rigid body that can be treated as an aligned bounding box for collision detection.
type CollisionBox struct {
	CollisionPrimitive
	halfSize *Vector // Holds the half-sizes of the box along each of its local axes.
}

type CollisionDetector struct{}
type IntersectionTests struct{}

// Checks if two spheres are colliding and returns the number of contacts.
// It takes three arguments: two CollisionSphere pointers and a ColluisionData pointer.
// Returns:
//   - The number of contacts
func (cd *CollisionDetector) SphereAndSphere(one, two *CollisionSphere, data *CollisionData) uint {
	if data.ContactsLeft <= 0 {
		// Insufficient capacity to store contact information
		return 0
	}

	// Calculate the position vectors of two spheres. These vectors represent the centers of spheres in 3D space.
	positionOne := one.Transform.GetAxisVector(3)
	positionTwo := two.Transform.GetAxisVector(3)

	// Calculate the vector difference b/w the two sphere positions. This vector represents the direction from the
	// center of the first sphere to the center of the other sphere.
	midline := positionOne.Subtract(positionTwo)

	// Calculate the euclidean distance b/w the two sphere centers.
	size := midline.Magnitude()

	// Check if the distance is large enough. If not, the spheres are coincident or overlapping, and the collision
	// is not valid
	if size <= 0.0 {
		return 0
	}

	// Check if the distance b/w the two spheres is greater than or equal to the sum of their radii. If so, the spheres
	// are not colliding, and the collision is not valid.
	if size >= Real(one.Radius)+Real(two.Radius) {
		return 0
	}

	// Calculate the direction from the center of the first sphere to the center of second sphere, normalized to unit length.
	normal := midline.Scale(1.0 / size)

	contact := &Contact{
		ContactNormal: normal,
		ContactPoint:  positionOne.Add(midline.Scale(0.5)),
		Penetration:   one.Radius + two.Radius - float64(size),
	}

	contact.SetBodyData(one.Body, two.Body, data.Friction, data.Restitution)

	data.Contacts = append(data.Contacts, contact)
	data.ContactsLeft--

	return 1
}

// Performs a collision detection b/w a sphere and a half-space(represented by a plane).
// Returns:
//   - The number of contacts
func (cd *CollisionDetector) SphereAndHalfSpace(sphere *CollisionSphere, plane *CollisionPlane, data *CollisionData) uint {
	if data.ContactsLeft <= 0 {
		// Insufficient capacity to store contact information
		return 0
	}

	// Calculate the center of the sphere in 3D space.
	position := sphere.Transform.GetAxisVector(3)

	// Calculate the distance from the plane to the sphere. The distance is calculated by projecting the sphere position onto the plane
	// normal and subtracting the sphere radius and plane offset.
	ballDistance := plane.Direction.ScalarProduct(position) - Real(sphere.Radius) - plane.Offset

	// If the sphere is not intersecting with half-space, the collision is not valid.
	if ballDistance >= 0 {
		return 0
	}

	contact := &Contact{
		ContactNormal: plane.Direction,
		Penetration:   -float64(ballDistance),
		ContactPoint:  position.Subtract(plane.Direction.Scale(ballDistance + Real(sphere.Radius))),
	}

	contact.SetBodyData(sphere.Body, nil, data.Friction, data.Restitution)

	data.Contacts = append(data.Contacts, contact)
	data.ContactsLeft--

	return 1
}

// Performs a collision detection b/w a sphere and a true plane.
func (cd *CollisionDetector) SphereAndTruePlane(sphere *CollisionSphere, plane *CollisionPlane, data *CollisionData) uint {
	if data.ContactsLeft <= 0 {
		return 0
	}

	// Calculate the center of the sphere in 3D space.
	position := sphere.Transform.GetAxisVector(3)

	// Calculate the distance from the plane to the center of the sphere.
	centerDistance := plane.Direction.ScalarProduct(position) - plane.Offset

	// Check if the distance from the plane to the center of the sphere is greater than the sphere's radius.
	// If so, the sphere is not intersecting with the plane and the collision is not valid.
	if centerDistance*centerDistance > Real(sphere.Radius)*Real(sphere.Radius) {
		return 0
	}

	// Determine the normal vector of the collision. If the center of the sphere is on the negative side of the plane, the normal vector points in the opposite direction.
	var normal = plane.Direction
	var penetration = -centerDistance
	if centerDistance < 0 {
		normal = plane.Direction.Scale(-1)
		penetration = -centerDistance
	}

	penetration += Real(sphere.Radius)

	contact := &Contact{
		ContactNormal: normal,
		Penetration:   float64(penetration),
		ContactPoint:  position.Subtract(plane.Direction.Scale(centerDistance)),
	}

	contact.SetBodyData(
		sphere.Body,
		nil,
		data.Friction,
		data.Restitution,
	)

	data.Contacts = append(data.Contacts, contact)
	data.ContactsLeft--

	return 1
}

// Calculates the projected radius of a box along a given axis. The projected radius is the furthest point of the box's bounding box along the given axis,
// transformed into the coordinate system of the box. This is useful for collision detection, as it allows us to determine if a collision is possible along a given axis.
func TransformToAxis(box *CollisionBox, axis *Vector) Real {
	return box.halfSize.X*Abs(axis.ScalarProduct(box.Transform.GetAxisVector(0))) + box.halfSize.Y*Abs(axis.ScalarProduct(box.Transform.GetAxisVector(1))) + box.halfSize.Z*Abs(axis.ScalarProduct(box.Transform.GetAxisVector(2)))
}

// Performs a collision detection b/w a box and a half-space (represented by a plane). It calculates the projected radius of the box along the plane direction,
// and the signed distance from the box to the plane. If the signed distance is less than or equal to the projected radius, the box is considered to be intersecting with
// the half-space.
func BoxAndHalfSpace(box *CollisionBox, plane *CollisionPlane) bool {
	projectedRadius := TransformToAxis(box, plane.Direction)

	boxDistance := plane.Direction.ScalarProduct(box.Transform.GetAxisVector(3)) - projectedRadius

	return boxDistance <= plane.Offset
}

// Performs a collision detection b/w a box and a half-space (represented by a plane).
// Returns:
//   - The number of contacts
func (cd *CollisionDetector) boxAndHalfSpace(box *CollisionBox, plane *CollisionPlane, data *CollisionData) int {
	if data.ContactsLeft <= 0 {
		return 0
	}

	// Check for intersection b/w the box and the half-space.
	if !BoxAndHalfSpace(box, plane) {
		return 0
	}

	// We have an intersection, so find the intersection points. We can make do with only checking vertices. If the box is resting on a plane
	// or an edge, it will be reported as four or two contact points.

	// Define the multipliers for each vertex. These multipliers represent the eight combinations of +/- half-size components.
	mults := [8][3]float64{
		{1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1},
		{1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1},
	}

	contact := &Contact{}
	contactsUsed := 0

	// Iterate over each vertex
	for i := 0; i < 8; i++ {
		// Calculate the position of each vertex
		vertexPos := NewVector3(Real(mults[i][0]), Real(mults[i][1]), Real(mults[i][2]))
		// Scale by box's half size
		vertexPos.ComponentProductInPlace(*box.halfSize)
		// Transform to world coordinates
		vertexPos = LocalToWorld(vertexPos, box.Transform)
		// Calculate the distance from the plane
		vertexDistance := vertexPos.ScalarProduct(plane.Direction)

		// Compare this to plane's distance
		if vertexDistance <= plane.Offset {
			contact.ContactPoint = plane.Direction.Scale(vertexDistance - plane.Offset).Add(vertexPos)
			contact.ContactNormal = plane.Direction
			contact.Penetration = float64(plane.Offset) - float64(vertexDistance)

			contact.SetBodyData(box.Body, nil, data.Friction, data.Restitution)

			contactsUsed++

			if contactsUsed == data.ContactsLeft {
				return contactsUsed
			}
		}
	}

	data.Contacts = append(data.Contacts, contact)
	data.ContactsLeft--

	return contactsUsed
}

// Detects collision b/w a box and a sphere primitive.
// Returns:
//   - The number of contacts generated
func (cd *CollisionDetector) BoxAndSphere(box *CollisionBox, sphere *CollisionSphere, data *CollisionData) int {
	// Transform sphere's center into box's local coordinate space
	center := sphere.Transform.GetAxisVector(3)
	relCenter := WorldToLocal(center, box.Transform)

	// Early out: check if sphere is too far from box in any dimension
	if Abs(relCenter.X)-Real(sphere.Radius) > box.halfSize.X ||
		Abs(relCenter.Y)-Real(sphere.Radius) > box.halfSize.Y ||
		Abs(relCenter.Z)-Real(sphere.Radius) > box.halfSize.Z {
		return 0
	}

	// Init closest point on box to sphere's center
	closestPt := NewZeroVector()
	var dist Real

	// Clamp each coordinate to the box's boundaries
	dist = relCenter.X
	if dist > box.halfSize.X {
		dist = box.halfSize.X
	}
	if dist < -box.halfSize.X {
		dist = -box.halfSize.X
	}
	closestPt.X = dist

	dist = relCenter.Y
	if dist > box.halfSize.Y {
		dist = box.halfSize.Y
	}
	if dist < -box.halfSize.Y {
		dist = -box.halfSize.Y
	}
	closestPt.Y = dist

	dist = relCenter.Z
	if dist > box.halfSize.Z {
		dist = box.halfSize.Z
	}
	if dist < -box.halfSize.Z {
		dist = -box.halfSize.Z
	}
	closestPt.Z = dist

	// Calculate squared distance b/w closest point and sphere center
	// Using squared magnitude to avoid unecessary square root
	dist = closestPt.Subtract(relCenter).SquareMagnitude()

	// Early out: Check if distance is greater than sphere's radius squared
	if dist > Real(sphere.Radius*sphere.Radius) {
		return 0
	}

	// Transform closest point back to world coordinates
	closestPtWorld := LocalToWorld(closestPt, box.Transform)

	contact := &Contact{
		ContactNormal: closestPtWorld.Subtract(center).Normalize(),
		ContactPoint:  closestPtWorld,
		Penetration:   sphere.Radius - float64(Sqrt(dist)),
	}

	contact.SetBodyData(box.Body, sphere.Body, data.Friction, data.Restitution)

	data.Contacts = append(data.Contacts, contact)
	data.ContactsLeft--

	return 1
}

// Calculates the overlap b/w two boxes along a given axis
// Params:
//
//	one: First collision box
//	two: Second collision box
//	axis: The axis along which to check penetration
//	toCenter: Vector from center of box one to center of box two
//
// Returns:
//
//	float64: Penetration depth
//	- positive value indicates boxes are overlapping
//	- Negative value indicates boxes are seperated
//	- Larger positive values mean deeper penetration
func penetrationOnAxis(one *CollisionBox, two *CollisionBox, axis, toCenter *Vector) float64 {
	// Calculate the projection of the box's extents onto the axis
	// This gives us half the length of box's when projected onto the axis
	oneProject := TransformToAxis(one, axis)
	twoProject := TransformToAxis(two, axis)

	// Calculate the distance b/w box centers along the axis along the axis
	distance := Abs(toCenter.ScalarProduct(axis))

	// Calculate and return the overlap:
	// - Sum of projections (oneProject + twoProject) gives total reach of both boxes
	// - Subtracting the distance gives us the overlap
	return float64(oneProject) + float64(twoProject) - float64(distance)
}

// Tests if two boxes are colliding along a given axis and updates the smallest found
// Params:
//
//	one: First collision box
//	two: Second collision box
//	axis: Direction to test for collision
//	toCenter: Vector from center of box one to box two
//	index: Identifier for this axis test case
//	smallestPenetration: Current smallest penetration found
//	smallestCase: Index of the case with smallest penetration
//
// Returns:
//
//	bool: true if axis is valid and boxes might be colliding, false if boxes are seperated on the axis
func tryAxis(one *CollisionBox, two *CollisionBox, axis *Vector, toCenter *Vector, index uint, smallestPenetration float64, smallestCase uint) bool {
	// Check if axis is too small. This prevents testing against effectively parallet axes and avoids normalization of very small vectors.
	if axis.SquareMagnitude() < 0.0001 {
		return true
	}

	axis.Normalize()

	// Calculate penetration depth along the axis
	penetration := penetrationOnAxis(one, two, axis, toCenter)
	// Early exit: If penetration is negative, the boxes are seperated along the axis
	if penetration < 0 {
		return false
	}

	if penetration < smallestPenetration {
		smallestPenetration = penetration
		smallestCase = index
	}

	return true
}

func CheckOverlap(one *CollisionBox,
	two *CollisionBox,
	axis *Vector,
	toCenter *Vector,
	index uint,
	pen float64,
	best uint) int {
	if !tryAxis(one, two, axis, toCenter, index, pen, best) {
		return 0
	}

	return 1
}

// Generates contact data for a vertex-face collision b/w two boxes. This is called when we know a vertex from box two is collifing with a face of box one.
func (c *CollisionDetector) fillPointFaceBoxBox(one, two *CollisionBox, toCenter *Vector, data *CollisionData, best uint, pen float64) {
	// Get the contact we're going to write to
	contact := &Contact{}

	// Get the collision normal from box one's axis. This is the face normal of the colliding face
	normal := one.Transform.GetAxisVector(int(best))

	// Ensure the normal points from box one to box two. If the axis points in the same direction as toCenter, we need to invert it to point toward box two.
	if normal.ScalarProduct(toCenter) > 0 {
		normal = normal.Scale(-1.0)
	}

	// Calculate which vertex of box two is colliding with. We do this by starting at the positive vertex (halfsize) and negating components based on the collision control.
	vertex := two.halfSize

	// For each axis, if the dot product of that axis with the normal is negative, it means we need to use the negative vertex in that dimension
	if two.Transform.GetAxisVector(0).ScalarProduct(normal) < 0 {
		vertex.X = -vertex.X
	}

	if two.Transform.GetAxisVector(1).ScalarProduct(normal) < 0 {
		vertex.Y = -vertex.Y
	}

	if two.Transform.GetAxisVector(2).ScalarProduct(normal) < 0 {
		vertex.Z = -vertex.Z
	}

	contact.ContactNormal = normal
	contact.Penetration = pen
	contact.ContactPoint = two.Transform.Transform(vertex)
	contact.SetBodyData(one.Body, two.Body, data.Friction, data.Restitution)
}

func (c *CollisionDetector) BoxAndBox(one *CollisionBox, two *CollisionBox, data *CollisionData) uint {
	// Calculate vector b/w box centers
	toCenter := two.Transform.GetAxisVector(3).Subtract(one.Transform.GetAxisVector(3))

	pen := MaxReal         // Smallest penetration found
	best := uint(0xffffff) // Index of axis with smallest penetration

	// Test all potential seperating axes:

	// 1. Test box one's face normals (local axes)
	CheckOverlap(one, two, one.Transform.GetAxisVector(0), toCenter, 0, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(1), toCenter, 1, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(2), toCenter, 2, float64(pen), best)

	// 1. Test box two's face normals (local axes)
	CheckOverlap(one, two, two.Transform.GetAxisVector(0), toCenter, 3, float64(pen), best)
	CheckOverlap(one, two, two.Transform.GetAxisVector(1), toCenter, 4, float64(pen), best)
	CheckOverlap(one, two, two.Transform.GetAxisVector(2), toCenter, 5, float64(pen), best)

	// Store the best axis from face tests for edge-edge collision case
	bestSingleAxis := best

	// 3. Test Cross products of edges (9 potential edge-edge cases)
	CheckOverlap(one, two, one.Transform.GetAxisVector(0).Cross(two.Transform.GetAxisVector(0)), toCenter, 6, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(0).Cross(two.Transform.GetAxisVector(1)), toCenter, 7, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(0).Cross(two.Transform.GetAxisVector(2)), toCenter, 8, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(1).Cross(two.Transform.GetAxisVector(0)), toCenter, 9, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(1).Cross(two.Transform.GetAxisVector(1)), toCenter, 10, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(1).Cross(two.Transform.GetAxisVector(2)), toCenter, 11, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(2).Cross(two.Transform.GetAxisVector(0)), toCenter, 12, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(2).Cross(two.Transform.GetAxisVector(1)), toCenter, 13, float64(pen), best)
	CheckOverlap(one, two, one.Transform.GetAxisVector(2).Cross(two.Transform.GetAxisVector(2)), toCenter, 14, float64(pen), best)

	if best == 0xffffff {
		return 0
	}

	// We now know there's a collision, and we know which of the axes gave the smallest penetration. We now can deal with it in different ways depending on the case.
	switch {
	case best < 3:
		// Vertex of box two on face of box one
		c.fillPointFaceBoxBox(one, two, toCenter, data, best, float64(pen))
		data.AddContacts(1)
		return 1

	case best < 6:
		// Vertex of box one on face of box two
		c.fillPointFaceBoxBox(two, one, toCenter.Scale(-1), data, best-3, float64(pen))
		data.AddContacts(1)
		return 1
	default:
		// Edge-edge contact
		return c.handleEdgeEdgeContact()
	}

}

// Handles the complex case of edge-edge collision b/w boxes
func (c *CollisionDetector) handleEdgeEdgeContact(one, two *CollisionBox, toCenter *Vector, best uint, pen float64, data *CollisionData) uint {
	// Calculates which edges are involved
	oneAxisIndex := best / 3
	twoAxisIndex := best % 3

	// Get the axes and compute collision axis
	oneAxis := one.Offset.GetAxisVector(int(oneAxisIndex))
	twoAxis := two.Offset.GetAxisVector(int(twoAxisIndex))
	axis := oneAxis.Cross(twoAxis)
	axis.Normalize()

	// Ensure axis points from box one to box two
	if axis.ScalarProduct(toCenter) > 0 {
		axis = axis.Scale(-1)
	}

	// We have the axes, but not the edges: each axis has 4 edges parallel
	// to it, we need to find which of the 4 for each object. We do
	// that by finding the point in the centre of the edge. We know
	// its component in the direction of the box's collision axis is zero
	// (its a mid-point) and we determine which of the extremes in each
	// of the other axes is closest.

	// we have the axis but not the edges:each axis has 4 edges parallel to it, we need ot find which 4 for each object.
	// We do that by finding the point in the centere of the edge. We know its components in the direction of the box's collision
	// axis is zero(it's a mid point) and we determine which of the extremes in each of other axes is closest.
	ptOnOneEdge := one.halfSize
	ptOnTwoEdge := two.halfSize

	// Adjust points based on axis directions
	for i := 0; i < 3; i++ {
		if i == int(oneAxisIndex) {
			switch i {
			case 0:
				ptOnOneEdge.X = 0
			case 1:
				ptOnOneEdge.Y = 0
			case 2:
				ptOnOneEdge.Z = 0
			}
		} else if one.Transform.GetAxisVector(i).ScalarProduct(axis) > 0 {
			switch i {
			case 0:
				ptOnOneEdge.X = -ptOnOneEdge.X
			case 1:
				ptOnOneEdge.Y = -ptOnOneEdge.Y
			case 2:
				ptOnOneEdge.Z = -ptOnOneEdge.Z
			}
		}

		if i == int(twoAxisIndex) {
			switch i {
			case 0:
				ptOnTwoEdge.X = 0
			case 1:
				ptOnTwoEdge.Y = 0
			case 2:
				ptOnTwoEdge.Z = 0
			}
		} else if two.Transform.GetAxisVector(i).ScalarProduct(axis) > 0 {
			switch i {
			case 0:
				ptOnTwoEdge.X = -ptOnTwoEdge.X
			case 1:
				ptOnTwoEdge.Y = -ptOnTwoEdge.Y
			case 2:
				ptOnTwoEdge.Z = -ptOnTwoEdge.Z
			}
		}
	}

	// Transform edge points to world coordinates
	ptOnOneEdge = LocalToWorld(ptOnOneEdge, one.Transform)
	ptOnOneEdge = LocalToWorld(ptOnOneEdge, two.Transform)

}

// Calculates the contact point b/w two edges in 3D space. This function finds the point of closest approach b/w two line segments.
// Params:
//   - pOne: Start point of first edge
//   - dOne: Direction vector of first edge
//   - oneSize: Half-length of first edge
//   - pTwo: Start point of second edge
//   - dTwo: Direction vector of second edge
//   - twoSize: Half-length of second edge
//   - useOne: If true, use first edge's midpoint when edges don't cross
//
// Returns:
//
//	Vector: The calculated contact point b/w the edges
func contactPoint(pOne *Vector, dOne *Vector, oneSize float64, pTwo *Vector, dTwo *Vector, twoSize float64, useOne bool) *Vector {
	// Calculate squares of magnitudes and dot products. These are used in the denominator calculation.
	smOne := dOne.SquareMagnitude()
	smTwo := dOne.SquareMagnitude()
	dpOneTwo := dTwo.ScalarProduct(dOne) // Dot product of directions

	// Calculate vector b/w start points and its dot products
	toSt := pOne.Subtract(pTwo) // Vector from pTwo to pOne
	dpStaOne := dOne.ScalarProduct(toSt)
	dpStaTwo := dTwo.ScalarProduct(toSt)

	// Calculate denominator for paramter equations. This will be zero if edges are parallel
	denom := smOne*smTwo - dpOneTwo*dpOneTwo

	// Handle paraller edge cases
	if math.Abs(float64(denom)) < 0.0001 {
		if useOne {
			return pOne
		}

		return pTwo
	}

	// Calculate parameters for closest points on infinite lines
	mua := (dpOneTwo*dpStaTwo - smTwo*dpStaOne) / denom
	mub := (smOne*dpStaTwo - dpOneTwo*dpStaOne) / denom

}
