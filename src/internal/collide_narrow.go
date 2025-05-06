package internal

// A Helper structure that contains information for the detector to use in building its contact data.
type CollisionData struct {
	Contacts     []*Contact
	ContactsLeft int
	Friction     Real
	Restitution  Real
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
