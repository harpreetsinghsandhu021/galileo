package internal

// Represents two bodies in contact.
// Resolving a contact removes their interpenetration, and applies sufficient impulse to keep them apart.
// Colliding bodies may also be rebound. Contacts can be used to represent positional joints, by making
// the contact constraint keep the bodies in their correct orientation.
type Contact struct {
	ContactPoint  *Vector // Holds the position of the contact in world coordinates
	ContactNormal *Vector // Holds the direction of the contact in world coordinates
	Penetration   float64 // Holds the depth of penetration at the contact point. If both bodies are specified,
	// then the contact point should be midway b/w the interpenetrating points.
	body        [2]*RigidBody // Holds the bodies that are involved in the contact.
	friction    Real          // Lateral friction coefficient
	restitution Real
}

// Sets the data that does'nt normally depend on the position of the contact (i.e. the bodies and their material properties)
func (c *Contact) SetBodyData(one, two *RigidBody, friction, restitution Real) {
	c.body[0] = one
	c.body[1] = two
	c.friction = friction
	c.restitution = restitution
}
