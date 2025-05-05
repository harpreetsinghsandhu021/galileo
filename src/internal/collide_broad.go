package internal

import "math"

// Stores a potential contact between two bodies.
// This struct is used to store the bodies that might be in contact with each other.
type PotentialContact struct {
	// Holds the bodies that might be in contact.
	Body [2]*RigidBody
}

// Represents a node in bounding volume heirarchy. This is used to represent a node in the heirarchy, which
// can be either a leaf node or an internal node.
type BVHNode struct {
	// Holds the child nodes of this node representing the left and right child nodes.
	Children [2]*BVHNode
	// Holds a single bounding volume encompassing all the descendants of this node.
	Volume *BoundingSphere
	// Holds the rigid body at this node of the heirarchy.
	Body *RigidBody
}

// Checks whether this node is at the bottom of the heirarchy.
// A node is considered a leaf node if it has a rigid body associated with it.
func (n *BVHNode) IsLeaf() bool {
	return n.Body != nil
}

// Checks the potential contacts from this node downward in the heirarchy, writing them to the
// given array (up to the given limit).
// Returns:
//   - The number of potential contacts if found.
func (n *BVHNode) GetPotentialContacts(contacts []PotentialContact, limit uint) uint {
	if n.IsLeaf() || limit == 0 {
		return 0
	}

	// Recursively call GetPotentialContactsWith on the left child node, passing the right child node as the other node
	return n.Children[0].GetPotentialContactsWith(n.Children[1], contacts, limit)
}

// Checks the potential contacts b/w this node and another node, writing them to the given array (up to the given limit).
func (n *BVHNode) GetPotentialContactsWith(other *BVHNode, contacts []PotentialContact, limit uint) uint {
	// If the bounding volumes of this node and the other node do not overlap, return
	if !n.Overlaps(other) || limit == 0 {
		return 0
	}

	// If both nodes are leaf nodes, add a potential contact b/w the two rigid bodies to the contact array.
	if n.IsLeaf() && other.IsLeaf() {
		contacts[0].Body[0] = n.Body
		contacts[0].Body[1] = other.Body
		return 1
	}

	// Determine which node to descend into. If the other node is a leaf node, or if this node is not a leaf node
	// and its bounding volume is larger than the other node's bounding volume, descend into this node.
	if other.IsLeaf() || (!n.IsLeaf() && n.Volume.GetSize() >= other.Volume.GetSize()) {
		// Recursively call GetPotentialContactsWith on the left child node, passing the other node as the other node.
		count := n.Children[0].GetPotentialContactsWith(other, contacts, limit)

		// If the limit has not been reached yet, recursively call GetPotentialContactsWith on the right child node.
		// and add the results to the count.
		if limit > uint(count) {
			return count + n.Children[1].GetPotentialContactsWith(other, contacts[count:], limit-uint(count))
		} else {
			return uint(count)
		}
	} else {
		// Descend into the other node.
		// Recursively call GetPotentialContactsWith on this node, passing the left child node of the other node as the other node.
		count := n.GetPotentialContactsWith(other.Children[0], contacts, limit)

		if limit > uint(count) {
			return count + n.GetPotentialContactsWith(other.Children[1], contacts[count:], limit-uint(count))
		} else {
			return uint(count)
		}
	}
}

// Checks whether the node overlaps with another node.
func (n *BVHNode) Overlaps(other *BVHNode) bool {
	return n.Volume.Radius+other.Volume.Radius >= float64(Sqrt(Pow(Real(n.Volume.Radius), 2)+Pow(Real(other.Volume.Radius), 2)))
}

// Represents a bounding sphere that can be tested for overlap
type BoundingSphere struct {
	Center *Vector
	Radius float64
}

// Creates a new bounding sphere at the given center and radius.
func NewBoundingSphere(center *Vector, radius float64) *BoundingSphere {
	return &BoundingSphere{
		Center: center,
		Radius: radius,
	}
}

func (s *BoundingSphere) GetSize() float64 {
	return (4.0 / 3.0) * math.Pi * math.Pow(s.Radius, 2)
}

// Creates a bounding sphere to enclose the two given bounding spheres
func NewBoundingSphereFromSphere(one, two *BoundingSphere) *BoundingSphere {
	centeroffset := two.Center.Subtract(one.Center) // Calculate the offset b/w the centers of two spheres
	distance := centeroffset.SquareMagnitude()      // Calculate the squared disance b/w the centers of the two spheres
	radiusDiff := two.Radius - one.Radius           // Calculate the difference in radii b/w the two spheres

	// Check whether the larger sphere encloses the small one. This is done by checking whether the squared difference
	// in radii is greater than or equal to the squared distance b/w the centers.
	if radiusDiff*radiusDiff >= float64(distance) {
		// If the larger sphere encloses the smaller one, return the larger sphere
		if one.Radius > two.Radius {
			return &BoundingSphere{Center: one.Center, Radius: one.Radius}
		} else {
			return &BoundingSphere{Center: two.Center, Radius: two.Radius}
		}
	} else {
		// Otherwise, we need to work with partially overlapping spheres.

		distance = Sqrt(distance) // Calculate the actual distance b/w the centers of two spheres

		// Calculate the radius of the new sphere that encloses both spheres.
		radius := ((distance + Real(one.Radius) + Real(two.Radius)) * 0.5)

		// Calculate the center of the new sphere. This is done by moving the center of the first
		// sphere towards the center of the second sphere by an amount proportional to the radii of the spheres.
		center := one.Center
		if distance > 0 {
			center = center.Add(centeroffset.Scale((radius - Real(one.Radius)) / distance))
		}

		return &BoundingSphere{Center: center, Radius: float64(radius)}
	}

}

// Checks whether this bounding sphere overlaps with the other given bounding sphere.
func (s *BoundingSphere) Overlaps(other *BoundingSphere) bool {
	distanceSquared := s.Center.Subtract(other.Center).SquareMagnitude()

	// Check whether the squared distance is less than the sum of the radii squared.
	return distanceSquared < (Real(s.Radius+other.Radius) * Real(s.Radius+other.Radius))
}
