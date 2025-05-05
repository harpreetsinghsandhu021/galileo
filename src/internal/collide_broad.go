package internal

import (
	"math"
)

// Stores a potential contact between two bodies.
// This struct is used to store the bodies that might be in contact with each other.
type PotentialContact struct {
	// Holds the bodies that might be in contact.
	Body [2]*RigidBody
}

// Represents a node in bounding volume heirarchy. This is used to represent a node in the heirarchy, which
// can be either a leaf node or an internal node.
type BVHNode struct {
	// Holds the node immediately above us in the tree.
	parent *BVHNode
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

// Inserts the given rigid body, with thte given bounding volume into te hierarchy. This may involve the creation of further bounding volume nodes.
func (n *BVHNode) Insert(body *RigidBody, volume *BoundingSphere) {
	// If we are a leaf, then the only option is to spawn two new children and place the new body in one
	if n.IsLeaf() {
		// Child one is a copy of us
		n.Children[0] = &BVHNode{
			Volume: n.Volume,
			Body:   n.Body,
		}

		// Child two holds the new body
		n.Children[1] = &BVHNode{
			Volume: volume,
			Body:   body,
		}

		n.Body = nil // We lose the body, (we're no longer a leaf)

		n.recalculateBoundingVolume(true)
	} else {
		// Otherwise, we need to work out which child gets to keep the inserted body. We give it to whoever would grow the least to incorporate it.
		if n.Children[0].Volume.GetGrowth(volume) < n.Children[1].Volume.GetGrowth(volume) {
			n.Children[0].Insert(body, volume)
		} else {
			n.Children[1].Insert(body, volume)
		}
	}
}

// Recalculates the bounding volume of the node.
//
// This function is used to update the bounding volume of the node after changes have been made to its children the node itself. It is typically
// called after a node has been inserted or removed from the hierarchy.
//
// If the node is not a leaf node node, the method calculates the new bounding volume of the node by combining the bounding volumes of its two children.
// This is done using `NewBoundingSphereFromSphere` function, which creates a new bounding sphere that encloses the two child spheres.
//
// Finally, If the `recurse` parameter is true, the method calls itself recursively on the node's parent, to propogate the changes up the hierarchy.
func (n *BVHNode) recalculateBoundingVolume(recurse bool) {
	if n.IsLeaf() {
		return
	}

	n.Volume = NewBoundingSphereFromSphere(n.Children[0].Volume, n.Children[1].Volume)

	if recurse && n.parent != nil {
		n.parent.recalculateBoundingVolume(true)
	}
}

// Deletes this node, removing it first from the hierarchy along with its associated rigid body and child nodes.
//
// This method is responsible for cleaning up any resources associated with the node, including its children and sibling nodes. It also updates the
// parent node's bounding volume to reflect the change.
func (n *BVHNode) Delete() {
	// Check if the node has a parent node. If not, we can skip the sibling processing.
	if n.parent != nil {
		// Find the sibling node. This is the other child node of the parent node
		sibling := n.parent.Children[0]
		if sibling == n {
			// If the sibling is the same as the current node, use the other child node
			sibling = n.parent.Children[1]
		}

		// Copy the sibling node's data to the parent node. This effectively replaces the current node with its sibling in the hierarchy.
		n.parent.Volume = sibling.Volume
		n.parent.Body = sibling.Body
		n.parent.Children[0] = sibling.Children[0]
		n.parent.Children[1] = sibling.Children[1]

		// Delete the sibling node. We set its parent and children to nil to avoid any potential cycles that could prevent the GB from freeing memory.
		sibling.parent = nil
		sibling.Body = nil
		sibling.Children[0] = nil
		sibling.Children[1] = nil

		// Recalculate the parent node's bounding volume to reflect the changes.
		n.parent.recalculateBoundingVolume(true)
	}

	// Delete the child nodes. We set their parent to nil to avoid any potential cycles.
	if n.Children[0] != nil {
		n.Children[0].parent = nil
	}
	if n.Children[1] != nil {
		n.Children[1].parent = nil
	}
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

// Returns the volume of the bounding sphere.
func (s *BoundingSphere) GetSize() float64 {
	return (4.0 / 3.0) * math.Pi * math.Pow(s.Radius, 2)
}

// Calculates the growth in volume required to enclose the given other sphere. The "growth" is the increase in volume required to enclose the
// other sphere. This is used when adding a new sphere to the hierarchy to work out which node to add it to.
func (s *BoundingSphere) GetGrowth(other *BoundingSphere) float64 {
	newRadius := math.Sqrt(math.Pow(s.Radius, 2) + math.Pow(other.Radius, 2))
	return math.Pow(newRadius, 2) - math.Pow(s.Radius, 2)
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
