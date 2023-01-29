#pragma once
#include "Constraint.h"
#include <Maths.h>
#include <MeshGeometry.h>

namespace NCL {
	namespace CSC8503 {
		class GameObject;

		class OrientationConstraint : public Constraint
		{
		public:
			OrientationConstraint(GameObject* a, GameObject* b, Vector3 axis);
			~OrientationConstraint();

			void UpdateConstraint(float dt) override;

		protected:
			GameObject* objectA;
			GameObject* objectB;

			Vector3 axis;
		};
	}
}

