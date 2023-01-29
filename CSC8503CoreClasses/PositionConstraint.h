#pragma once
#include "Constraint.h"
#include <MeshGeometry.h>

namespace NCL {
	namespace CSC8503 {
		class GameObject;

		class PositionConstraint : public Constraint	{
		public:
			PositionConstraint(GameObject* a, GameObject* b, float d);
			~PositionConstraint();

			void UpdateConstraint(float dt) override;

			void SetOffsetA(Vector3 offset) {
				posOffA = offset;
			}

			void SetOffsetB(Vector3 offset) {
				posOffB = offset;
			}

		protected:
			GameObject* objectA;
			GameObject* objectB;

			float distance;

			Vector3 posOffA, posOffB;
		};
	}
}