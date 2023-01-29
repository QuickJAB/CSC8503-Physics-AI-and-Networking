#include "OrientationConstraint.h"
#include "GameObject.h"
#include "PhysicsObject.h"

using namespace NCL;
using namespace Maths;
using namespace CSC8503;

OrientationConstraint::OrientationConstraint(GameObject* a, GameObject* b, Vector3 ax) {
	objectA = a;
	objectB = b;
	axis = ax;
}

OrientationConstraint::~OrientationConstraint() {

}

void OrientationConstraint::UpdateConstraint(float dt) {
	Vector3 oriB = objectB->GetTransform().GetOrientation() * axis;

	Vector3 offsetDir = (oriB).Normalised();
	float offset = 1 - Vector3::Dot(offsetDir, axis);

	if (abs(offset) > 0.0f) {
		PhysicsObject* physB = objectB->GetPhysicsObject();

		Vector3 relativeVelocity = Vector3::Cross(-physB->GetAngularVelocity(), axis);

		float constraintMass = physB->GetInverseMass();

		if (constraintMass > 0.0f) {
			float velocityDot = Vector3::Dot(relativeVelocity, oriB);
			float biasFactor = 0.01f;
			float bias = -(biasFactor / dt) * offset;

			Vector3 correction = Vector3::Cross(axis, offsetDir);

			float lambda = -(velocityDot + bias) / constraintMass;

			Vector3 aImpulse = correction * lambda;
			Vector3 bImpulse = -correction * lambda;

			physB->ApplyAngularImpulse(bImpulse);
		}
	}
}