#pragma once
using namespace NCL::Maths;

namespace NCL {
	class CollisionVolume;

	namespace CSC8503 {
		class Transform;

		class PhysicsObject {
		public:
			PhysicsObject(Transform* parentTransform, const CollisionVolume* parentVolume);
			~PhysicsObject();

			Vector3 GetLinearVelocity() const {
				return linearVelocity;
			}

			Vector3 GetAngularVelocity() const {
				return angularVelocity;
			}

			Vector3 GetTorque() const {
				return torque;
			}

			Vector3 GetForce() const {
				return force;
			}

			void SetInverseMass(float invMass) {
				inverseMass = invMass;
			}

			float GetInverseMass() const {
				return inverseMass;
			}

			float GetFriction() const {
				return friction;
			}

			void ApplyAngularImpulse(const Vector3& force);
			void ApplyLinearImpulse(const Vector3& force);

			void AddForce(const Vector3& force);

			void AddForceAtPosition(const Vector3& force, const Vector3& position);

			void AddTorque(const Vector3& torque);

			void ClearForces();

			void SetLinearVelocity(const Vector3& v) {
				linearVelocity = v;
			}

			void SetAngularVelocity(const Vector3& v) {
				angularVelocity = v;
			}

			void InitCubeInertia();
			void InitSphereInertia(bool hollow = false);

			void UpdateInertiaTensor();

			Matrix3 GetInertiaTensor() const {
				return inverseInteriaTensor;
			}

			float GetElasticity() const {
				return elasticity;
			}

			void SetElasticity(float value) {
				elasticity = value;
			}

			Vector3 GetExtTorque() const {
				return extTorque;
			}

			Vector3 GetExtForce() const {
				return extForce;
			}

			void ToggleSpring() {
				isSpring = !isSpring;
			}

			bool IsSpring() const {
				return isSpring;
			}

			bool IsGravity() const {
				return feelsGravity;
			}

			void FeelsGravity(bool grav) {
				feelsGravity = grav;
			}

			bool IsAwake() const {
				return awake;
			}

			void ToggleAwake() {
				awake = !awake;
				if (awake) {
					wakeTimer = 5;
				}
			}

			float GetWakeTimer() {
				return wakeTimer;
			}

			void TickWakeTimer(float dt) {
				wakeTimer -= dt;
			}

			void AddExtForceAtPosition(const Vector3& force, const Vector3& position);
			void ClearExtForces();

			void SetMaxVel(Vector3 val) {
				maxVel = val;
			}

			Vector3 GetMaxVel() {
				return maxVel;
			}

		protected:
			const CollisionVolume* volume;
			Transform* transform;

			float inverseMass;
			float elasticity;
			float friction;

			bool feelsGravity;

			Vector3 maxVel;

			//sleep stuff
			bool awake;
			float wakeTimer;

			//linear stuff
			Vector3 linearVelocity;
			Vector3 force;

			//angular stuff
			Vector3 angularVelocity;
			Vector3 torque;
			Vector3 inverseInertia;
			Matrix3 inverseInteriaTensor;

			//external stuff
			bool isSpring;
			Vector3 extForce;
			Vector3 extTorque;
		};
	}
}

