#pragma once
#include "GameWorld.h"

namespace NCL {
	namespace CSC8503 {
		class PhysicsSystem {
		public:
			PhysicsSystem(GameWorld& g);
			~PhysicsSystem();

			void Clear();

			void Update(float dt);

			void UseGravity(bool state) {
				applyGravity = state;
			}

			void SetGlobalDamping(float d) {
				globalDamping = d;
			}

			void SetGravity(const Vector3& g);

			inline float GetLinearDampingFactor() { return linearDampingFactor; }
			inline void SetLinearDampingFactor(float val) { linearDampingFactor = val; }

			inline float GetAngularDampingFactor() { return angularDampingFactor; }
			inline void SetAngularDampingFactor(float val) { angularDampingFactor = val; }
		protected:
			void Sleep();
			void Wake(float dt);

			void BasicCollisionDetection(float dt);
			void BroadPhase();
			void NarrowPhase();

			void ClearForces();
			void ClearExtForces();

			void IntegrateAccel(float dt);
			void IntegrateVelocity(float dt);

			void UpdateConstraints(float dt);

			void UpdateCollisionList();
			void UpdateObjectAABBs();

			void ImpulseResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const;
			void ImpulseResolveSpringCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const;

			CollisionDetection::ContactPoint FindCentrePoint(std::vector<CollisionDetection::ContactPoint> points) const;

			GameWorld& gameWorld;

			bool	applyGravity;
			Vector3 gravity;
			float	dTOffset;
			float	globalDamping;

			std::set<CollisionDetection::CollisionInfo> allCollisions;
			std::set<CollisionDetection::CollisionInfo> broadphaseCollisions;
			std::vector<CollisionDetection::CollisionInfo> broadphaseCollisionsVec;
			bool useBroadPhase = true;
			int numCollisionFrames = 5;

			float linearDampingFactor = 0.4f;
			float angularDampingFactor = 0.4f;

			std::vector<std::vector<bool>> layers;
		};
	}
}

