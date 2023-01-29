#pragma once
#include "Transform.h"
#include "CollisionVolume.h"

using std::vector;

namespace NCL::CSC8503 {
	class NetworkObject;
	class RenderObject;
	class PhysicsObject;

	class GameObject	{
	public:
		GameObject(std::string name = "");
		~GameObject();

		void SetBoundingVolume(CollisionVolume* vol) {
			boundingVolume = vol;
		}

		const CollisionVolume* GetBoundingVolume() const {
			return boundingVolume;
		}

		bool IsActive() const {
			return isActive;
		}

		Transform& GetTransform() {
			return transform;
		}

		RenderObject* GetRenderObject() const {
			return renderObject;
		}

		PhysicsObject* GetPhysicsObject() const {
			return physicsObject;
		}

		NetworkObject* GetNetworkObject() const {
			return networkObject;
		}

		void SetRenderObject(RenderObject* newObject) {
			renderObject = newObject;
		}

		void SetPhysicsObject(PhysicsObject* newObject) {
			physicsObject = newObject;
		}

		const std::string& GetName() const {
			return name;
		}

		void SetName(const std::string& val) {
			name = val;
		}

		int GetScore() {
			return objectScore;
		}

		virtual void OnCollisionBegin(GameObject* otherObject) {
			//std::cout << "OnCollisionBegin event occured!\n";
		}

		virtual void OnCollisionEnd(GameObject* otherObject) {
			//std::cout << "OnCollisionEnd event occured!\n";
		}

		bool GetBroadphaseAABB(Vector3&outsize) const;

		void UpdateBroadphaseAABB();

		void SetWorldID(int newID) {
			worldID = newID;
		}

		int		GetWorldID() const {
			return worldID;
		}

		void SetLayer(int id) {
			layerID = id;
		}

		int GetLayer() const {
			return layerID;
		}

		void SetPoint(bool val) {
			point = val;
		}

		bool GetPoint() {
			return point;
		}

		void IncreaseScore(int val) {
			objectScore += val;
		}

		int AddScore() {
			int tmp = objectScore;
			objectScore = 0;
			return tmp;
		}

	protected:
		Transform			transform;

		CollisionVolume*	boundingVolume;
		PhysicsObject*		physicsObject;
		RenderObject*		renderObject;
		NetworkObject*		networkObject;

		bool		isActive;
		int			worldID;
		int			layerID;
		bool		point;
		int			objectScore = 0;
		std::string	name;

		Vector3 broadphaseAABB;
	};
}

