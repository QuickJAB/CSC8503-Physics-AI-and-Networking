#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif
#include "PhysicsSystem.h"

#include "StateGameObject.h"

namespace NCL {
	namespace CSC8503 {
		class TutorialGame {
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);
			
			void UpdateMenu(float dt);
			void StartMenu();
			void StartSingle();

			int GetGameEnd() {
				return gameEnd;
			}

			int GetScore() {
				return soloScore;
			}

		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys();

			void InitWorld();
			void InitAI(std::vector<GameObject*> players);

			void UpdateAI(std::vector<GameObject*> players);

			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();

			GameObject* AddFloorToWorld(const Vector3& position, Vector3 dimensions);
			GameObject* AddObjectiveToWorld(const Vector3& position, float radius);
			GameObject* AddSphereToWorld(const Vector3& position, float radius);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f, bool OBB = false);
			GameObject* AddWallToWorld(const Vector3& position, Vector3 dimensions);
			GameObject* AddPlayerToWorld();
			GameObject* AddDoorToWorld(const Vector3& position, Vector3 dimensions);

			void AddFarmerToWorld(const Vector3& position, Vector3 dimensions, std::vector<GameObject*> targets);
			void AddGooseToWorld(const Vector3& position, std::vector<GameObject*> players, std::vector<GameObject*> farmers);

			StateGameObject* farmer = nullptr;
			BehaviourGameObject* goose = nullptr;

			int soloScore = 0;
			int gameEnd = 0;

			float gameTimer;

#ifdef USEVULKAN
			GameTechVulkanRenderer* renderer;
#else
			GameTechRenderer* renderer;
#endif
			PhysicsSystem* physics;
			GameWorld* world;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;

			GameObject* selectionObject = nullptr;

			MeshGeometry* capsuleMesh = nullptr;
			MeshGeometry* cubeMesh = nullptr;
			MeshGeometry* sphereMesh = nullptr;

			TextureBase* basicTex = nullptr;
			TextureBase* defaultTex = nullptr;
			TextureBase* objTex = nullptr;
			TextureBase* gooseTex = nullptr;
			ShaderBase* basicShader = nullptr;

			//Coursework Meshes
			MeshGeometry* charMesh = nullptr;
			MeshGeometry* enemyMesh = nullptr;
			MeshGeometry* bonusMesh = nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject = nullptr;
			Vector3 lockedOffset = Vector3(0, 1, 4);
			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}

			GameObject* objClosest = nullptr;
		};
	}
}

