#pragma once
#include <NavigationGrid.h>
#include "GameObject.h"
#include <GameWorld.h>

#include <BehaviourAction.h>
#include <BehaviourSequence.h>
#include <BehaviourSelector.h>

namespace NCL {
    namespace CSC8503 {

#pragma region BehaviourGameObject

        class BehaviourGameObject : public GameObject {
        public:
            BehaviourGameObject(GameWorld* world);
            ~BehaviourGameObject();

            void Update(float dt);

            void SetObjectives(std::vector<Vector3> objectives) {
                this->objectives = objectives;
            }

            void SetPlayers(std::vector<GameObject*> players) {
                this->players = players;
            }

            void SetFarmers(std::vector<GameObject*> farmers) {
                this->farmers = farmers;
            }

            bool HasCaughtPlayer() {
                return caughtPlayer;
            }

        protected:
            bool CreatePath(Vector3 target);
            
            bool FindObjective();
            void FindFarmer();
            void FindPlayer();
            
            bool FollowPath();
            bool InRange(Vector3 point);

            bool SeeTarget();
            bool ChaseTarget();

            BehaviourSequence* gooseSequence;
            BehaviourState state;

            NavigationGrid* grid;
            std::vector<Vector3> path;
            int nodeID;

            std::vector<Vector3> objectives;
            std::vector<GameObject*> players;
            std::vector<GameObject*> farmers;
            GameObject* target;

            float behaviourTimer;
            int prevScore;

            bool caughtPlayer = false;

            GameWorld* world;
        };

#pragma endregion

#pragma region StateGameObject

        class StateMachine;
        class StateGameObject : public GameObject  {
        public:
            StateGameObject(std::vector<GameObject*> targets, GameWorld* world);
            ~StateGameObject();

            virtual void Update(float dt);

            void SetTargets(std::vector<GameObject*> newTargets) {
                targets = newTargets;
            }

            void SetGoose(GameObject* goose) {
                this->goose = goose;
            }

        protected:
            void Chase(float dt);
            void Flee(float dt);
            void Patrol(float dt);

            bool InRange(Vector3 point);

            void BuildPath();

            StateMachine* stateMachine;
            int point;
            std::vector<Vector3> points;
            float speed = 2;
            
            std::vector<GameObject*> targets;
            int targetID;

            GameObject* goose = nullptr;

            vector<Vector3> path;
            int nodeID;

            GameWorld* world;
        };
    }

#pragma endregion
}
