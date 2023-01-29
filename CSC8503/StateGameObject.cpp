#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "PhysicsObject.h"
#include "Debug.h"

using namespace NCL;
using namespace CSC8503;

#pragma region BehaviourGameObject

BehaviourGameObject::BehaviourGameObject(GameWorld* world) {
	grid = new NavigationGrid("TestGrid1.txt");
	this->world = world;

	BehaviourAction* findObjective = new BehaviourAction("Find Objective", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			if (FindObjective()) {
				state = Ongoing;
			}
			else {
				return Failure;
			}
		}
		else if (state == Ongoing) {
			if (FollowPath()) {
				objectives.pop_back();
				return Success;
			}
		}
		return state;
	});

	BehaviourAction* chaseFarmer = new BehaviourAction("Chase Farmer", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			FindFarmer();
			behaviourTimer = 10;
			state = Ongoing;
			if (!CreatePath(target->GetTransform().GetPosition())) {
				return Failure;
			}
		}
		else if (state == Ongoing) {
			behaviourTimer -= dt;
			if (behaviourTimer <= 0) {
				return Success;
			}
			bool atTarget = false;
			if (!SeeTarget()) {
				atTarget = FollowPath();
			}
			else {
				atTarget = ChaseTarget();
			}
			if (atTarget) {
				return Success;
			}
		}
		return state;
	});

	BehaviourAction* huntPlayer = new BehaviourAction("Hunt Player", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			FindPlayer();
			behaviourTimer = 15;
			state = Ongoing;
			if (!CreatePath(target->GetTransform().GetPosition())) {
				return Failure;
			}
		}
		else if (state == Ongoing) {
			behaviourTimer -= dt;
			bool atTarget = false;
			if (SeeTarget()) {
				behaviourTimer = 15;
				atTarget = ChaseTarget();
			}
			else {
				atTarget = FollowPath();
			}
			if (behaviourTimer <= 0) {
				return Failure;
			}
			if (atTarget) {
				caughtPlayer = true;
				return Success;
			}
		}
		return state;
	});

	gooseSequence = new BehaviourSequence("Goose Sequence");
	gooseSequence->AddChild(findObjective);
	gooseSequence->AddChild(chaseFarmer);
	gooseSequence->AddChild(huntPlayer);

	state = Initialise;
}

BehaviourGameObject::~BehaviourGameObject() {
	delete gooseSequence;
	delete grid;
	delete target;
	objectives.clear();
	players.clear();
	farmers.clear();
}

void BehaviourGameObject::Update(float dt) {

	if (state == Initialise) {
		gooseSequence->Reset();
		state = Ongoing;
	}
	else if (state == Ongoing) {
		state = gooseSequence->Execute(dt);
	}
	else if (state == Success) {
		state = Ongoing;
	}
	else if (state == Failure) {
		state = Initialise;
	}

	Debug::DrawAxisLines(GetTransform().GetMatrix());

	for (int i = 1; i < path.size(); i++) {
		Vector3 a = path[i - 1];
		Vector3 b = path[i];

		Debug::DrawLine(a, b, Vector4(0, 0, 1, 1));
	}
}

bool BehaviourGameObject::CreatePath(Vector3 target) {
	NavigationPath outPath;
	path.clear();
	if (grid->FindPath(this->GetTransform().GetPosition(), target, outPath)) {
		Vector3 pos;
		while (outPath.PopWaypoint(pos)) {
			path.push_back(pos);
		}
		nodeID = 1;
		return true;
	}
	return false;
}

bool BehaviourGameObject::FindObjective() {
	if (objectives.size() > 0 && CreatePath(objectives.back())) {
		return true;
	}
	return false;
}

void BehaviourGameObject::FindFarmer() {
	float shortestDist = FLT_MAX;
	for (int i = 0; i < farmers.size(); i++) {
		float distToFarmer = (farmers[i]->GetTransform().GetPosition() - GetTransform().GetPosition()).Length();
		if (distToFarmer < shortestDist) {
			shortestDist = distToFarmer;
			target = farmers[i];
		}
	}
}

void BehaviourGameObject::FindPlayer() {
	float shortestDist = FLT_MAX;
	for (int i = 0; i < players.size(); i++) {
		float distToPlayer = (players[i]->GetTransform().GetPosition() - GetTransform().GetPosition()).Length();
		if (distToPlayer < shortestDist) {
			shortestDist = distToPlayer;
			target = players[i];
		}
	}
}

bool BehaviourGameObject::FollowPath() {
	if (InRange(path[nodeID])) {
		nodeID++;
	}

	if (nodeID == path.size()) {
		return true;
	}

	Vector3 direction = ((path[nodeID] - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();

	Vector3 right = Vector3(GetTransform().GetMatrix().GetColumn(0));
	float angle = Vector3::Dot(-right * Vector3(1, 0, 1), direction);
	Quaternion q = Quaternion::EulerAnglesToQuaternion(0, angle, 0);
	GetTransform().SetOrientation(GetTransform().GetOrientation() * q);
	
	float speed = 4.5f;
	float force = speed / GetPhysicsObject()->GetInverseMass();

	GetPhysicsObject()->AddForce(direction * force);

	return false;
}

bool BehaviourGameObject::InRange(Vector3 point) {
	Vector3 pos = GetTransform().GetPosition();
	float radius = 3;

	if ((pos - point).Length() <= radius) {
		return true;
	}
	return false;
}

bool BehaviourGameObject::SeeTarget() {
	Vector3 dir = ((target->GetTransform().GetPosition() - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();
	Ray ray = Ray(GetTransform().GetPosition(), dir);
	RayCollision col;
	Debug::DrawLine(GetTransform().GetPosition(), GetTransform().GetPosition() + (dir * 10));
	if (!world->Raycast(ray, col, true, this)) {
		return false;
	}
	if (col.node == this->target) {
		return true;
	}
	return false;
}

bool BehaviourGameObject::ChaseTarget() {
	if (InRange(target->GetTransform().GetPosition())) {
		return true;
	}
	
	float speed = 4.5f;
	float force = speed / GetPhysicsObject()->GetInverseMass();

	Vector3 direction = ((target->GetTransform().GetPosition() - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();

	Vector3 right = Vector3(GetTransform().GetMatrix().GetColumn(0));
	float angle = Vector3::Dot(-right * Vector3(1, 0, 1), direction);
	Quaternion q = Quaternion::EulerAnglesToQuaternion(0, angle, 0);
	GetTransform().SetOrientation(GetTransform().GetOrientation() * q);

	GetPhysicsObject()->AddForce(direction * force);

	return false;
}

#pragma endregion

#pragma region StateGameObject

StateGameObject::StateGameObject(std::vector<GameObject*> targets, GameWorld* world) {
	this->world = world;
	stateMachine = new StateMachine();
	this->targets = targets;
	
	State* stateA = new State([&](float dt)->void {
		this->Patrol(dt);
	});

	State* stateB = new State([&](float dt)->void {
		this->Chase(dt);
	});

	State* stateC = new State([&](float dt)->void {
		this->Flee(dt);
	});

	stateMachine->AddState(stateA);
	stateMachine->AddState(stateB);
	stateMachine->AddState(stateC);

	stateMachine->AddTransition(new StateTransition(stateA, stateB, [&]()->bool {
		for (int i = 0; i < this->targets.size(); i++) {
			Vector3 targetPos = this->targets[i]->GetTransform().GetPosition();
			if ((this->GetTransform().GetPosition() - targetPos).Length() >= 20) {
				continue;
			}
			Vector3 dir = (targetPos - this->GetTransform().GetPosition()).Normalised();
			Ray ray = Ray(this->GetTransform().GetPosition(), dir);
			RayCollision col;
			Debug::DrawLine(this->GetTransform().GetPosition(), this->GetTransform().GetPosition() + (dir * 10));
			if (!this->world->Raycast(ray, col, true, this)) {
				continue;
			}
			if (col.node == this->targets[i]) {
				targetID = i;
				return true;
			}
		}
		return false;
	}));

	stateMachine->AddTransition(new StateTransition(stateB, stateA, [&]()->bool {
		for (int i = 0; i < this->targets.size(); i++) {
			if ((this->GetTransform().GetPosition() - this->targets[i]->GetTransform().GetPosition()).Length() >= 40) {
				targetID = i;
				BuildPath();
				return true;
			}
		}
		return false;
	}));

	stateMachine->AddTransition(new StateTransition(stateA, stateC, [&]()->bool {
		if (goose != nullptr) {
			if ((this->GetTransform().GetPosition() - goose->GetTransform().GetPosition()).Length() < 25) {
				return true;
			}
			return false;
		}
		return false;
	}));

	stateMachine->AddTransition(new StateTransition(stateB, stateC, [&]()->bool {
		if (goose != nullptr) {
			if ((this->GetTransform().GetPosition() - goose->GetTransform().GetPosition()).Length() < 10) {
				return true;
			}
			return false;
		}
		return false;
		}));

	stateMachine->AddTransition(new StateTransition(stateC, stateA, [&]()->bool {
		if (goose != nullptr) {
			if ((this->GetTransform().GetPosition() - goose->GetTransform().GetPosition()).Length() >= 30) {
				return true;
			}
			return false;
		}
		return false;
	}));

	points.emplace_back(50, 0, 20);
	points.emplace_back(85, 0, 25);
	points.emplace_back(50, 0, 50);
	points.emplace_back(50, 0, 70);

	point = 1;
}

StateGameObject::~StateGameObject() {
	delete stateMachine;
}

void StateGameObject::BuildPath() {
	NavigationGrid grid("TestGrid1.txt");
	NavigationPath outPath;
	bool test = grid.FindPath(this->GetTransform().GetPosition(), points[point], outPath);
	Vector3 pos;
	path.clear();
	while (outPath.PopWaypoint(pos)) {
		path.push_back(pos);
	}
	nodeID = 1;
}

void StateGameObject::Update(float dt) {
	stateMachine->Update(dt);
	Debug::DrawAxisLines(GetTransform().GetMatrix());

	for (int i = 1; i < path.size(); i++) {
		Vector3 a = path[i - 1];
		Vector3 b = path[i];

		Debug::DrawLine(a, b, Vector4(1, 0, 0, 1));
	}
}

void StateGameObject::Chase(float dt) {
	float force = (speed * 3) / GetPhysicsObject()->GetInverseMass();
	
	Vector3 direction = ((targets[targetID]->GetTransform().GetPosition() - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();

	Vector3 right = Vector3(GetTransform().GetMatrix().GetColumn(0));
	float angle = Vector3::Dot(-right * Vector3(1, 0, 1), direction);
	Quaternion q = Quaternion::EulerAnglesToQuaternion(0, angle, 0);
	GetTransform().SetOrientation(GetTransform().GetOrientation() * q);


	GetPhysicsObject()->AddForce(direction * force);
}

void StateGameObject::Patrol(float dt) {
	if (InRange(points[point])) {
		point++;

		if (point >= points.size()) {
			point = 0;
		}

		BuildPath();
	}
	
	float force = speed / GetPhysicsObject()->GetInverseMass();
	
	if (path.size() == 0) {
		BuildPath();
	}

	if (InRange(path[nodeID])) {
		nodeID++;
	}
	
	Vector3 direction = ((path[nodeID] - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();
	
	Vector3 right = Vector3(GetTransform().GetMatrix().GetColumn(0));
	float angle = Vector3::Dot(-right * Vector3(1, 0, 1), direction);
	Quaternion q = Quaternion::EulerAnglesToQuaternion(0, angle, 0);
	GetTransform().SetOrientation(GetTransform().GetOrientation() * q);

	GetPhysicsObject()->AddForce(direction * force);
}

void StateGameObject::Flee(float dt) {
	Vector3 direction = -((goose->GetTransform().GetPosition() - GetTransform().GetPosition()) * Vector3(1, 0, 1)).Normalised();

	Vector3 right = Vector3(GetTransform().GetMatrix().GetColumn(0));
	float angle = Vector3::Dot(-right * Vector3(1, 0, 1), direction);
	Quaternion q = Quaternion::EulerAnglesToQuaternion(0, angle, 0);
	GetTransform().SetOrientation(GetTransform().GetOrientation() * q);

	float force = speed / GetPhysicsObject()->GetInverseMass();
	GetPhysicsObject()->AddForce(direction * force);
}

bool StateGameObject::InRange(Vector3 point) {
	Vector3 pos = GetTransform().GetPosition();
	float radius = 5;

	if ((pos - point).Length() <= radius) {
		return true;
	}
	return false;
}

#pragma endregion