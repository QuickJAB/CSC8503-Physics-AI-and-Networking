#include "PhysicsSystem.h"
#include "PhysicsObject.h"
#include "GameObject.h"
#include "CollisionDetection.h"
#include "Quaternion.h"

#include "Constraint.h"

#include "Maths.h"

#include "Debug.h"
#include "Window.h"
#include <functional>
using namespace NCL;
using namespace CSC8503;

PhysicsSystem::PhysicsSystem(GameWorld& g) : gameWorld(g) {
	applyGravity = true;
	useBroadPhase = true;
	dTOffset = 0.0f;
	globalDamping = 0.995f;
	SetGravity(Vector3(0.0f, -9.8f, 0.0f));

	std::vector<bool> layerA;
	layerA.push_back(true);
	layerA.push_back(false);
	layerA.push_back(true);
	layers.push_back(layerA);

	std::vector<bool> layerB;
	layerB.push_back(false);
	layerB.push_back(false);
	layerB.push_back(false);
	layers.push_back(layerB);

	std::vector<bool> layerC;
	layerC.push_back(true);
	layerC.push_back(false);
	layerC.push_back(true);
	layers.push_back(layerC);
}

PhysicsSystem::~PhysicsSystem() {
}

void PhysicsSystem::SetGravity(const Vector3& g) {
	gravity = g;
}

/*

If the 'game' is ever reset, the PhysicsSystem must be
'cleared' to remove any old collisions that might still
be hanging around in the collision list. If your engine
is expanded to allow objects to be removed from the world,
you'll need to iterate through this collisions list to remove
any collisions they are in.

*/
void PhysicsSystem::Clear() {
	allCollisions.clear();
}

/*

This is the core of the physics engine update

*/

bool useSimpleContainer = true;

int constraintIterationCount = 10;

//This is the fixed timestep we'd LIKE to have
const int   idealHZ = 120;
const float idealDT = 1.0f / idealHZ;

/*
This is the fixed update we actually have...
If physics takes too long it starts to kill the framerate, it'll drop the
iteration count down until the FPS stabilises, even if that ends up
being at a low rate.
*/
int realHZ = idealHZ;
float realDT = idealDT;

void PhysicsSystem::Update(float dt) {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::B)) {
		useBroadPhase = !useBroadPhase;
		std::cout << "Setting broadphase to " << useBroadPhase << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::N)) {
		useSimpleContainer = !useSimpleContainer;
		std::cout << "Setting broad container to " << useSimpleContainer << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::I)) {
		constraintIterationCount--;
		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::O)) {
		constraintIterationCount++;
		std::cout << "Setting constraint iterations to " << constraintIterationCount << std::endl;
	}

	dTOffset += dt;

	GameTimer t;
	t.GetTimeDeltaSeconds();

	if (useBroadPhase) {
		UpdateObjectAABBs();
	}
	int iteratorCount = 0;
	while (dTOffset > realDT) {
		IntegrateAccel(realDT);
		ClearExtForces();
		if (useBroadPhase) {
			BroadPhase();
			NarrowPhase();
		}
		else {
			BasicCollisionDetection(realDT);
		}
		
		float constraintDt = realDT / (float)constraintIterationCount;
		for (int i = 0; i < constraintIterationCount; ++i) {
			UpdateConstraints(constraintDt);
		}

		//Wake(dt);

		IntegrateVelocity(realDT);

		dTOffset -= realDT;
		iteratorCount++;
	}

	//Sleep();

	ClearForces();	//Once we've finished with the forces, reset them to zero

	UpdateCollisionList(); //Remove any old collisions

	t.Tick();
	float updateTime = t.GetTimeDeltaSeconds();

	//Uh oh, physics is taking too long...
	if (updateTime > realDT) {
		realHZ /= 2;
		realDT *= 2;
		std::cout << "Dropping iteration count due to long physics time...(now " << realHZ << ")\n";
	}
	else if (dt * 2 < realDT) { //we have plenty of room to increase iteration count!
		int temp = realHZ;
		realHZ *= 2;
		realDT /= 2;

		if (realHZ > idealHZ) {
			realHZ = idealHZ;
			realDT = idealDT;
		}
		if (temp != realHZ) {
			std::cout << "Raising iteration count due to short physics time...(now " << realHZ << ")\n";
		}
	}
}

void PhysicsSystem::Sleep() {
	std::vector <GameObject*>::const_iterator first;
	std::vector <GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		if ((*i)->GetPhysicsObject() == nullptr) {
			continue;
		}

		PhysicsObject* obj = (*i)->GetPhysicsObject();

		if (!obj->IsAwake() || obj->GetWakeTimer() > 0) {
			continue;
		}

		bool colliding = false;
		for (auto j = allCollisions.begin(); j != allCollisions.end(); j++) {
			if ((*j).a == (*i) || (*j).b == (*i)) {
				colliding = true;
				break;
			}
		}

		if (colliding && fabs(obj->GetLinearVelocity().Length()) < 0.1 && fabs(obj->GetAngularVelocity().Length()) < 0.01) {
			obj->ToggleAwake();
		}
	}
}

void PhysicsSystem::Wake(float dt) {
	std::vector <GameObject*>::const_iterator first;
	std::vector <GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		if ((*i)->GetPhysicsObject() == nullptr) {
			continue;
		}
		
		PhysicsObject* obj = (*i)->GetPhysicsObject();

		if (obj->IsAwake()) {
			obj->TickWakeTimer(dt);
			continue;
		}

		if ((obj->GetForce().Length() > 0 || obj->GetTorque().Length() > 0)) {
			obj->ToggleAwake();
		}
	}
}

/*
Later on we're going to need to keep track of collisions
across multiple frames, so we store them in a set.

The first time they are added, we tell the objects they are colliding.
The frame they are to be removed, we tell them they're no longer colliding.

From this simple mechanism, we we build up gameplay interactions inside the
OnCollisionBegin / OnCollisionEnd functions (removing health when hit by a
rocket launcher, gaining a point when the player hits the gold coin, and so on).
*/
void PhysicsSystem::UpdateCollisionList() {
	for (std::set<CollisionDetection::CollisionInfo>::iterator i = allCollisions.begin(); i != allCollisions.end(); ) {
		if ((*i).framesLeft == numCollisionFrames) {
			i->a->OnCollisionBegin(i->b);
			i->b->OnCollisionBegin(i->a);
		}

		CollisionDetection::CollisionInfo& in = const_cast<CollisionDetection::CollisionInfo&>(*i);
		in.framesLeft--;

		if ((*i).framesLeft < 0) {
			i->a->OnCollisionEnd(i->b);
			i->b->OnCollisionEnd(i->a);
			i = allCollisions.erase(i);
		}
		else {
			++i;
		}
	}
}

void PhysicsSystem::UpdateObjectAABBs() {
	gameWorld.OperateOnContents(
		[](GameObject* g) {
			g->UpdateBroadphaseAABB();
		}
	);
}

CollisionDetection::ContactPoint PhysicsSystem::FindCentrePoint(std::vector<CollisionDetection::ContactPoint> points) const {
	CollisionDetection::ContactPoint centrePoint;

	//centrePoint.localA = points[0].localA;
	//centrePoint.localB = points[0].localB;
	//centrePoint.normal = points[0].normal;
	//centrePoint.penetration = points[0].penetration;
	//for (int i = 1; i < points.size(); i++) {
	//	centrePoint.localA += points[i].localA;
	//	centrePoint.localB += points[i].localB;
	//	centrePoint.normal += points[i].normal;
	//	centrePoint.penetration += points[i].penetration;
	//}
	//centrePoint.localA /= points.size();
	//centrePoint.localB /= points.size();
	//centrePoint.normal /= points.size();
	//centrePoint.penetration /= points.size();

	//return centrePoint;

	return points[points.size() - 1];
}

/*

This is how we'll be doing collision detection in tutorial 4.
We step thorugh every pair of objects once (the inner for loop offset
ensures this), and determine whether they collide, and if so, add them
to the collision set for later processing. The set will guarantee that
a particular pair will only be added once, so objects colliding for
multiple frames won't flood the set with duplicates.
*/
void PhysicsSystem::BasicCollisionDetection(float realDT) {
	std::vector <GameObject*>::const_iterator first;
	std::vector <GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);
	
	for (auto i = first; i != last; ++i) {
		for (auto j = i + 1; j != last; ++j) {

			if ((*i)->GetPhysicsObject() == nullptr && (*j)->GetPhysicsObject() == nullptr) {
				continue;
			}

			if (!layers[(*i)->GetLayer()][(*j)->GetLayer()]) {
				continue;
			}

			CollisionDetection::CollisionInfo info;
			info.a = *i;
			info.b = *j;
			for (auto colInfo = allCollisions.begin(); colInfo != allCollisions.end(); colInfo++) {
				if (info == *colInfo) {
					info = *colInfo;
					break;
				}
			}
			if (CollisionDetection::ObjectIntersection(*i, *j, info)) {

				if ((*i)->GetLayer() == 2 && (*i)->GetPoint() && ((*j)->GetName() == "player" || (*j)->GetName() == "goose")) {
					(*i)->SetPoint(false);
					(*i)->SetLayer(1);
					(*j)->IncreaseScore(1);
					continue;
				}
				else if ((*j)->GetLayer() == 2 && (*j)->GetPoint() && ((*i)->GetName() == "player" || (*i)->GetName() == "goose")) {
					(*j)->SetPoint(false);
					(*j)->SetLayer(1);
					(*i)->IncreaseScore(1);
					continue;
				}

				CollisionDetection::ContactPoint centrePoint = FindCentrePoint(info.point);
				if ((*i)->GetPhysicsObject() == nullptr || (*j)->GetPhysicsObject() == nullptr) {
					ImpulseResolveCollision(*info.a, *info.b, centrePoint);
				}
				else if ((*i)->GetPhysicsObject()->IsSpring() || (*j)->GetPhysicsObject()->IsSpring()) {
					ImpulseResolveSpringCollision(*info.a, *info.b, centrePoint);
				}
				else {
					ImpulseResolveCollision(*info.a, *info.b, centrePoint);
				}
				info.framesLeft = numCollisionFrames;
				allCollisions.insert(info);
			}
		}
	}
}

/*

In tutorial 5, we start determining the correct response to a collision,
so that objects separate back out.

*/
void PhysicsSystem::ImpulseResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const {
	PhysicsObject* physA = a.GetPhysicsObject();
	PhysicsObject* physB = b.GetPhysicsObject();

	Transform& transformA = a.GetTransform();
	Transform& transformB = b.GetTransform();

	float totalMass = 0;
	if (physA != nullptr) {
		totalMass += physA->GetInverseMass();
	}
	if (physB != nullptr) {
		totalMass += physB->GetInverseMass();
	}
	if (totalMass == 0) {
		return;
	}

	Vector3 relativeA = p.localA;
	Vector3 relativeB = p.localB;

	float invMA, crA, cfA;
	Vector3 angVelA, fullVelA, inertiaA;
	if (physA == nullptr) {
		invMA = 0;
		angVelA = Vector3();
		fullVelA = Vector3();
		inertiaA = Vector3();
		crA = 0;
		cfA = 0;
	} else {
		invMA = physA->GetInverseMass();
		angVelA = Vector3::Cross(physA->GetAngularVelocity(), relativeA);
		fullVelA = physA->GetLinearVelocity() + angVelA;
		inertiaA = Vector3::Cross(physA->GetInertiaTensor() * Vector3::Cross(relativeA, p.normal), relativeA);
		crA = physA->GetElasticity();
		cfA = physA->GetFriction();
	}

	float invMB, crB, cfB;
	Vector3 angVelB, fullVelB, inertiaB;
	if (physB == nullptr) {
		invMB = 0;
		angVelB = Vector3();
		fullVelB = Vector3();
		inertiaB = Vector3();
		crB = 0;
		cfB = 0;
	}
	else {
		invMB = physB->GetInverseMass();
		angVelB = Vector3::Cross(physB->GetAngularVelocity(), relativeB);
		fullVelB = physB->GetLinearVelocity() + angVelB;
		inertiaB = Vector3::Cross(physB->GetInertiaTensor() * Vector3::Cross(relativeB, p.normal), relativeB);
		crB = physB->GetElasticity();
		cfB = physB->GetFriction();
	}

	transformA.SetPosition(transformA.GetPosition() - p.normal * p.penetration * (invMA / totalMass));
	transformB.SetPosition(transformB.GetPosition() + p.normal * p.penetration * (invMB / totalMass));

	//if ((a.GetBoundingVolume()->type == VolumeType::Sphere && b.GetBoundingVolume()->type == VolumeType::OBB) ||
	//	(b.GetBoundingVolume()->type == VolumeType::Sphere && a.GetBoundingVolume()->type == VolumeType::OBB)) {
	//	return;
	//}

	Vector3 contactVel = fullVelB - fullVelA;

	float impulseForce = Vector3::Dot(contactVel, p.normal);
	float angEffect = Vector3::Dot(inertiaA + inertiaB, p.normal);
	float coefRest = (crA * 0.5f) + (crB * 0.5f);
	float j = (-(1 + coefRest) * impulseForce) / (totalMass + angEffect);
	Vector3 fullImpulse = p.normal * j;

	Vector3 tangent = (contactVel - (p.normal * Vector3::Dot(contactVel, p.normal))).Normalised();
	float frictionForce = Vector3::Dot(contactVel, tangent);
	float angFricEffect = Vector3::Dot(inertiaA + inertiaB, tangent);
	float coefFriction = (cfA * 0.5f) + (cfB * 0.5f);
	float fj = -(Vector3::Dot(contactVel * coefFriction, tangent)) / (totalMass + angEffect);
	Vector3 fullFrictionImpulse = tangent * fj;

	if (physA != nullptr) {
		physA->ApplyLinearImpulse(-fullImpulse);
		physA->ApplyLinearImpulse(-fullFrictionImpulse);
		physA->ApplyAngularImpulse(Vector3::Cross(relativeA, -fullImpulse));
	}
	
	if (physB != nullptr) {
		physB->ApplyLinearImpulse(fullImpulse);
		physB->ApplyLinearImpulse(fullFrictionImpulse);
		physB->ApplyAngularImpulse(Vector3::Cross(relativeB, fullImpulse));
	}
}

void PhysicsSystem::ImpulseResolveSpringCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const {
	PhysicsObject* physA = a.GetPhysicsObject();
	PhysicsObject* physB = b.GetPhysicsObject();

	float totalMass = 0;
	if (physA != nullptr) {
		totalMass += physA->GetInverseMass();
	}
	if (physB != nullptr) {
		totalMass += physB->GetInverseMass();
	}
	if (totalMass == 0) {
		return;
	}

	float k = 100.0f;

	Vector3 springForce = p.normal * (-k * -p.penetration);

	if (physA != nullptr) {
		physA->AddExtForceAtPosition(-springForce, p.localA);
	}
	if (physB != nullptr) {
		physB->AddExtForceAtPosition(springForce, p.localB);
	}
}

/*

Later, we replace the BasicCollisionDetection method with a broadphase
and a narrowphase collision detection method. In the broad phase, we
split the world up using an acceleration structure, so that we can only
compare the collisions that we absolutely need to.

*/
void PhysicsSystem::BroadPhase() {
	broadphaseCollisions.clear();
	QuadTree<GameObject*> tree(Vector2(1024, 1024), 7, 6);

	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);
	for (auto i = first; i != last; i++) {
		Vector3 halfSize;
		if (!(*i)->GetBroadphaseAABB(halfSize)) {
			continue;
		}
		Vector3 pos = (*i)->GetTransform().GetPosition();
		tree.Insert(*i, pos, halfSize);
	}
	tree.OperateOnContents(
		[&](std::list<QuadTreeEntry<GameObject*>>& data) {
			CollisionDetection::CollisionInfo info;
			for (auto i = data.begin(); i != data.end(); i++) {
				for (auto j = std::next(i); j != data.end(); j++) {
					info.a = std::min((*i).object, (*j).object);
					info.b = std::max((*i).object, (*j).object);
					broadphaseCollisions.insert(info);
				}
			}
		}
	);
}

/*

The broadphase will now only give us likely collisions, so we can now go through them,
and work out if they are truly colliding, and if so, add them into the main collision list
*/
void PhysicsSystem::NarrowPhase() {
	for (std::set<CollisionDetection::CollisionInfo>::iterator i = broadphaseCollisions.begin(); i != broadphaseCollisions.end(); i++) {
		CollisionDetection::CollisionInfo info = *i;
		
		if ((*i).a->GetPhysicsObject() == nullptr && (*i).b->GetPhysicsObject() == nullptr) {
			continue;
		}

		if (!layers[(*i).a->GetLayer()][(*i).b->GetLayer()]) {
			continue;
		}

		if (CollisionDetection::ObjectIntersection(info.a, info.b, info)) {
			if ((*i).a->GetLayer() == 2 && (*i).a->GetPoint() && ((*i).b->GetName() == "player" || (*i).b->GetName() == "goose")) {
				(*i).a->SetPoint(false);
				(*i).a->SetLayer(1);
				(*i).b->IncreaseScore(1);
				continue;
			}
			else if ((*i).b->GetLayer() == 2 && (*i).b->GetPoint() && ((*i).a->GetName() == "player" || (*i).b->GetName() == "goose")) {
				(*i).b->SetPoint(false);
				(*i).b->SetLayer(1);
				(*i).a->IncreaseScore(1);
				continue;
			}
			
			info.framesLeft = numCollisionFrames;
			CollisionDetection::ContactPoint centrePoint = FindCentrePoint(info.point);
			if ((*i).a->GetPhysicsObject() == nullptr || (*i).b->GetPhysicsObject() == nullptr) {
				ImpulseResolveCollision(*info.a, *info.b, centrePoint);
			}
			else if ((*i).a->GetPhysicsObject()->IsSpring() || (*i).b->GetPhysicsObject()->IsSpring()) {
				ImpulseResolveSpringCollision(*info.a, *info.b, centrePoint);
			}
			else {
				ImpulseResolveCollision(*info.a, *info.b, centrePoint);
			}
			allCollisions.insert(info);
		}
	}
}

/*
Integration of acceleration and velocity is split up, so that we can
move objects multiple times during the course of a PhysicsUpdate,
without worrying about repeated forces accumulating etc.

This function will update both linear and angular acceleration,
based on any forces that have been accumulated in the objects during
the course of the previous game frame.
*/
void PhysicsSystem::IntegrateAccel(float dt) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);

	for (auto i = first; i != last; i++) {
		PhysicsObject* object = (*i)->GetPhysicsObject();
		if (object == nullptr || !object->IsAwake()) {
			continue;
		}
		float inverseMass = object->GetInverseMass();

		Vector3 linearVel = object->GetLinearVelocity();
		Vector3 force = object->GetForce() + object->GetExtForce();

		Vector3 accel = force * inverseMass;

		if ((*i)->GetPhysicsObject()->IsGravity() && applyGravity && inverseMass > 0) {
			accel += gravity;
		}

		linearVel += accel * dt;
		object->SetLinearVelocity(linearVel);

		Vector3 torque = object->GetTorque() + object->GetExtTorque();
		Vector3 angVel = object->GetAngularVelocity();

		object->UpdateInertiaTensor();

		Vector3 angAccel = object->GetInertiaTensor() * torque;

		angVel += angAccel * dt;
		object->SetAngularVelocity(angVel);
	}
}

/*
This function integrates linear and angular velocity into
position and orientation. It may be called multiple times
throughout a physics update, to slowly move the objects through
the world, looking for collisions.
*/
void PhysicsSystem::IntegrateVelocity(float dt) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	gameWorld.GetObjectIterators(first, last);
	float frameLinearDamping = 1.0f - (GetLinearDampingFactor() * dt);
	float frameAngularCamping = 1.0f - (GetAngularDampingFactor() * dt);

	for (auto i = first; i != last; i++) {
		PhysicsObject* object = (*i)->GetPhysicsObject();
		if (object == nullptr || !object->IsAwake()) {
			continue;
		}

		Transform& transform = (*i)->GetTransform();

		Vector3 position = transform.GetPosition();
		Vector3 linearVel = object->GetLinearVelocity();
		linearVel = Maths::Clamp(linearVel, -object->GetMaxVel(), object->GetMaxVel());
		position += linearVel * dt;
		transform.SetPosition(position);

		linearVel *= frameLinearDamping;

		object->SetLinearVelocity(linearVel);

		Quaternion orientation = transform.GetOrientation();
		Vector3 angVel = object->GetAngularVelocity();

		orientation = orientation + (Quaternion(angVel * dt * 0.5f, 0.0f) * orientation);
		orientation.Normalise();

		transform.SetOrientation(orientation);

		object->SetAngularVelocity(angVel);
	}
}

/*
Once we're finished with a physics update, we have to
clear out any accumulated forces, ready to receive new
ones in the next 'game' frame.
*/
void PhysicsSystem::ClearForces() {
	gameWorld.OperateOnContents(
		[](GameObject* o) {
			if (o->GetPhysicsObject() == nullptr) {
				return;
			}
			o->GetPhysicsObject()->ClearForces();
		}
	);
}

void PhysicsSystem::ClearExtForces() {
	gameWorld.OperateOnContents(
		[](GameObject* o) {
			if (o->GetPhysicsObject() == nullptr) {
				return;
			}
			o->GetPhysicsObject()->ClearExtForces();
		}
	);
}

/*

As part of the final physics tutorials, we add in the ability
to constrain objects based on some extra calculation, allowing
us to model springs and ropes etc.

*/
void PhysicsSystem::UpdateConstraints(float dt) {
	std::vector<Constraint*>::const_iterator first;
	std::vector<Constraint*>::const_iterator last;
	gameWorld.GetConstraintIterators(first, last);

	for (auto i = first; i != last; ++i) {
		(*i)->UpdateConstraint(dt);
	}
}