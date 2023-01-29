#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"

#include "Assets.h"
#include <fstream>

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame() {
	world = new GameWorld();
#ifdef USEVULKAN
	renderer = new GameTechVulkanRenderer(*world);
#else 
	renderer = new GameTechRenderer(*world);
#endif

	physics = new PhysicsSystem(*world);

	forceMagnitude = 10.0f;
	useGravity = false;
	inSelectionMode = false;

	InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes,
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	cubeMesh = renderer->LoadMesh("cube.msh");
	sphereMesh = renderer->LoadMesh("sphere.msh");
	charMesh = renderer->LoadMesh("goat.msh");
	enemyMesh = renderer->LoadMesh("Keeper.msh");
	bonusMesh = renderer->LoadMesh("goose.msh");
	capsuleMesh = renderer->LoadMesh("capsule.msh");

	basicTex = renderer->LoadTexture("checkerboard.png");
	defaultTex = renderer->LoadTexture("Default.png");
	objTex = renderer->LoadTexture("Objective.png");
	gooseTex = renderer->LoadTexture("Goose.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");

	InitCamera();
	InitWorld();
}

TutorialGame::~TutorialGame() {
	delete cubeMesh;
	delete sphereMesh;
	delete charMesh;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete defaultTex;
	delete objTex;
	delete gooseTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
	}
	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
		
		float objYaw = lockedObject->GetTransform().GetOrientation().ToEuler().y;
		Vector3 camPos = (Matrix4::Rotation(objYaw, Vector3(0, 1, 0)) * Matrix4::Translation(lockedOffset)).GetPositionVector();
		camPos += objPos;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0, 1, 0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler();

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetYaw(angles.y);

		soloScore += lockedObject->AddScore();
		Debug::Print("Player: " + std::to_string(soloScore) + "/" + std::to_string(9 - goose->GetScore()), Vector2(5, 5));
	}

	if (soloScore == (9 - goose->GetScore())) {
		gameEnd = 1;
	}

	if (goose->HasCaughtPlayer()) {
		gameEnd = -1;
	}

	if (gameTimer <= 0) {
		gameEnd = -1;
	}

	gameTimer -= dt;

	UpdateKeys();

	RayCollision closestCollision;
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::K) && selectionObject) {
		Vector3 rayPos;
		Vector3 rayDir;

		rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

		rayPos = selectionObject->GetTransform().GetPosition();

		Ray r = Ray(rayPos, rayDir);

		if (world->Raycast(r, closestCollision, true, selectionObject)) {
			if (objClosest) {
				objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
			objClosest = (GameObject*)closestCollision.node;

			objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));
		}
	}

	SelectObject();
	MoveSelectedObject();

	Debug::Print("Goose: " + std::to_string(goose->GetScore()), Vector2(5, 10), Vector4(1, 0, 0, 1));
	Debug::Print("Time Remaining: " + std::to_string((int)gameTimer) + "s", Vector2(5, 15));

	if (farmer) {
		farmer->Update(dt);
	}
	if (goose) {
		goose->Update(dt);
	}

	world->UpdateWorld(dt);
	renderer->Update(dt);
	physics->Update(dt);

	renderer->Render();

	Debug::UpdateRenderables(dt);
}

void TutorialGame::StartMenu() {
	InitWorld();
	world->GetMainCamera()->SetPosition(Vector3(0, 7, 0));
	world->GetMainCamera()->SetYaw(225);
	world->GetMainCamera()->SetPitch(-5);
	gameTimer = 120;
	soloScore = 0;
	lockedObject = nullptr;
	selectionObject = nullptr;
}

void TutorialGame::UpdateMenu(float dt) {
	renderer->Update(dt);
	renderer->Render();
	Debug::UpdateRenderables(dt);
}

void TutorialGame::UpdateKeys() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	Camera* cam = world->GetMainCamera();
	
	float yaw = -(Window::GetMouse()->GetRelativePosition().x);

	if (yaw < 0) {
		yaw += 360.0f;
	}
	if (yaw > 360.0f) {
		yaw -= 360.0f;
	}

	lockedObject->GetTransform().SetOrientation(lockedObject->GetTransform().GetOrientation() * Matrix4::Rotation(yaw, Vector3(0, 1, 0)));

	Matrix4 view = cam->BuildViewMatrix();
	Matrix4 camWorld = view.Inverse();

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0));

	Vector3 fwdAxis = Vector3::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis.Normalise();

	PhysicsObject* physObj = selectionObject->GetPhysicsObject();

	float speed = 15;
	float moveForce = speed / lockedObject->GetPhysicsObject()->GetInverseMass();
	float jumpForce = 700;

	Debug::DrawAxisLines(lockedObject->GetTransform().GetMatrix());

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::W)) {
		physObj->AddForce(fwdAxis * moveForce);
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::S)) {
		physObj->AddForce(-fwdAxis * moveForce);
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::A)) {
		physObj->AddForce(-rightAxis * moveForce);
	}
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::D)) {
		physObj->AddForce(rightAxis * moveForce);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::SPACE)) {
		physObj->ApplyLinearImpulse(Vector3(0, jumpForce, 0));
	}
}

void TutorialGame::DebugObjectMovement() {
	//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(-60, 40, 60));
	lockedObject = nullptr;
}

void TutorialGame::StartSingle() {
	InitWorld();
	
	std::vector<GameObject*> players;
	GameObject* player = AddPlayerToWorld();
	players.push_back(player);

	InitAI(players);
	gameEnd = 0;
	selectionObject = player;
	selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
	lockedObject = player;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	std::ifstream infile(Assets::DATADIR + "TestGrid1.txt");

	int nodeSize, gridWidth, gridHeight;

	infile >> nodeSize;
	infile >> gridWidth;
	infile >> gridHeight;

	for (int y = 0; y < gridHeight; y++) {
		for (int x = 0; x < gridWidth; x++) {
			char type = 0;
			infile >> type;

			Vector3 position = Vector3(x * nodeSize, 0, y * nodeSize);

			if (type == 'x') {
				AddWallToWorld(position, Vector3(nodeSize * 0.5, nodeSize * 0.5, nodeSize * 0.5));
			}
			else if (type == 'd') {
				AddDoorToWorld(position, Vector3(nodeSize * 0.5, nodeSize * 0.5, nodeSize * 0.5));
			}
			else if (type == 'o') {
				AddObjectiveToWorld(position, nodeSize * 0.3);
			}
			else if (type == 'b') {
				AddCubeToWorld(position, Vector3(nodeSize * 0.49, nodeSize * 0.5, nodeSize * 0.49), 0.1, true);
			}
			else if (type == 's') {
				AddSphereToWorld(position, nodeSize * 0.2);
			}
		}
	}

	AddFloorToWorld(Vector3(0, 0, 0), Vector3(gridWidth, 1, gridHeight) * nodeSize);
}

void TutorialGame::InitAI(std::vector<GameObject*> players) {
	AddFarmerToWorld(Vector3(50, 2, 20), Vector3(0.5, 1, 0.5), players);

	std::vector<GameObject*> farmers;
	farmers.push_back(farmer);

	AddGooseToWorld(Vector3(75, 3, 90), players, farmers);

	farmer->SetGoose(goose);
}

void TutorialGame::UpdateAI(std::vector<GameObject*> players) {
	farmer->SetTargets(players);
	goose->SetPlayers(players);
}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* floor = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions / 2);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(dimensions)
		.SetPosition(position + (dimensions / 2) - (Vector3(0, 1, 0) * dimensions.y * 1.5));

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple'
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddObjectiveToWorld(const Vector3& position, float radius) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, objTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(2);
	sphere->GetPhysicsObject()->InitSphereInertia(true);
	sphere->SetLayer(2);
	sphere->SetPoint(true);

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position + Vector3(0, 4, 0));

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, nullptr, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(4);
	sphere->GetPhysicsObject()->InitSphereInertia(false);
	sphere->GetPhysicsObject()->ToggleSpring();

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass, bool OBB) {
	GameObject* cube = new GameObject();

	if (OBB) {
		OBBVolume* volume = new OBBVolume(dimensions);
		cube->SetBoundingVolume((CollisionVolume*)volume);
	}
	else {
		AABBVolume* volume = new AABBVolume(dimensions);
		cube->SetBoundingVolume((CollisionVolume*)volume);
	}

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddWallToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddPlayerToWorld() {
	float radius = 0.5;
	float inverseMass = 1;

	GameObject* character = new GameObject();
	SphereVolume* volume = new SphereVolume(radius);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(radius, radius, radius))
		.SetPosition(Vector3(5, 3, 5));

	character->SetRenderObject(new RenderObject(&character->GetTransform(), charMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	character->GetPhysicsObject()->SetElasticity(0.01);
	character->SetName("player");

	world->AddGameObject(character);

	return character;
}

void TutorialGame::AddGooseToWorld(const Vector3& position, std::vector<GameObject*> players, std::vector<GameObject*> farmers) {
	float radius = 0.4;
	float inverseMass = 1;

	goose = new BehaviourGameObject(world);

	SphereVolume* volume = new SphereVolume(radius);
	goose->SetBoundingVolume((CollisionVolume*)volume);

	goose->GetTransform()
		.SetScale(Vector3(radius, radius, radius))
		.SetPosition(position);

	goose->SetRenderObject(new RenderObject(&goose->GetTransform(), bonusMesh, gooseTex, basicShader));
	goose->SetPhysicsObject(new PhysicsObject(&goose->GetTransform(), goose->GetBoundingVolume()));

	goose->GetPhysicsObject()->SetInverseMass(inverseMass);
	goose->GetPhysicsObject()->InitSphereInertia();

	goose->SetName("goose");

	std::vector<Vector3> objectives;
	objectives.push_back(Vector3(90, 0, 10));
	objectives.push_back(Vector3(30, 0, 15));
	objectives.push_back(Vector3(60, 0, 20));
	objectives.push_back(Vector3(10, 0, 30));
	objectives.push_back(Vector3(80, 0, 45));
	objectives.push_back(Vector3(20, 0, 50));
	objectives.push_back(Vector3(30, 0, 65));
	objectives.push_back(Vector3( 5, 0, 75));
	objectives.push_back(Vector3(85, 0, 75));
	objectives.push_back(Vector3(45, 0, 90));

	goose->SetPlayers(players);
	goose->SetFarmers(farmers);
	goose->SetObjectives(objectives);

	world->AddGameObject(goose);
}

void TutorialGame::AddFarmerToWorld(const Vector3& position, Vector3 dimensions, std::vector<GameObject*> targets) {
	farmer = new StateGameObject(targets, world);

	OBBVolume* volume = new OBBVolume(dimensions);
	farmer->SetBoundingVolume((CollisionVolume*)volume);

	farmer->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	farmer->SetRenderObject(new RenderObject(&farmer->GetTransform(), enemyMesh, defaultTex, basicShader));
	farmer->SetPhysicsObject(new PhysicsObject(&farmer->GetTransform(), farmer->GetBoundingVolume()));

	farmer->GetPhysicsObject()->SetInverseMass(1.0f);
	farmer->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(farmer);

	OrientationConstraint* oriCon = new OrientationConstraint(farmer, farmer, Vector3(0, 1, 0));
	world->AddConstraint(oriCon);
}

GameObject* TutorialGame::AddDoorToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* door = AddCubeToWorld(position, dimensions* Vector3(0.85, 1, 0.25), 10, true);

	GameObject* hinge = AddCubeToWorld(position, Vector3(0.05, 0.05, 0.05), 0);
	hinge->SetLayer(1);

	OrientationConstraint* oriCon = new OrientationConstraint(hinge, door, Vector3(0, 1, 0));
	PositionConstraint* posCon = new PositionConstraint(hinge, door, 0);
	
	world->AddConstraint(oriCon);
	world->AddConstraint(posCon);

	return door;
}

/*
Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around.

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		Debug::Print("Press Q to change to camera mode!", Vector2(5, 85));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {
				selectionObject = (GameObject*)closestCollision.node;

				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				return true;
			}
			else {
				return false;
			}
		}
		if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
			if (selectionObject) {
				if (lockedObject == selectionObject) {
					lockedObject = nullptr;
				}
				else {
					lockedObject = selectionObject;
				}
			}
		}
	}
	else {
		Debug::Print("Press Q to change to select mode!", Vector2(5, 85));
	}
	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/
void TutorialGame::MoveSelectedObject() {
	Debug::Print("Click Force:" + std::to_string(forceMagnitude), Vector2(5, 90));
	forceMagnitude += Window::GetMouse()->GetWheelMovement() * 100.0f;

	if (!selectionObject) {
		return;//we haven't selected anything!
	}
	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());

		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			if (closestCollision.node == selectionObject) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
			}
		}
	}
}