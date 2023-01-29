#include "Window.h"

#include "Debug.h"

#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"

#include "GameServer.h"
#include "GameClient.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "TutorialGame.h"
#include "NetworkedGame.h"

#include "PushdownMachine.h"

#include "PushdownState.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

using namespace NCL;
using namespace CSC8503;

#include <chrono>
#include <thread>
#include <sstream>

void TestBehaviourTree() {
	float behaviourTimer;
	float distanceToTarget;
	BehaviourAction* findKey = new BehaviourAction("Find Key", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for key!\n";
			behaviourTimer = rand() % 100;
			state = Ongoing;
		}
		else if (state == Ongoing) {
			behaviourTimer -= dt;
			if (behaviourTimer <= 0.0f) {
				std::cout << "Found a key!\n";
				return Success;
			}
			return state;
		}
		});

	BehaviourAction* goToRoom = new BehaviourAction("Go To Room", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Going to the loot room!\n";
			state = Ongoing;
		}
		else if (state == Ongoing) {
			distanceToTarget -= dt;
			if (distanceToTarget <= 0.0f) {
				std::cout << "Reached room!\n";
				return Success;
			}
		}
		return state;
		});

	BehaviourAction* openDoor = new BehaviourAction("Open Door", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Opening Door!\n";
			return Success;
		}
		return state;
		});

	BehaviourAction* lookForTreasure = new BehaviourAction("Look For Treasure", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for treasure!\n";
			return Ongoing;
		}
		else if (state == Ongoing) {
			bool found = rand() % 2;
			if (found) {
				std::cout << "I found some treasure!\n";
				return Success;
			}
			std::cout << "No treasure in here...\n";
			return Failure;
		}
		return state;
		});

	BehaviourAction* lookForItems = new BehaviourAction("Look For Items", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for items!\n";
			return Ongoing;
		}
		else if (state == Ongoing) {
			bool found = rand() % 2;
			if (found) {
				std::cout << "I found some items!\n";
				return Success;
			}
			std::cout << "No items in here...\n";
			return Failure;
		}
		return state;
		});

	BehaviourSequence* sequence = new BehaviourSequence("Room Sequence");
	sequence->AddChild(findKey);
	sequence->AddChild(goToRoom);
	sequence->AddChild(openDoor);

	BehaviourSelector* selection = new BehaviourSelector("Loot Selection");
	selection->AddChild(lookForTreasure);
	selection->AddChild(lookForItems);

	BehaviourSequence* rootSequence = new BehaviourSequence("Root Sequence");
	rootSequence->AddChild(sequence);
	rootSequence->AddChild(selection);

	for (int i = 0; i < 5; i++) {
		rootSequence->Reset();
		behaviourTimer = 0.0f;
		distanceToTarget = rand() % 250;
		BehaviourState state = Ongoing;
		std::cout << "We're going on an adventure\n";
		while (state == Ongoing) {
			state = rootSequence->Execute(1.0f);
		}
		if (state == Success) {
			std::cout << "What a successful adventure!\n";
		}
		else if (state == Failure) {
			std::cout << "What a waste of time!\n";
		}
	}
	std::cout << "All done!\n";
}

class PauseScreen : public PushdownState {
public:
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::R)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {
		std::cout << "Press R to resume the game!\n";
	}
};

class GameScreen : public PushdownState {
public:
	GameScreen(Window* w, TutorialGame* g) {
		wind = w;
		game = g;
	}
	
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		wind->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));
		game->UpdateGame(dt);

		if (game->GetGameEnd() != 0) {
			return PushdownResult::Pop;
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::P)) {
			*newState = new PauseScreen();
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) {
			return PushdownResult::Pop;
		}

		return PushdownResult::NoChange;
	};
protected:
	Window* wind;
	TutorialGame* game;
};

class IntroScreen : public PushdownState {
public:
	IntroScreen(Window* w) {
		wind = w;
		sGame = new TutorialGame();
		nGame = new NetworkedGame();
	}
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		wind->SetTitle("Main Menu");

		Debug::Print("High Score: " + std::to_string(highScore), Vector2(5, 5), Vector4(1, 0, 0, 1));

		if (sGame->GetGameEnd() == -1) {
			Debug::Print("Your Score: " + std::to_string(score), Vector2(5, 10), Vector4(1, 0, 0, 1));
			Debug::Print("You lost :(, better luck next time", Vector2(5, 15), Vector4(1, 0, 0, 1));
		}
		else if (sGame->GetGameEnd() == 1) {
			Debug::Print("Your Score: " + std::to_string(score), Vector2(5, 10), Vector4(1, 0, 0, 1));
			Debug::Print("You WON! :), CONGRATULATIONS!!!", Vector2(5, 15), Vector4(1, 0, 0, 1));
		}

		Debug::Print("Press '1' to start in single player", Vector2(5, 20), Vector4(1, 0, 0, 1));
		Debug::Print("Press '2' to connect as a client", Vector2(5, 25), Vector4(1, 0, 0, 1));
		Debug::Print("Press '3' to create a server", Vector2(5, 30), Vector4(1, 0, 0, 1));
		Debug::Print("Press 'Escape' to quit", Vector2(5, 35), Vector4(1, 0, 0, 1));
		sGame->UpdateMenu(dt);

		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NUM1)) {
			sGame->StartSingle();
			*newState = new GameScreen(wind, sGame);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NUM2)) {
			nGame->StartLevel();
			nGame->StartAsClient(127, 0, 0, 1);
			*newState = new GameScreen(wind, nGame);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NUM3)) {
			nGame->StartLevel();
			nGame->StartAsServer();
			*newState = new GameScreen(wind, nGame);
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() {
		score = sGame->GetScore();
		if (sGame->GetGameEnd() == 1 && score > highScore) {
			highScore = score;
		}
		sGame->StartMenu();
	}
protected:
	Window* wind;
	NetworkedGame* nGame;
	TutorialGame* sGame;
	int score = 0;
	int highScore = 0;
};

/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead.

This time, we've added some extra functionality to the window class - we can
hide or show the

*/
int main() {
	Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720);

	if (!w->HasInitialised()) {
		return -1;
	}

	w->ShowOSPointer(false);
	w->LockMouseToWindow(true);

	w->GetTimer()->GetTimeDeltaSeconds();

	PushdownMachine machine(new IntroScreen(w));
	while (w->UpdateWindow()) {
		float dt = w->GetTimer()->GetTimeDeltaSeconds();
		if (!machine.Update(dt)) {
			break;
		}
	}

	Window::DestroyGameWindow();
}