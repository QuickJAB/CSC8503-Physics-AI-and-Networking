#include "NetworkedGame.h"
#include "NetworkPlayer.h"
#include "GameServer.h"
#include "GameClient.h"

#define COLLISION_MSG 30

struct MessagePacket : public GamePacket {
	short playerID;
	short messageID;

	MessagePacket() {
		type = Message;
		size = sizeof(short) * 2;
	}
};

NetworkedGame::NetworkedGame()	{
	thisServer = nullptr;
	thisClient = nullptr;

	NetworkBase::Initialise();
	timeToNextPacket  = 0.0f;
	packetsToSnapshot = 0;

	localPlayerID = -1;
}

NetworkedGame::~NetworkedGame()	{
	delete thisServer;
	delete thisClient;
}

void NetworkedGame::StartAsServer() {
	thisServer = new GameServer(NetworkBase::GetDefaultPort(), 4);

	thisServer->RegisterPacketHandler(Received_State, this);
	thisServer->RegisterPacketHandler(Client_Update, this);
	thisServer->RegisterPacketHandler(Spawn_Player, this);

	StartLevel();
}

void NetworkedGame::StartAsClient(char a, char b, char c, char d) {
	thisClient = new GameClient();
	thisClient->Connect(a, b, c, d, NetworkBase::GetDefaultPort());

	thisClient->RegisterPacketHandler(Delta_State, this);
	thisClient->RegisterPacketHandler(Full_State, this);
	thisClient->RegisterPacketHandler(Player_Connected, this);
	thisClient->RegisterPacketHandler(Player_Disconnected, this);

	thisClient->RegisterPacketHandler(Confirm_Spawn, this);
	thisClient->RegisterPacketHandler(Server_Update, this);

	StartLevel();
}

void NetworkedGame::UpdateGame(float dt) {
	timeToNextPacket -= dt;
	if (timeToNextPacket < 0) {
		if (thisServer) {
			UpdateAsServer(dt);
		}
		else if (thisClient) {
			UpdateAsClient(dt);
		}
		timeToNextPacket += 1.0f / 20.0f; //20hz server/client update
	}

	if (!thisServer && Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		StartAsServer();
	}
	if (!thisClient && Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		StartAsClient(127,0,0,1);
	}

	// Needs far more work, this is the rest function when the timer runs out on the server
	if (gameTimer - dt <= 0) {
		StartLevel();
	}

	TutorialGame::UpdateGame(dt);
}

void NetworkedGame::UpdateAsServer(float dt) {
	packetsToSnapshot--;
	if (packetsToSnapshot < 0) {
		BroadcastSnapshot(false);
		packetsToSnapshot = 5;
	}
	else {
		BroadcastSnapshot(true);
	}
	thisServer->UpdateServer();
}

void NetworkedGame::UpdateAsClient(float dt) {
	if (localPlayerID != -1) {
		ClientPacket packet(localPlayerID, localPlayer->GetTransform().GetPosition());
		thisClient->SendPacket(packet);
	}
	thisClient->UpdateClient();
}

void NetworkedGame::BroadcastSnapshot(bool deltaFrame) {
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;

	world->GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		//TODO - you'll need some way of determining
		//when a player has sent the server an acknowledgement
		//and store the lastID somewhere. A map between player
		//and an int could work, or it could be part of a 
		//NetworkPlayer struct. 
		int playerState = 0;
		GamePacket* newPacket = nullptr;
		if (o->WritePacket(&newPacket, deltaFrame, playerState)) {
			thisServer->SendGlobalPacket(*newPacket);
			delete newPacket;
		}
	}
}

void NetworkedGame::UpdateMinimumState() {
	//Periodically remove old data from the server
	int minID = INT_MAX;
	int maxID = 0; //we could use this to see if a player is lagging behind?

	for (auto i : stateIDs) {
		minID = min(minID, i.second);
		maxID = max(maxID, i.second);
	}
	//every client has acknowledged reaching at least state minID
	//so we can get rid of any old states!
	std::vector<GameObject*>::const_iterator first;
	std::vector<GameObject*>::const_iterator last;
	world->GetObjectIterators(first, last);

	for (auto i = first; i != last; ++i) {
		NetworkObject* o = (*i)->GetNetworkObject();
		if (!o) {
			continue;
		}
		o->UpdateStateHistory(minID); //clear out old states so they arent taking up memory...
	}
}

void NetworkedGame::SpawnPlayer() {
	if (thisClient) {
		localPlayer = AddPlayerToWorld();
	}
}

void NetworkedGame::StartLevel() {
	InitWorld();
	
	SpawnPlayer();

	std::vector<GameObject*> players;
	for (auto i = serverPlayers.begin(); i != serverPlayers.end(); i++) {
		players.push_back(i->second);
	}

	if (players.size() == 1 || players.size() == 0) {
		InitAI(players);
	}
	else {
		UpdateAI(players);
	}

	gameEnd = 0;
	gameTimer = 120;
}

void NetworkedGame::ReceivePacket(int type, GamePacket* payload, int source) {
	if (thisServer) {
		switch (payload->type) {
		case Spawn_Player:
			CreatePlayer();
			break;
		case Client_Update:
			int id = ((ClientPacket*)payload)->GetPlayerID();
			Vector3 pos = ((ClientPacket*)payload)->GetPosition();
			UpdateAndSend(id, pos);
			break;
		}
	}
	else if (thisClient) {
		switch (payload->type) {
		case Confirm_Spawn:
			ConfirmSpawn((ConfSpawnPacket*)payload);
			break;
		case Server_Update:
			UpdateClientPositions((ServerPacket*)payload);
			break;
		}
	}
		
}

void NetworkedGame::CreatePlayer() {
	int id = serverPlayers.size();
	std::cout << id << std::endl;
	ConfSpawnPacket packet(id);
	
	for (auto i = serverPlayers.begin(); i != serverPlayers.end(); i++) {
		packet.existingPlayers.push_back(i->first);
	}
	
	serverPlayers.emplace(id, AddPlayerToWorld());
	thisServer->SendGlobalPacket(packet);
}

void NetworkedGame::ConfirmSpawn(ConfSpawnPacket* payload) {
	if (localPlayerID == -1) {
		localPlayerID = payload->GetID();

		for (int i = 0; i < payload->existingPlayers.size(); i++) {
			serverPlayers.emplace(payload->existingPlayers[i], AddPlayerToWorld());
		}
	}
}

void NetworkedGame::OnPlayerCollision(NetworkPlayer* a, NetworkPlayer* b) {
	if (thisServer) { //detected a collision between players!
		MessagePacket newPacket;
		newPacket.messageID = COLLISION_MSG;
		newPacket.playerID  = a->GetPlayerNum();

		thisClient->SendPacket(newPacket);

		newPacket.playerID = b->GetPlayerNum();
		thisClient->SendPacket(newPacket);
	}
}

void NetworkedGame::UpdateAndSend(int id, Vector3 position) {
	if (id > -1 && id < serverPlayers.size()) {
		serverPlayers.at(id)->GetTransform().SetPosition(position);

		ServerPacket packet = ServerPacket();
		packet.playerID = id;
		packet.playerPos = position;
		thisServer->SendGlobalPacket(packet);
	}
}

void NetworkedGame::UpdateClientPositions(ServerPacket* packet) {
	if (packet->playerID == localPlayerID) {
		return;
	}
}