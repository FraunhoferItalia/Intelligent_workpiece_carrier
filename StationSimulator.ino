#include <SPI.h>
#include "nfc_p2p.h"
#include <ArduinoJson.h>

#define MY_LUNA_ADDRESS 0x01
#define VEHICLE_TO_LOOK 0x03
#define PN532_CS 10
#define OCCUPY_BUTTON 4
#define CHANGE_STATION 3
#define RED_LED 7
#define GREEN_LED 6

nfc_p2p nfc(PN532_CS);
StaticJsonBuffer<360> jsonBuffer;
JsonObject& reportBackStation1 = jsonBuffer.createObject();
JsonObject& reportBackStation2 = jsonBuffer.createObject();
JsonObject& reportBackStation3 = jsonBuffer.createObject();
JsonObject& reportBackStation4 = jsonBuffer.createObject();

#define NUMBER_OF_PROCESSES 4

/*
SENDED VALUE START
*/
#define WPK_NAME_LEN 6
#define WPK_PREREQU_LEN 30
#define WPK_SUCC_LEN 3
#define WPK_N_OF_PARAMETERS 3

typedef struct station
{
	char name[WPK_NAME_LEN];				// Name of the station. Max 5 characters
	uint8_t ID;								// ID of the process

	uint8_t prerequ_len;					// Effective len prerequ
	char prerequ[WPK_PREREQU_LEN];			// Prerequisite 

	uint8_t succ_len;						// Effective len prerequ
	uint8_t succ[WPK_SUCC_LEN];				// List of possible successors


	uint8_t first;							// Parameter to say if this station can go first

	uint8_t parameters_len;					// Effective len of working parameters
	uint32_t parameters[WPK_N_OF_PARAMETERS];	// Parameters of control
	char writing[6];

}station;

typedef union {
	station  singleStation;
	char data_byte[sizeof(station)];
}unionParamSend;

unionParamSend process[NUMBER_OF_PROCESSES];
/*
SENDED VALUE END
*/


/**/

typedef struct returnedVar
{
	uint8_t ID;								// ID of the process
	uint8_t parameters_len;					// Effective len of working parameters
	uint32_t parameters[WPK_N_OF_PARAMETERS];	// Parameters of control
	unsigned long time;
	uint8_t numberOfOccupied;
}returnedVar;


typedef union {
	returnedVar  valuesOfLines;
	char data_byte[sizeof(returnedVar)];
}data;

data receivedParameters[NUMBER_OF_PROCESSES]; // struttura che ritorna i parametri
uint8_t number_of_time_transferred = 0;
/**/

typedef union {
	unsigned long timeBTWStations[NUMBER_OF_PROCESSES + 1];
	char data_byte[];
}returnedTimes;

returnedTimes returnedTimeBTWstations;


typedef union {
	uint8_t executed[NUMBER_OF_PROCESSES];
	char data_byte[];
}returnedExe;

returnedExe returnedExecutedOrder;

enum workingStationProcess {
	ENDED_INITIAL_UPLOADING_TO_WC,
	BASE_STATION_TO_WC_TRANSMIT_STRUCTURE,
	RESET,
	LOOK_FOR_IWC,
	LOOK_DOWLOADING_PARAM,
	START_PROCESS,
	END_PROCESS,
	UPLOADING_PARAM,
	GIVE_CONSENSUM_TO_UPLOADING,
	TRANSMIT_STRUCTURE_OF_VALUES,
	TRANSMIT_TIME_BETWEEN_STATIONS,
	TRANSMIT_EXECUTED_ARRAY,
	PROGRAM_FINISHED
};
enum workingStationProcess currentWSprocess = LOOK_FOR_IWC;

enum stations {
	START,
	BASE_STATION_TO_WC,
	BASE_STATION_FROM_WC,
	DRILLING,
	GLUING,
	SPRAYING,
	ENGRAVING
};

enum stations currentStation = START;


uint8_t valChangeStation;                        // variable for reading the pin status
uint8_t val2ChangeStation;                       // variable for reading the delayed status
uint8_t valOccupy;
uint8_t val2Occupy;
uint8_t buttonStateChangeStation;                // variable to hold the button state
uint8_t buttonStateOccupy;                // variable to hold the button state
uint8_t busy = 0;

enum NFC_state {
	READY_FOR_FIRMLOOP,
	FIRMLOOP,
	READY_FOR_SAM,
	SAMLOOP,
	READY_FOR_INRELEASE,
	INRELEASE_TARGET,
	READY_FOR_LOOK_FOR_TARGET,
	LOOK_FOR_TARGET,
	READY_FOR_INDATAEXCHANGE,
	SENDED_MESSAGE_TO_TARGET,
	RECEIVED_ANSWER_FROM_TARGET,
	
	ACTIVE
};

enum NFC_state currentNFCstate = READY_FOR_FIRMLOOP;

void cbFirmwareVersion(uint8_t ic, uint8_t ver, uint8_t rev, uint8_t support) {

	Serial.print(F("Firmware version IC="));
	Serial.print(ic, HEX);
	Serial.print(F(", Ver="));
	Serial.print(ver, HEX);
	Serial.print(F(", Rev="));
	Serial.print(rev, HEX);
	Serial.print(F(", Support="));
	Serial.println(support, HEX);
	
}

void cbFirmwareVersionLoop(uint8_t ic, uint8_t ver, uint8_t rev, uint8_t support) {

	//Serial.print(F("Firmwareloop version IC="));
	//Serial.print(ic, HEX);
	//Serial.print(F(", Ver="));
	//Serial.print(ver, HEX);
	//Serial.print(F(", Rev="));
	//Serial.print(rev, HEX);
	//Serial.print(F(", Support="));
	//Serial.println(support, HEX);
	currentNFCstate = READY_FOR_SAM;

}


void cbSamConfigLoop(void) {
	//Serial.println(F("Samloop config executed"));
	currentNFCstate = READY_FOR_LOOK_FOR_TARGET;
	currentWSprocess = LOOK_FOR_IWC;
}



uint8_t ATR_REQ[] = { 0x00,0xFF,0xFF,0x00,0x00 };
uint8_t RFConfigData[] = { 0x00,0x0A,0x09 };
uint8_t flag = 0;
uint8_t GEN[] = { 0x00,0x00 };// INdirizzo del PAD e busy status
uint8_t yes_msg[] = {0x89};
char dataOut[250];


void cbSamConfig(void) {
	Serial.println(F("Sam config executed"));
}
/*void cbInRelease(uint8_t statusCommand) {
	if (statusCommand != 0x00) {
		Serial.println(F("error Status"));
	}
	else
	{
		Serial.println(F("Released"));
		currentWSprocess = LOOK_FOR_IWC;
		currentNFCstate = READY_FOR_LOOK_FOR_TARGET;
	}
}*/


void cbInitAsInitiator(uint8_t statusCommand, uint8_t targetNumber, uint8_t* nfcid3t, uint8_t did, uint8_t bst, uint8_t brt, uint8_t to, uint8_t ppt, uint8_t lenGt, uint8_t* Gt)
{
	//Serial.println(F("inside callback"));
	if (statusCommand != 0x00) {

		Serial.println(F("error in cb init as initiator in status"));
	}
	else {
		//Serial.println(F("cb init as initiator inizializzator"));
		//Serial.println(lenGt, HEX);
		//Serial.println(*Gt, HEX);

		switch(currentStation){
		case BASE_STATION_TO_WC:
			if (*Gt == GEN[0])
			{
				Serial.println(F("Right Workpiece carrier found. Proceed"));
				Serial.print(F("Station needed: "));
				Serial.print(*Gt, HEX);
				Serial.println("");
				currentNFCstate = READY_FOR_INDATAEXCHANGE;
				currentWSprocess = BASE_STATION_TO_WC_TRANSMIT_STRUCTURE;
			}
			else {
				Serial.println(F("Not right station for this worpiece carrier"));
				delay(2000);
				//Serial.println(*Gt, HEX);
				currentNFCstate = READY_FOR_FIRMLOOP;
				currentWSprocess = RESET;
			}

			break;
		case BASE_STATION_FROM_WC:
			if (*Gt == GEN[0] )
			{
				Serial.println(F("Right Workpiece carrier found. Proceed"));
				Serial.print(F("Station needed: "));
					Serial.print(*Gt, HEX);
				Serial.println("");
				currentNFCstate = READY_FOR_INDATAEXCHANGE;
				currentWSprocess = TRANSMIT_STRUCTURE_OF_VALUES;
			}
			else {
				Serial.println(F("Not right station for this worpiece carrier"));
				delay(2000);
				//Serial.println(*Gt, HEX);
				currentNFCstate = READY_FOR_FIRMLOOP;
				currentWSprocess = RESET;
			}
			break;
		default:
			if (*Gt == GEN[0] && busy == false)
			{
				Serial.println(F("Right Workpiece carrier found. Proceed"));
				Serial.print(F("Station needed: "));
					Serial.print(*Gt, HEX);
				Serial.println("");
				currentNFCstate = READY_FOR_INDATAEXCHANGE;
				currentWSprocess = LOOK_DOWLOADING_PARAM;
			}
			else if (*Gt == GEN[0] && busy == true) {
				Serial.println(F("Right Workpiece carrier found but BUSY station"));
				delay(2000);
				currentNFCstate = READY_FOR_FIRMLOOP;
				currentWSprocess = RESET;
			}

			else {
				Serial.println(F("Not right station for this worpiece carrier"));
				delay(2000);
				//Serial.println(*Gt, HEX);
				currentNFCstate = READY_FOR_FIRMLOOP;
				currentWSprocess = RESET;
			}
			break;
		}
	
	}
}


void cbInitiatorRx(uint8_t statusCommand, uint16_t len, uint8_t* dataIn) {
	if (statusCommand != 0x00) {
		Serial.println(F("Error in cb initiatorRx"));
	}
	else {
		//Serial.println(F("Initiator data received"));

		//for (uint8_t i = 0; i<len; i++) {
		//	Serial.println(dataIn[i], HEX);
		//}



		switch (currentWSprocess)
		{

		case BASE_STATION_TO_WC_TRANSMIT_STRUCTURE :
			if (dataIn[0] == 0x89)
			{
				if (number_of_time_transferred < NUMBER_OF_PROCESSES)
				{
					currentNFCstate = READY_FOR_INDATAEXCHANGE;
					
				}
				
				else {
					Serial.println(F("DONE UPLOADING TO IWC"));
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = ENDED_INITIAL_UPLOADING_TO_WC;
					number_of_time_transferred = 0;
				}
			}
			break;

		case TRANSMIT_STRUCTURE_OF_VALUES:
			for (uint8_t i = 0; i < len; i++) {

				



				receivedParameters[number_of_time_transferred-1].data_byte[i] = dataIn[i]; // struttura che ritorna i parametri
			}
			

			if (number_of_time_transferred < NUMBER_OF_PROCESSES) {
				currentNFCstate = READY_FOR_INDATAEXCHANGE;
				}
			
			else {
				currentWSprocess = TRANSMIT_TIME_BETWEEN_STATIONS;
				currentNFCstate = READY_FOR_INDATAEXCHANGE;
			}
			

			break;
		case TRANSMIT_TIME_BETWEEN_STATIONS:
			for (uint8_t i = 0; i < len; i++) {
				returnedTimeBTWstations.data_byte[i] = dataIn[i]; // struttura che ritorna i parametri
			}

			currentWSprocess = TRANSMIT_EXECUTED_ARRAY;
			currentNFCstate = READY_FOR_INDATAEXCHANGE;
			break;
		case TRANSMIT_EXECUTED_ARRAY:

			for (uint8_t i = 0; i < len; i++) {
				returnedExecutedOrder.data_byte[i] = dataIn[i]; // struttura che ritorna i parametri
			}
			currentWSprocess = END_PROCESS;
			currentNFCstate = READY_FOR_FIRMLOOP;
			break;


		case LOOK_DOWLOADING_PARAM:
		{JsonObject& root = jsonBuffer.parseObject((char *)dataIn);

		if (!root.success())
		{
			Serial.println(F("parseObject() failed"));
			return;
		}

		switch (currentStation)
		{
		case START:

			break;
		case ENGRAVING:
		{
			long  spindVel = root["spindVel"];
			long  diam = root["Diam"];
			long  vel = root["HeadVel"];
			char word[6];
			strcpy(word, root["Word"]);

			//Serial.println(spindVel);
			//Serial.println(diam);
			//Serial.println(vel);
			//Serial.println(word);
			root.prettyPrintTo(Serial);

		}
		break;
		case BASE_STATION_FROM_WC:

			break;
		case DRILLING:
		{
			long  spindVel = root["spindVel"];
			long  diam = root["Diam"];
			long  vel = root["HeadVel"];
			//Serial.println(spindVel);
			//Serial.println(diam);
			//Serial.println(vel);
			root.prettyPrintTo(Serial);
		}

		break;
		case GLUING:
		{
			long  typeGlu = root["TypeGlu"];
			long  diam = root["Diam"];
			long  vel = root["HeadVel"];
			//Serial.println(typeGlu);
			//Serial.println(diam);
			//Serial.println(vel);
			root.prettyPrintTo(Serial);
		}
		break;
		case SPRAYING:
		{
			long  typeCol = root["TypeCol"];
			long  vel = root["HeadVel"];
			//Serial.println(typeCol);
			//Serial.println(vel);
			root.prettyPrintTo(Serial);
		}
		break;
		default:
			break;
		}

		//received Json from target
		currentNFCstate = RECEIVED_ANSWER_FROM_TARGET;
		currentWSprocess = START_PROCESS; }
			
			break;
		case UPLOADING_PARAM:
			if (dataIn[0] == 0x89)
			{
				currentNFCstate = READY_FOR_FIRMLOOP;
				currentWSprocess = RESET;
			}
			else {

				Serial.println(F("Problem in uploading par in WC"));
			}
			break;
		default:
			break;
		}	
	}

}


void cbRF(void) {
	Serial.println(F("RF config executed"));
}

void setup(void) {
	//STATION A
	strcpy(process[0].singleStation.name, "DRILL");
	process[0].singleStation.prerequ_len = 0;
	process[0].singleStation.prerequ[0] = NULL;

	process[0].singleStation.ID = 0x01;

	process[0].singleStation.succ_len = 3;

	process[0].singleStation.succ[0] = 0x02;
	process[0].singleStation.succ[1] = 0x03;
	process[0].singleStation.succ[2] = 0x04;

	process[0].singleStation.first = 1;

	process[0].singleStation.parameters_len = 3;
	process[0].singleStation.parameters[0] = 3600; // Spindle velocity
	process[0].singleStation.parameters[1] = 8;// Drilling diameter
	process[0].singleStation.parameters[2] = 56;// Drilling velocity


								  //STATION B
	strcpy(process[1].singleStation.name, "GLU01");
	process[1].singleStation.prerequ_len = 5;
	strcpy(process[1].singleStation.prerequ, "im(1)");

	process[1].singleStation.ID = 0x02;


	process[1].singleStation.succ_len = 2;

	process[1].singleStation.succ[0] = 0x03;
	process[1].singleStation.succ[1] = 0x04;

	process[1].singleStation.first = 0;

	process[1].singleStation.parameters_len = 3;
	process[1].singleStation.parameters[0] = 2; // Type glue
	process[1].singleStation.parameters[1] = 3;// Diameter mm 
	process[1].singleStation.parameters[2] = 60;// Gluing velocity mm/s

								  //STATION C
	strcpy(process[2].singleStation.name, "SPRY1");
	process[2].singleStation.prerequ_len = 23;

	process[2].singleStation.ID = 0x03;


	strcpy(process[2].singleStation.prerequ, "im(1)&(!im(4)|im(2))");

	process[2].singleStation.succ_len = 2;

	process[2].singleStation.succ[0] = 0x02;
	process[2].singleStation.succ[1] = 0x04;

	process[2].singleStation.first = 0;

	process[2].singleStation.parameters_len = 2;
	process[2].singleStation.parameters[0] = 141; // color type
	process[2].singleStation.parameters[1] = 90; // velocity


								   //STATION D
	strcpy(process[3].singleStation.name, "ENGRv");
	process[3].singleStation.prerequ_len = 0;
	strcpy(process[3].singleStation.prerequ, "im(1)&(!im(3)|im(2))");

	process[3].singleStation.ID = 0x04;


	process[3].singleStation.succ_len = 2;

	process[3].singleStation.succ[0] = 0x02;
	process[3].singleStation.succ[1] = 0x03;
	process[3].singleStation.first = 0;

	process[3].singleStation.parameters_len = 3;
	process[3].singleStation.parameters[0] = 3600; // Spindle velocity
	process[3].singleStation.parameters[1] = 8;// Drilling diameter
	process[3].singleStation.parameters[2] = 56;// Drilling velocity
	strcpy(process[3].singleStation.writing, "UNITN"); // Personal writing



	pinMode(OCCUPY_BUTTON, INPUT);
	pinMode(CHANGE_STATION, INPUT);

	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);

	buttonStateChangeStation = digitalRead(CHANGE_STATION);
	buttonStateOccupy = digitalRead(OCCUPY_BUTTON);
	digitalWrite(GREEN_LED, HIGH);

	Serial.begin(115200);

	//Serial.println(F("Hello, I am initiator!"));
	nfc.GetFirmwareVersion(&cbFirmwareVersion);
	nfc.update();

	//Serial.println("call sam!"); //sam could not be used in normal mode in dep,so our case

	nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfig);
	nfc.update();

	//nfc.RFConfiguration(0x02, RFConfigData, cbRF);
	//nfc.update();
}

void loop(void) {
	delay(100);
	valOccupy = digitalRead(OCCUPY_BUTTON);      // read input value and store it in val
	delay(10);                         // 10 milliseconds is a good amount of time
	val2Occupy = digitalRead(OCCUPY_BUTTON);     // read the input again to check for bounces
	if (valOccupy == val2Occupy) {                 // make sure we got 2 consistant readings!
		if (valOccupy != buttonStateOccupy) {          // the button state has changed!
			if (valOccupy == LOW) {                // check if the button is pressed
				if (busy == 0) {// light is off
					busy = 1;               // turn light on!
					digitalWrite(RED_LED, HIGH);
					digitalWrite(GREEN_LED, LOW);
				}
				else {
					busy = 0;               // turn light off!
					digitalWrite(RED_LED, LOW);
					digitalWrite(GREEN_LED, HIGH);
				}
			}
		}
		buttonStateOccupy = valOccupy;                 // save the new state in our variable
	}


	valChangeStation = digitalRead(CHANGE_STATION);      // read input value and store it in val
	delay(10);                         // 10 milliseconds is a good amount of time
	val2ChangeStation = digitalRead(CHANGE_STATION);     // read the input again to check for bounces
	if (valChangeStation == val2ChangeStation) {                 // make sure we got 2 consistant readings!
		if (valChangeStation != buttonStateChangeStation) { // the button state has changed!
			if (valChangeStation == LOW) {
				switch (currentStation)// In realtä stato precedente
				{
				case START:
					currentStation = ENGRAVING;
					
				case ENGRAVING:
					Serial.println(F("Simulated: BASE_STATION_FROM_IWC"));
					GEN[0] = { 0x00 };
					currentStation = BASE_STATION_FROM_WC;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				case  BASE_STATION_FROM_WC:
					Serial.println(F("Simulated: BASE_STATION_TO_WC"));
					GEN[0] = { 0x00 };
					currentStation = BASE_STATION_TO_WC;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				case BASE_STATION_TO_WC: //nel caso che il precedente sia base_station, ora il sucessivo deve essere drilling
					Serial.println(F("Simulated: DRILLING"));
					GEN[0] = {0x01};
					currentStation = DRILLING;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				case DRILLING:
					Serial.println(F("Simulated: GLUING"));
					GEN[0] = { 0x02 };
					currentStation = GLUING;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				case GLUING:
					Serial.println(F("Simulated: SPRAYING"));
					GEN[0] = { 0x03 };
					currentStation = SPRAYING;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				case SPRAYING:
					Serial.println(F("Simulated: ENGRAVING"));
					GEN[0] = { 0x04 };
					currentStation = ENGRAVING;
					currentNFCstate = READY_FOR_FIRMLOOP;
					currentWSprocess = RESET;
					nfc.abortPreviousCommand();
					break;
				default:
					break;
				}
			}
		}
		buttonStateChangeStation = valChangeStation;                 // save the new state in our variable
	}


	switch (currentStation)
	{
	case BASE_STATION_TO_WC:
		switch (currentWSprocess) {
		case RESET:
			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				//Serial.println(GEN[0]);
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				//Serial.println(F("Injumpfordep base_to_wc"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;
		case BASE_STATION_TO_WC_TRANSMIT_STRUCTURE:

			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				memset(dataOut, 0, sizeof(dataOut));
				for (uint8_t i = 0; i<sizeof(process[0]); i++) {
					dataOut[i] = process[number_of_time_transferred].data_byte[i]; // struttura che ritorna i parametri
				}
				nfc.InDataExchange(0x01, sizeof(process[0]), (uint8_t*)dataOut, &cbInitiatorRx);
				//Serial.println(F("Indataexchange base_to_wc!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
				number_of_time_transferred++;
			}
			break;
		case ENDED_INITIAL_UPLOADING_TO_WC:
		default:
			break;

		}
		break;

	case BASE_STATION_FROM_WC:
		switch (currentWSprocess) // Start working
		{
		case RESET:

			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				//Serial.println(GEN[0]);
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				//Serial.println(F("Injumpfordep base_from_wc"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;

		case TRANSMIT_STRUCTURE_OF_VALUES:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE ) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange base_from_wc!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
				number_of_time_transferred++;
			}

			break;

		case TRANSMIT_TIME_BETWEEN_STATIONS:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange time!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
				
			}
			break;
		case TRANSMIT_EXECUTED_ARRAY:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange trans exe!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
				
			}

			break;
		case END_PROCESS:
			Serial.println("");
			Serial.println(F("Parameters from stations"));
			Serial.print(F("Station ID "));
			Serial.print(receivedParameters[0].valuesOfLines.ID);
			Serial.println("");
			Serial.print(F("Spindle speed "));
			Serial.print(receivedParameters[0].valuesOfLines.parameters[0]);
			Serial.println("");
			Serial.print(F("Diameter hole "));
			Serial.print(receivedParameters[0].valuesOfLines.parameters[1]);
			Serial.println("");
			Serial.print(F("Penetration speed "));
			Serial.print(receivedParameters[0].valuesOfLines.parameters[2]);
			Serial.println("");
			Serial.print(F("Duration time "));
			Serial.print(receivedParameters[0].valuesOfLines.time);
			Serial.println("");
			Serial.print(F("N time busy "));
			Serial.print(receivedParameters[0].valuesOfLines.numberOfOccupied);
			Serial.println("");
			Serial.println("");

			Serial.print(F("Station ID "));
			Serial.print(receivedParameters[1].valuesOfLines.ID);
			Serial.println("");
			//Serial.println(receivedParameters[1].valuesOfLines.parameters_len);
			Serial.print(F("Type glue "));
			Serial.print(receivedParameters[1].valuesOfLines.parameters[0]);
			Serial.println("");
			Serial.print(F("Diameter hole "));
			Serial.print(receivedParameters[1].valuesOfLines.parameters[1]);
			Serial.println("");
			Serial.print(F("Head speed "));
			Serial.print(receivedParameters[1].valuesOfLines.parameters[2]);
			Serial.println("");
			Serial.print(F("Duration time "));
			Serial.print(receivedParameters[1].valuesOfLines.time);
			Serial.println("");
			Serial.print(F("N time busy "));
			Serial.print(receivedParameters[1].valuesOfLines.numberOfOccupied);
			Serial.println("");
			Serial.println("");
			
			Serial.print(F("Station ID "));
			Serial.print(receivedParameters[2].valuesOfLines.ID);
			Serial.println("");
			//Serial.print(receivedParameters[2].valuesOfLines.parameters_len);
			Serial.print(F("Type of color "));
			Serial.print(receivedParameters[2].valuesOfLines.parameters[0]);
			Serial.println("");
			Serial.print(F("Head speed "));
			Serial.print(receivedParameters[2].valuesOfLines.parameters[1]);
			Serial.println("");
			Serial.print(F("Duration time "));
			Serial.print(receivedParameters[2].valuesOfLines.time);
			Serial.println("");
			Serial.print(F("N time busy "));
			Serial.print(receivedParameters[2].valuesOfLines.numberOfOccupied);
			Serial.println("");
			Serial.println("");
			
			Serial.print(F("Station ID "));
			Serial.print(receivedParameters[3].valuesOfLines.ID);
			Serial.println("");
			//Serial.println(receivedParameters[3].valuesOfLines.parameters_len);
			Serial.print(F("Spindle speed "));
			Serial.print(receivedParameters[3].valuesOfLines.parameters[0]);
			Serial.println("");
			Serial.print(F("Diameter hole "));
			Serial.print(receivedParameters[3].valuesOfLines.parameters[1]);
			Serial.println("");
			Serial.print(F("Head speed "));
			Serial.print(receivedParameters[3].valuesOfLines.parameters[2]);
			Serial.println("");
			Serial.print(F("Duration time "));
			Serial.print(receivedParameters[3].valuesOfLines.time);
			Serial.println("");
			Serial.print(F("N time busy "));
			Serial.print(receivedParameters[3].valuesOfLines.numberOfOccupied);
			Serial.println("");
			Serial.println("");

			{
				uint8_t jj = 0;
				Serial.println(F("Time between stations"));

				Serial.print(F("Time between Base station and station "));
				Serial.print(returnedExecutedOrder.executed[jj]);
				Serial.print(F(" = "));
				Serial.print(returnedTimeBTWstations.timeBTWStations[jj++]);
				Serial.print(F(" millsec "));
				Serial.println("");

				while (jj < 4 ) {
					Serial.print(F("Time between  "));
					Serial.print(returnedExecutedOrder.executed[jj-1]);
					Serial.print(F(" and station  "));
					Serial.print(returnedExecutedOrder.executed[jj]);
					Serial.print(F(" = "));
					Serial.print(returnedTimeBTWstations.timeBTWStations[jj]);
					Serial.print(F(" millsec "));
					Serial.println("");
					jj++;
				}

				Serial.print(F("Time between  "));
				Serial.print(returnedExecutedOrder.executed[jj-1]);
				Serial.print(F(" and Base station = "));
				Serial.print(returnedTimeBTWstations.timeBTWStations[jj]);
				Serial.print(F(" millsec "));
				Serial.println("");
			}
			currentWSprocess = PROGRAM_FINISHED;			
			break;
		case PROGRAM_FINISHED:

			break;
		default:
			break;
		}
		break;
		
	
	case DRILLING:
		
		switch (currentWSprocess) // Start working
		{
		case RESET:
			
			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				GEN[1] = busy;
				if(busy)
					Serial.println(F("BUSY"));				
				else
					Serial.println(F("FREE"));
				//Serial.println(GEN[0]);
				
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				
				//Serial.println(F("Injumpfordep drill"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;
		case LOOK_DOWLOADING_PARAM:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
			break;
		case START_PROCESS:
			Serial.println("");
			Serial.println(F("Simulating DRILLING process"));
			delay(5000);
			Serial.println(F("End simulation"));
			currentWSprocess = END_PROCESS;
			break;
		case END_PROCESS:
		 
			reportBackStation1["spindVel"] = 3000;
			reportBackStation1["Diam"] = 8;
			reportBackStation1["HeadVel"] = 50;
			currentWSprocess = UPLOADING_PARAM;
			currentNFCstate = READY_FOR_INDATAEXCHANGE;
			break;
		case UPLOADING_PARAM:
		{
			uint8_t lenJ = reportBackStation1.measureLength();
			memset(dataOut, 0, sizeof(dataOut));
		reportBackStation1.printTo(dataOut, sizeof(dataOut));
		//reportBackStation1.printTo(Serial);
		if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
			nfc.InDataExchange(0x01, lenJ,(uint8_t *)dataOut, &cbInitiatorRx);
			//Serial.println(F("Indataexchange called!"));
			currentNFCstate = SENDED_MESSAGE_TO_TARGET;
		}
		}			
			break;
		default:
			break;				
		}		
		break;
	case GLUING:
		
		switch (currentWSprocess) // Start working
		{
		case RESET:
			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				GEN[1] = busy;
				if (busy)
					Serial.println(F("BUSY"));
				else
					Serial.println(F("FREE"));
				//Serial.println(GEN[0]);
				
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				
				
				//Serial.println(F("Injumpfordep gluin"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;
		case LOOK_DOWLOADING_PARAM:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
			break;
		case START_PROCESS:
			Serial.println("");
			Serial.println(F("Simulating GLUING process"));
			delay(5000);
			Serial.println(F("End simulation"));
			currentWSprocess = END_PROCESS;
			break;
		case END_PROCESS:
			reportBackStation2["TypeGlu"] = 2;
			reportBackStation2["Diam"] = 3;
			reportBackStation2["HeadVel"] = 61;
			currentWSprocess = UPLOADING_PARAM;
			currentNFCstate = READY_FOR_INDATAEXCHANGE;
			break;
		case UPLOADING_PARAM:
		{
			uint8_t lenJ = reportBackStation2.measureLength();
			memset(dataOut, 0, sizeof(dataOut));
			reportBackStation2.printTo(dataOut, sizeof(dataOut));
			//reportBackStation2.printTo(Serial);
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, lenJ, (uint8_t *)dataOut, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
		}
		break;
		default:
			break;
		}
		break;
	case SPRAYING:
		
		switch (currentWSprocess) // Start working
		{
		case RESET:
			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				GEN[1] = busy;
				if (busy)
					Serial.println(F("BUSY"));
				else
					Serial.println(F("FREE"));
				//Serial.println(GEN[0]);
				
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				
				//Serial.println(F("Injumpfordep spray"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;
		case LOOK_DOWLOADING_PARAM:
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
			break;
		case START_PROCESS:
			Serial.println("");
			Serial.println(F("Simulating SPRAYING process"));
			delay(5000);
			Serial.println(F("End simulation"));
			currentWSprocess = END_PROCESS;
			break;
		case END_PROCESS:
			
			reportBackStation3["TypeCol"] = 141;
			reportBackStation3["HeadVel"] = 55;
			currentWSprocess = UPLOADING_PARAM;
			currentNFCstate = READY_FOR_INDATAEXCHANGE;
			break;
		case UPLOADING_PARAM:
		{
			uint8_t lenJ = reportBackStation3.measureLength();
			memset(dataOut, 0, sizeof(dataOut));
			reportBackStation3.printTo(dataOut, sizeof(dataOut));
			//reportBackStation3.printTo(Serial);
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, lenJ, (uint8_t *)dataOut, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
		}
		break;
		default:
			break;
		}
		break;
	case ENGRAVING:
		
		switch (currentWSprocess) // Start working
		{
		case RESET:
			if (currentNFCstate == READY_FOR_FIRMLOOP) {
				nfc.GetFirmwareVersion(&cbFirmwareVersionLoop);
				currentNFCstate = FIRMLOOP;
			}
			if (currentNFCstate == READY_FOR_SAM) {
				nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfigLoop);
				currentNFCstate = SAMLOOP;
			}
			break;
		case LOOK_FOR_IWC:
			if (currentNFCstate == READY_FOR_LOOK_FOR_TARGET) {
				GEN[1] = busy;
				if (busy)
					Serial.println(F("BUSY"));
				else
					Serial.println(F("FREE"));
				//Serial.println(GEN[0]);
				
				nfc.InJumpForDEP(0x01, 0x02, 0x02, 0, NULL, 5, ATR_REQ, 2, GEN, &cbInitAsInitiator);
				
				//Serial.println(F("Injumpfordep engr"));
				currentNFCstate = LOOK_FOR_TARGET;
			}
			break;
		case LOOK_DOWLOADING_PARAM:

			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, sizeof(yes_msg), yes_msg, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;


			}
			break;
		case START_PROCESS:
			Serial.println("");
			Serial.println(F("Simulating ENGRAVING process"));
			delay(5000);
			Serial.println(F("End simulation"));
			currentWSprocess = END_PROCESS;
			break;
		case END_PROCESS:
			reportBackStation4["spindVel"] = 7000;
			reportBackStation4["Diam"] = 8;
			reportBackStation4["HeadVel"] = 50; 
			
			currentWSprocess = UPLOADING_PARAM;
			currentNFCstate = READY_FOR_INDATAEXCHANGE;
			break;
		case UPLOADING_PARAM:
		{

			uint8_t lenJ = reportBackStation4.measureLength();
			memset(dataOut, 0, sizeof(dataOut));
			reportBackStation4.printTo(dataOut, sizeof(dataOut));

			//reportBackStation4.printTo(Serial);
			if (currentNFCstate == READY_FOR_INDATAEXCHANGE) {
				nfc.InDataExchange(0x01, lenJ, (uint8_t *)dataOut, &cbInitiatorRx);
				//Serial.println(F("Indataexchange called!"));
				currentNFCstate = SENDED_MESSAGE_TO_TARGET;
			}
		}
		break;
		default:
			break;
		}
		break;
	default:
		break;
	}



	
	nfc.update();
	
}


