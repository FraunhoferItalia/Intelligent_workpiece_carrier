/*
 Name:		wpk.ino
 Created:	10/11/2016 11:57:24 AM
 Author:	wgasparetto
*/




#include <ArduinoJson.h>
#include <Arduino.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <BooleanParser.h>

/*
Define the NFC structures -START-
*/
#include <nfc_p2p.h>
#define PN532_CS 10
uint8_t ATR_REQ[] = { 0x00,0xFF,0xFF,0x00,0x00 };
uint8_t MIFARE[] = { 0x08, 0x00,0x12, 0x34, 0x56,0x40 };
uint8_t FELICA[] = { 0x01, 0xFE, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,0xFF, 0xFF };
uint8_t NFCID3t[] = { 0xAA, 0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11 };
uint8_t RFConfigData[] = { 0x00,0x0A,0x09 };
nfc_p2p nfc(PN532_CS);
uint8_t busy = LOW;
char dataOut[150];

enum NFC_state {
	READY_FOR_LOOK_FOR_INITIATOR,
	LOOK_FOR_INITIATOR,
	READY_FOR_GET_DATA,
	LOOK_FOR_DATA,
	READY_FOR_SEND_MSG_TO_INIT,
	SENDED_MESSAGE_TO_INITIATOR,
	RECEIVED_OK_FROM_SENDED,
	ACTIVE
};

enum NFC_state currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
/*
Define the NFC structures -END-
*/




BooleanParser pars; // Create an object for parsing some of the rules
LiquidCrystal lcd(5, 6, 7, 8, 4, 3);// This is used to initialize the library with respective pins. In my case, I am using pin 5 to pin 10 for respective pins. So it would be LiquidCrystal lcd(5,6,7,8,9,10).
StaticJsonBuffer<300> jsonBuffer;



/*
Define data structure for wpk - START
*/

#define NUMBER_OF_PROCESSES 4
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


uint8_t len_executed_process = 0;
uint8_t n_prerequisites[NUMBER_OF_PROCESSES];
uint8_t len_prerequisites = 0;

typedef enum {  
				START,
				LOOK_FOR_BASE_DOWNLOAD_STRUCTURE_DATA,
				LISTEN_FOR_BASE_PERMISSION_IN_DOWNLOADING,
				SEND_MSG_CONFIRMATION_TO_BASE_STATION_DOWNLOADING,
				VARIABLES_INITIALIZATION,
			    START_PROCESSES_LOOKAROUND,
				NFC_MOVE_TO_NEXT_STATION,
				MOVE_TO_NEXT_STATION,
				UPLOADING_PARAMETERS_PERMISSION,
				UPLOADING_PARAMETERS, 
				WAIT_TILL_END_PROCESS,
				SEND_PARAM_CONFIRMATION_RECEIVED,
				PROCESS_ENDED,
				FIND_NEW_TARGET_STATION,
				WAIT_TO_REPORT,
				LOOKING_FOR_BASE,
				UPLOADING_TO_BASE_STATION,
				SUCCESSFULLY_TERMINATED,
				BASE_FOUND_TO_WC,
				TRANSMIT_STRUCTURE_OF_VALUES,
				TRANSMIT_TIME_BETWEEN_STATIONS_WAIT,
				TRANSMIT_TIME_BETWEEN_STATIONS,
				TRANSMIT_EXECUTED_ARRAY_WAIT,
				TRANSMIT_EXECUTED_ARRAY,
				END,
				SLEEP,
				ERROR

} stateMachine;

stateMachine state = LOOK_FOR_BASE_DOWNLOAD_STRUCTURE_DATA;
uint8_t nextProcess = 0;
uint8_t lastProcess = 0;
uint8_t temp_occupied = 0; // Put the station that for now is occupied, eg next_available cant look at it in successors


typedef struct
{
	uint8_t ID;								// ID of the process
	uint8_t parameters_len;					// Effective len of working parameters
	uint32_t parameters[WPK_N_OF_PARAMETERS];	// Parameters of control
	unsigned long time;
	uint8_t numberOfOccupied;
}returnedValues;


typedef union {
	returnedValues valuesOfLines;
	char data_byte[sizeof(returnedValues)];
}unionReturnedValues;

uint8_t number_of_time_transferred = 0;
unionReturnedValues receivedParameters[NUMBER_OF_PROCESSES]; // struttura che ritorna i parametri


typedef union {
	unsigned long timeBTWStations[NUMBER_OF_PROCESSES + 1];
	char data_byte[];
}returnedTimes;

returnedTimes returnedTimeBTWstations;
uint8_t LenTimeBTWStations = 0;
unsigned long StartTime = millis();
unsigned long CurrentTime = millis();


typedef union {
	uint8_t executed[NUMBER_OF_PROCESSES];
	char data_byte[];
}returnedExe;

returnedExe returnedExecutedOrder;







/*
Define data structure for wpk END
*/





/*
Define random START
*/
#define RND_ANALOG_IN A1
long randNumber;
/*
Define random STOP
*/


/*
Define NFC Functions START
*/
void cbSamConfig(void) {
	//Serial.println(F("Sam config executed"));
}

void cbRF(void) {
	//Serial.println(F("RF config executed"));
}

void cbFirmwareVersion(uint8_t ic, uint8_t ver, uint8_t rev, uint8_t support) {

	Serial.print(F("Firmware version IC="));
	Serial.print(ic, HEX);
	Serial.println("");
	//Serial.print(F(", Ver="));
	//Serial.print(ver, HEX);
	//Serial.print(F(", Rev="));
	//Serial.print(rev, HEX);
	//Serial.print(F(", Support="));
	//Serial.println(support, HEX);
}


void cbTgInitialization(uint8_t mode, uint16_t len, uint8_t* InitiatorCommand) {
	//Serial.println(F("Target initialization executed"));
	//Serial.println(mode, HEX);
	//Serial.println(len, HEX);
	//Serial.println(F("Station"));
	
	switch (state)
	{
	case LOOK_FOR_BASE_DOWNLOAD_STRUCTURE_DATA:

		if (InitiatorCommand[8] == nextProcess) // Se ho trovato la stazione che mi interessa ed e libera
		{
			// La stazione e quella giusta
			//Serial.println(F("Found B"));
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.print(F("Found BASE"));
			delay(2000);
			state = LISTEN_FOR_BASE_PERMISSION_IN_DOWNLOADING;
			currentNFCstate = READY_FOR_GET_DATA;

		}
		else //se non e la base
		{
			//Serial.println(F("wrong sta"));

			// La stazione non e quella giusta, cerca di nuovo
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Wrong station"));
			delay(2000);
			currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
			state = LOOK_FOR_BASE_DOWNLOAD_STRUCTURE_DATA;
		}
		
		break;
	case LOOKING_FOR_BASE:

		if (InitiatorCommand[8] == nextProcess) // Se ho trovato la stazione che mi interessa ed e libera
		{
			// La stazione e quella giusta
			//Serial.println(F("Found B"));
			CurrentTime = millis();
			
			returnedTimeBTWstations.timeBTWStations[LenTimeBTWStations] = CurrentTime - StartTime;
			//LenTimeBTWStations++;
			
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.print(F("Found BASE"));
			delay(2000);
			state = BASE_FOUND_TO_WC;
			currentNFCstate = READY_FOR_GET_DATA;
			
		}
		else //se non e la base
		{
			//Serial.println(F("wrong sta"));
			
			// La stazione non e quella giusta, cerca di nuovo
			lcd.clear();
			lcd.setCursor(0, 0);
			lcd.print(F("Wrong station"));
			delay(2000);
			currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
			state = WAIT_TO_REPORT;
		}
			break;
		default:
			//Serial.println(InitiatorCommand[8], HEX);
			//Serial.println(InitiatorCommand[9], HEX);
			busy = InitiatorCommand[9];

			if (InitiatorCommand[8] == nextProcess && !InitiatorCommand[9]) // Se ho trovato la stazione che mi interessa ed e libera
			{
				// La stazione e quella giusta
				//Serial.println(F("Right !Busy stat"));
				currentNFCstate = READY_FOR_GET_DATA;
				state = MOVE_TO_NEXT_STATION;
			}
			else if (InitiatorCommand[8] == nextProcess && InitiatorCommand[9]) // stazione giusta ma occupata
			{
				//Serial.println(F("Right Busy stat"));
				busy = InitiatorCommand[9];
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(F("Busy stat"));
				delay(2000);
				state = MOVE_TO_NEXT_STATION;
				currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;

			}
			else //se non e la stazione giusta cerca di nuovo
			{
				//Serial.println(F("wrong stat"));
				currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
				state = START_PROCESSES_LOOKAROUND;
				// La stazione non e quella giusta, cerca di nuovo
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print(F("Wrong station"));
				delay(2000);
			}
			break;

	}

	

}


void cbTgRx(uint8_t statusCommand, uint16_t len, uint8_t* dataIn) {

	if (statusCommand != 0x00) {
		//Serial.println(F("Error cbTgRx"));
	}
	else
	{
		//Serial.println(F("Target rx"));

		/*for(uint8_t i =0;i<len;i++){
		Serial.println(dataIn[i],HEX);
		}*/
		 

		switch (state) {
		case LISTEN_FOR_BASE_PERMISSION_IN_DOWNLOADING:

			for (uint8_t i = 0; i<len; i++) {
				process[number_of_time_transferred].data_byte[i]= dataIn[i];	
			}
			number_of_time_transferred++;
			state = SEND_MSG_CONFIRMATION_TO_BASE_STATION_DOWNLOADING;
			currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
			break;
		case TRANSMIT_EXECUTED_ARRAY_WAIT:
			state = TRANSMIT_EXECUTED_ARRAY;
			currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
			break;
		case TRANSMIT_TIME_BETWEEN_STATIONS_WAIT:
			if (dataIn[0] == 0x89) {
				currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
				state = TRANSMIT_TIME_BETWEEN_STATIONS;
			}
			break;
		case BASE_FOUND_TO_WC:
			if (dataIn[0] == 0x89) {
				//Serial.println("Y received");
				currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
				state = UPLOADING_TO_BASE_STATION;
			}
			break;
		case UPLOADING_PARAMETERS_PERMISSION:
			//
			if (dataIn[0] == 0x89) // se ricevuto il permesso Y manda parametri json
			{

				currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
				state = UPLOADING_PARAMETERS ;
			}
			else {
				//Serial.println(F("Rec not Y"));
			}
			break;
		case WAIT_TILL_END_PROCESS:
			// scaricare e memorizzare i dati in memoria a seconda del tipo di stazione
		{

			JsonObject& dataFromWorkingStation = jsonBuffer.parseObject((char *)dataIn);

			if (!dataFromWorkingStation.success())
			{
				//Serial.println(F("parsefailed"));
				return;
			}
			//Serial.println(F("Dat from stat"));

			switch (nextProcess) {
			case 1:
			{
			
			receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters_len = 3;
			receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[0] = dataFromWorkingStation["spindVel"];
			receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[1] = dataFromWorkingStation["Diam"];
			receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[2] = dataFromWorkingStation["HeadVel"];
			}
				
				break;
			case 2:
			{			
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters_len = 3;
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[0] = dataFromWorkingStation["TypeGlu"];
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[1] = dataFromWorkingStation["Diam"];
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[2] = dataFromWorkingStation["HeadVel"];
			}
				break;
			case 3:
			{
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters_len = 2;
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[0] = dataFromWorkingStation["TypeCol"];
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[1] = dataFromWorkingStation["HeadVel"];
			}
				break;
			case 4:
			{
			
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters_len = 3;
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[0] = dataFromWorkingStation["spindVel"];
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[1] = dataFromWorkingStation["Diam"];
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.parameters[2] = dataFromWorkingStation["HeadVel"];

			}
				break;
			default:
				break;

			}

			
			//dataFromWorkingStation.printTo(Serial);
			state = SEND_PARAM_CONFIRMATION_RECEIVED;
			currentNFCstate = READY_FOR_SEND_MSG_TO_INIT;
		}	
			break;
		default:
			break;
		}
	}
}

void cbTgTx(uint8_t statusCommand) {

	if (statusCommand != 0x00) {
		//Serial.println(F("error cbTgTx"));
	}
	else
	{
		//Serial.println(F("Targtx ok"));
		

		switch (state) {
		case SEND_MSG_CONFIRMATION_TO_BASE_STATION_DOWNLOADING:
			if (number_of_time_transferred < NUMBER_OF_PROCESSES) {
				state = LISTEN_FOR_BASE_PERMISSION_IN_DOWNLOADING;
				currentNFCstate = READY_FOR_GET_DATA;

			}
			else {
				state = VARIABLES_INITIALIZATION;
				currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
				number_of_time_transferred = 0;
			}
			break;
		case TRANSMIT_EXECUTED_ARRAY:
			state = SUCCESSFULLY_TERMINATED;
			currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;

			break;
		case TRANSMIT_TIME_BETWEEN_STATIONS:
			state = TRANSMIT_EXECUTED_ARRAY_WAIT;
			currentNFCstate = READY_FOR_GET_DATA;

			break;
		case TRANSMIT_STRUCTURE_OF_VALUES:

			if (number_of_time_transferred < NUMBER_OF_PROCESSES) {
				currentNFCstate = READY_FOR_GET_DATA;
				state = BASE_FOUND_TO_WC;
			}
				
			else {
				state = TRANSMIT_TIME_BETWEEN_STATIONS_WAIT;
				currentNFCstate = READY_FOR_GET_DATA;
			}

			break;
		case WAIT_TILL_END_PROCESS :
			// HO mandato i paramentri json alla stazione
			currentNFCstate = READY_FOR_GET_DATA;
			
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.print(F("Wait end of "));
			lcd.setCursor(5, 1);
			lcd.print(process[nextProcess - 1].singleStation.name);
			delay(2000);
			break;
		case SEND_PARAM_CONFIRMATION_RECEIVED:
			state = PROCESS_ENDED;
			currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
			break;
		default:
			break;
		}

	}
}
/*
Define NFC Functions STOP
*/

void setup()
{
	//Serial.begin(115200);
	//while (!Serial);
	randomSeed(analogRead(RND_ANALOG_IN));
	nfc.GetFirmwareVersion(&cbFirmwareVersion);
	delay(10);
	nfc.update();
	delay(10);
	nfc.SAMConfiguration(0x01, 0x14, 0x01, &cbSamConfig);
	nfc.update();
	delay(10);
	nfc.RFConfiguration(0x02, RFConfigData, cbRF);
	nfc.update();
	delay(10);

	lcd.begin(16, 2);//lcd.begin(column, row) This is used to initialize the lcd that you are using. In my case, I’m using a 16×2 LCD, so include it in the void setup() as lcd.begin(16,2).

}


void firstInTheLine()
{
	for (uint8_t i = 0; i < NUMBER_OF_PROCESSES; i++)
	{
		if (process[i].singleStation.first == 1)
		{
			n_prerequisites[len_prerequisites] = process[i].singleStation.ID;
			len_prerequisites++;
		}

	}

	randNumber = random(0, len_prerequisites);
	nextProcess = n_prerequisites[randNumber];
	
}


uint8_t ismember(uint8_t that_process) {
	uint8_t response;
	for (uint8_t ii = 0; ii < len_executed_process+1; ii++) {
	
		if (returnedExecutedOrder.executed[ii] == that_process) {
			response = 1;
			break;
		}
		else response = 0;
	}
	return response;
}


int8_t next_available(void) {
	uint8_t result_bool,result_next;
	uint8_t valid_successor[NUMBER_OF_PROCESSES];
	uint8_t len_valid_successor = 0;
	uint8_t ism,ism2;
	// Last process is important, there stay the list of possible successors
	if (process[lastProcess - 1].singleStation.succ_len == 0 && (len_executed_process < NUMBER_OF_PROCESSES))
	{
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("ERROR in logic"));
		state = ERROR;
		return -1;
	}
	else {

		if (len_executed_process == NUMBER_OF_PROCESSES) { //We have finished
			return -2; //there are no processes available but we have finished
		}

		for (uint8_t ii = 0; ii < process[lastProcess - 1].singleStation.succ_len; ii++) {
			
			if (!ismember(process[lastProcess - 1].singleStation.succ[ii]) ) { // if is not member of the executed processes
				if (process[lastProcess - 1].singleStation.succ[ii] != temp_occupied) {
					
					result_bool = pars.ParseIt(process[process[lastProcess - 1].singleStation.succ[ii] - 1].singleStation.prerequ, returnedExecutedOrder.executed, len_executed_process);

					if (result_bool) // If prerequisites are verified
					{
						valid_successor[len_valid_successor] = process[lastProcess - 1].singleStation.succ[ii];
						len_valid_successor++;
					}
				}
			}
		}

		if (len_valid_successor != 0) {
			// Take a random process to continue if there are some
			
			
			randNumber = random(0, len_valid_successor);
			result_next = valid_successor[randNumber];
			return result_next;
		}
		else {
			// Probabily there are no processes available
			return 0;
		}
	}
}

void loop()
{
	switch (state) {

	case LOOK_FOR_BASE_DOWNLOAD_STRUCTURE_DATA: 
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(F("Intelligent Work"));
		lcd.setCursor(1, 1);
		lcd.print(F("Piece Carrier"));
		delay(4000);
		lcd.clear();
		lcd.setCursor(4, 0);
		lcd.print(F("Starting"));
		delay(2000);
		StartTime = millis();
		if (currentNFCstate == READY_FOR_LOOK_FOR_INITIATOR) {
			//Serial.println(F("InitAstarget"));
			nextProcess = 0;
			nfc.TgInitAsTarget(0x00, MIFARE, FELICA, NFCID3t, 1, &nextProcess, 0, NULL, &cbTgInitialization); // send station ID we are looking for
			currentNFCstate = LOOK_FOR_INITIATOR;

		}
		break;

	case LISTEN_FOR_BASE_PERMISSION_IN_DOWNLOADING:
		if (currentNFCstate == READY_FOR_GET_DATA) {
			//Serial.println(F("Wait for param"));
			nfc.TgGetData(&cbTgRx);
			currentNFCstate = LOOK_FOR_DATA;

		}
		break;
	case SEND_MSG_CONFIRMATION_TO_BASE_STATION_DOWNLOADING:
		if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {
			//Serial.println(F("Tring set data "));
			uint8_t yes_msg[] = { 0x89 };
			nfc.TgSetData(sizeof(yes_msg), yes_msg, &cbTgTx);
			currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;
		}

		break;
	case VARIABLES_INITIALIZATION:
		firstInTheLine();
		StartTime = millis(); // BTW station start time 
		state = START_PROCESSES_LOOKAROUND;
		break;

	case START_PROCESSES_LOOKAROUND:
		
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Go to "));
		lcd.print(nextProcess,HEX);
		lcd.setCursor(3, 1);
		lcd.print(process[nextProcess-1].singleStation.name);
		delay(5000);
		currentNFCstate = READY_FOR_LOOK_FOR_INITIATOR;
		state = NFC_MOVE_TO_NEXT_STATION;

		if (currentNFCstate == READY_FOR_LOOK_FOR_INITIATOR) {
			//Serial.println(F("InitAstarget"));
			nfc.TgInitAsTarget(0x00, MIFARE, FELICA, NFCID3t, 1, &nextProcess, 0, NULL, &cbTgInitialization); // send station ID we are looking for
			currentNFCstate = LOOK_FOR_INITIATOR;

		}
		//accendi NFC detection
		break;
	case NFC_MOVE_TO_NEXT_STATION:

		break;
	case MOVE_TO_NEXT_STATION:
		if(len_executed_process < NUMBER_OF_PROCESSES) 
		{ 
			//delay(2000);
			if (!busy) { // se e stata trovata la stazione, ed e quella giusta, e non e occupata
				CurrentTime = millis();
				returnedTimeBTWstations.timeBTWStations[LenTimeBTWStations] = CurrentTime - StartTime; // stop time between stations 
				LenTimeBTWStations++;
				
				receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.ID = process[nextProcess - 1].singleStation.ID;
				state = UPLOADING_PARAMETERS_PERMISSION;	

				}
				
			else {
				if (len_executed_process == 0) // se ancora nessuna stazione e stata visitata
				{
					
					receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.ID = process[nextProcess - 1].singleStation.ID;
					receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.numberOfOccupied++;
					temp_occupied = process[nextProcess - 1].singleStation.ID; // Se una delle prime occupata allora scegline un altra
					busy = LOW;
					lcd.clear();
					lcd.setCursor(1, 0);
					lcd.print(process[nextProcess - 1].singleStation.name);
					lcd.setCursor(0, 1);
					lcd.print(F("Not available"));
					//delay(2000);
					firstInTheLine();
					state = START_PROCESSES_LOOKAROUND;
				}
				else {
					// Trova alternative valide se e´ occupata
					lcd.clear();
					lcd.setCursor(1, 0);
					lcd.print(process[nextProcess - 1].singleStation.name);
					lcd.setCursor(0, 1);
					lcd.print(F("Not available"));
					delay(2000);
					temp_occupied = process[nextProcess - 1].singleStation.ID; // metti temporaneamente quella occupata 
					receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.ID = process[nextProcess - 1].singleStation.ID;
					receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.numberOfOccupied++;
					state = FIND_NEW_TARGET_STATION;
				}	
			}
		}
		else
		{
		// No more processes
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.println(F("No more processes"));
			delay(2000);
			state = WAIT_TO_REPORT;	

		}

		break;
	case UPLOADING_PARAMETERS_PERMISSION:
		if (currentNFCstate == READY_FOR_GET_DATA) {
			//Serial.println(F("Wait for Y"));
			nfc.TgGetData(&cbTgRx);
			currentNFCstate = LOOK_FOR_DATA;
		}
		break;

	case UPLOADING_PARAMETERS:
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Proceed with"));
		lcd.setCursor(0, 1);
		lcd.print(F("Uploading param"));
		delay(2000);
		StartTime = millis();// start time for the stations

		if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {
			JsonObject& root = jsonBuffer.createObject();

			//Serial.println(F("Tring set data "));

			switch (nextProcess)
			{
			case 0:
				break;
			case 1: // DRILLING
				root["spindVel"] = 3600;
				root["Diam"] = 8;
				root["HeadVel"] = 56;
				
				break;
			case 2: // GLUING
				root["TypeGlu"] = 2;
				root["Diam"] = 3;
				root["HeadVel"] = 60;
				break;
			case 3: // SPRAYING
				root["TypeCol"] = 141;
				root["HeadVel"] = 56;
				break;
			case 4: // ENGRAVING
				root["spindVel"] = 70;
				root["Diam"] = 2;
				root["HeadVel"] = 56;
				root["Word"] ="UNITN";
				break;
			}

			
			uint8_t len = root.measureLength();
			memset(dataOut, 0, sizeof(dataOut));
			root.printTo(dataOut, sizeof(dataOut));
			//root.printTo(Serial);
			//Serial.println(F(""));
			nfc.TgSetData(len, (uint8_t*)dataOut, &cbTgTx);
			currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;

		}
		
		state = WAIT_TILL_END_PROCESS;
		
		break;
		
	case WAIT_TILL_END_PROCESS:
		if (currentNFCstate == READY_FOR_GET_DATA) {
			//Serial.println(F("Wait for Parameters back"));
			nfc.TgGetData(&cbTgRx);
			currentNFCstate = LOOK_FOR_DATA;
		}
		
		break;
	case  SEND_PARAM_CONFIRMATION_RECEIVED:
		if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {

			//Serial.println(F("Tring set data "));
			uint8_t yes_msg[] = { 0x89 };
			nfc.TgSetData(sizeof(yes_msg), yes_msg, &cbTgTx);
			currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;
		}

		break;
	case PROCESS_ENDED:
		CurrentTime = millis();
		receivedParameters[process[nextProcess - 1].singleStation.ID - 1].valuesOfLines.time = CurrentTime - StartTime; // Fine tempo lavorazione stazione

		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Process  "));
		lcd.print(process[nextProcess - 1].singleStation.name);
		lcd.setCursor(5, 1);
		lcd.print(F("Ended"));
		delay(2000);

		if (len_executed_process < NUMBER_OF_PROCESSES) {
			returnedExecutedOrder.executed[len_executed_process++] = process[nextProcess - 1].singleStation.ID; // Added to executed
			lastProcess = returnedExecutedOrder.executed[len_executed_process - 1];
			state = FIND_NEW_TARGET_STATION;
			StartTime = millis(); // per il movimento tra le stazioni
			if (len_executed_process == NUMBER_OF_PROCESSES)
			{
				state = WAIT_TO_REPORT;
			}
		}
		else {
			StartTime = millis(); // per il movimento tra le stazioni
			state = WAIT_TO_REPORT;

		}
		break;
	case FIND_NEW_TARGET_STATION:
		
		nextProcess = next_available();

		//I have new nextProcess
		switch (nextProcess) {
		case 0: //problem manca un processo ma e occupato.
			lcd.clear();
			lcd.setCursor(1, 0);
			lcd.print(F("Wait, no process"));
			lcd.setCursor(0, 1);
			lcd.print(F("available "));
			delay(2000);
			temp_occupied = 0;
			busy = LOW;

			break;
		case -1:// errore imperdonabile nella logica
			state = ERROR;
			break;
		case -2: // processes finished
			state = WAIT_TO_REPORT;
			break;

		default: // sputa un nuovo processo
			// Move to next station?
			state = START_PROCESSES_LOOKAROUND;
			busy = LOW;
			temp_occupied = 0;
			break;
		// se non ci sono piu processi disponibili, devi aspettare per forza questo processo
		}
		break;

	case WAIT_TO_REPORT: // wait to the base station
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Processes END"));
		lcd.setCursor(0, 1);
		lcd.print(F("Waiting for BASE"));
		delay(2000);
		
		if (currentNFCstate == READY_FOR_LOOK_FOR_INITIATOR) {
			//Serial.println(F("InitAstarget"));
			nextProcess = 0;
			nfc.TgInitAsTarget(0x00, MIFARE, FELICA, NFCID3t, 1, &nextProcess, 0, NULL, &cbTgInitialization); // send station ID we are looking for
			currentNFCstate = LOOK_FOR_INITIATOR;

		}
		//accendi NFC detection
		state = LOOKING_FOR_BASE;
		break;

	case ERROR:
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("ERROR"));
			break;
	case LOOKING_FOR_BASE:

		break;
	case BASE_FOUND_TO_WC:
		if (currentNFCstate == READY_FOR_GET_DATA) {

			//Serial.println(F("Y?"));
			nfc.TgGetData(&cbTgRx);
			currentNFCstate = LOOK_FOR_DATA;
		}
		break;

	case UPLOADING_TO_BASE_STATION:
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Proceed with"));
		lcd.setCursor(0, 1);
		lcd.print(F("Uploading param"));
		delay(2000);

		state = TRANSMIT_STRUCTURE_OF_VALUES;
		break;

	case TRANSMIT_STRUCTURE_OF_VALUES:
		if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {
			//Serial.println(F("Transmstructvalu"));
			memset(dataOut, 0, sizeof(dataOut));
			for (uint8_t i = 0; i<sizeof(receivedParameters[0]); i++) {
				dataOut[i] = receivedParameters[number_of_time_transferred].data_byte[i]; // struttura che ritorna i parametri
				//Serial.println(receivedParameters[number_of_time_transferred].data_byte[i], HEX);
			}
			
			nfc.TgSetData(sizeof(receivedParameters[0]), (uint8_t*)dataOut, &cbTgTx);
			currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;
		number_of_time_transferred++;
		}

		break;
	case TRANSMIT_TIME_BETWEEN_STATIONS_WAIT:
		if (currentNFCstate == READY_FOR_GET_DATA) {
			//Serial.println(F("Y?"));
			nfc.TgGetData(&cbTgRx);
			currentNFCstate = LOOK_FOR_DATA;
		}
		break;
	case TRANSMIT_TIME_BETWEEN_STATIONS:
		if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {
			//Serial.println(F("Transmstime"));
			memset(dataOut, 0, sizeof(dataOut));
			for (uint8_t i = 0; i<sizeof(returnedTimes); i++) {
				dataOut[i] = returnedTimeBTWstations.data_byte[i]; // struttura che ritorna i parametri
			}
			nfc.TgSetData(sizeof(returnedTimes), (uint8_t*)dataOut, &cbTgTx);
			currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;
		}
		break;
		case TRANSMIT_EXECUTED_ARRAY_WAIT:
			if (currentNFCstate == READY_FOR_GET_DATA) {
				//Serial.println(F("Y?"));
				nfc.TgGetData(&cbTgRx);
				currentNFCstate = LOOK_FOR_DATA;
			}
			break;
		case TRANSMIT_EXECUTED_ARRAY:
			if (currentNFCstate == READY_FOR_SEND_MSG_TO_INIT) {
				//Serial.println(F("Transmsexe"));
				memset(dataOut, 0, sizeof(dataOut));
				for (uint8_t i = 0; i<sizeof(returnedExe); i++) {
					dataOut[i] = returnedExecutedOrder.data_byte[i]; // struttura che ritorna i parametri
					
				}
				nfc.TgSetData(sizeof(returnedExe), (uint8_t*)dataOut, &cbTgTx);
				currentNFCstate = SENDED_MESSAGE_TO_INITIATOR;
			}
			break;
	case SUCCESSFULLY_TERMINATED:
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Successfully"));
		lcd.setCursor(0, 1);
		lcd.print(F("Terminated"));
		delay(2000);
		state = SLEEP;
		break;

	case SLEEP:
		lcd.clear();
		lcd.setCursor(1, 0);
		lcd.print(F("Going to "));
		lcd.setCursor(0, 1);
		lcd.print(F("Sleep"));
		state = END;
		break;
	default:

		break;
	}

	nfc.update();
}
