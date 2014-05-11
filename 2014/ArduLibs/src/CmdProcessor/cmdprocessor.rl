/**
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "cmdprocessor.h"

#define SERIALCOMMAND_DEBUG
//////////////////////////////////
//
//   MACHINE DATA
//
//
///////////////////////////////////

%%{
	machine foo;

	action mark {
		marker = p;
	}

	action numberend {
		processNumberend(marker, p);
	}

	action cmdend {
		onCmdEnd(marker, p);
	}

	action oncmdline {
		//execute cmd
		onCmdlineComplete(marker, p);
	}

	action onerror {
		//
		//alert ('error');
#ifdef SERIALCOMMAND_DEBUG
		  Serial.println("Parse error");   // Echo back to serial stream
#endif

//		if (defaultHandler)
//		{
//			defaultHandler(buffer);
//		}
	}

	plusminus = '+' | '-';
	INT = (plusminus? digit+) >mark %numberend;
	cmd = (alpha alnum*) >mark %cmdend;
	ws = [\t ];
	eoln = [\r\n\0];
	cmdline = ws* cmd (ws+ INT)* ws* eoln+ %oncmdline;

	main := cmdline (';' ws* cmdline)* $!onerror;
}%%

%% write data;

//////////////////////////////////
//
//   SerialCommand
//
//
///////////////////////////////////

//#define SERIALCOMMAND_DEBUG

/**
 * Constructor makes sure some things are set.
 */
SerialCommand::SerialCommand(SerialCommandCallback * commandList_, char term_) :
		commandList(commandList_), commandCount(0), defaultHandler(NULL), term(
				term_),    // default terminator for commands, newline character
		last(NULL) {
	strcpy(delim, " "); // strtok_r needs a null-terminated string
	clearBuffer();
	currentCommand = 0;
	nparam = 0;
}

void SerialCommand::processNumberend(char * marker, char* current) {
#ifdef SERIALCOMMAND_DEBUG
	Serial.print("Process number: ");   // Echo back to serial stream
#endif

	if (nparam > 9) {
#ifdef SERIALCOMMAND_DEBUG
		Serial.println("maximum number of params reached (10)"); // Echo back to serial stream
#endif

		return;
	}

	int n = 0;
	bool negative = false;

	for (char * ptr = marker; ptr != current; ptr++) {
#ifdef SERIALCOMMAND_DEBUG
		Serial.print(*ptr);
#endif

		if (*ptr == '-') {
			negative = true;
		} else if (*ptr >= '0' && *ptr <= '9') {
			n *= 10;
			n += *ptr - '0';
		}
	}
	if (negative) {
		n *= -1;
	}

	param[nparam] = n;

#ifdef SERIALCOMMAND_DEBUG
	Serial.print(" ");
	Serial.print(nparam);
	Serial.print(" ");
	Serial.print(n);   // Echo back to serial stream
	Serial.print(" ");
	Serial.print(param[nparam]);
	Serial.println();   // Echo back to serial stream
#endif

	nparam++;
}

void SerialCommand::onCmdEnd(char * marker, char* current) {
#ifdef SERIALCOMMAND_DEBUG
	Serial.print("End cmd:'");   // Echo back to serial stream
#endif

	int count = 0;
	for (char * ptr = marker; ptr != current; ptr++) {
#ifdef SERIALCOMMAND_DEBUG
		Serial.print(*ptr);   // Echo back to serial stream
#endif

		count++;
	}

#ifdef SERIALCOMMAND_DEBUG
	Serial.println("'");
#endif

	currentCommand = 0;

	for (SerialCommandCallback * list = commandList;
			!currentCommand && list->command != NULL; list++) {
#ifdef SERIALCOMMAND_DEBUG
		Serial.print("Comparing [");
		Serial.print(list->command);
		//           Serial.print("] to [");
//            Serial.print(commandList[i].command);
		Serial.println("]");
#endif

		// Compare the found command against the list of known commands for a match
		// store the function for the command
		if (strncmp(marker, list->command, count) == 0) {
			currentCommand = list->function;

#ifdef SERIALCOMMAND_DEBUG
			Serial.print("Matched Command: ");
			Serial.println(list->command);
#endif
		}
	}
#ifdef SERIALCOMMAND_DEBUG
//	if (!currentCommand)
//		Serial.println("Could not match command");
#endif
}

void SerialCommand::onCmdlineComplete(char * marker, char* current) {
#ifdef SERIALCOMMAND_DEBUG
		Serial.println("*******");
		Serial.println("Executing command.");
#endif

	if (currentCommand) {
		currentCommand();
	} else {
#ifdef SERIALCOMMAND_DEBUG
		Serial.println("Executing Default command.");
#endif
		defaultHandler(buffer);
	}
	currentCommand = 0;
	nparam = 0;
}

/**
 * Adds a "command" and a handler function to the list of available commands.
 * This is used for matching a found token in the buffer, and gives the pointer
 * to the handler function to deal with it.
 */
//void SerialCommand::addCommand(const char *command, void (*function)()) {
//  #ifdef SERIALCOMMAND_DEBUG
//    Serial.print("Adding command (");
//    Serial.print(commandCount);
//    Serial.print("): ");
//    Serial.println(command);
//  #endif
//
//  commandList = (SerialCommandCallback *) realloc(commandList, (commandCount + 1) * sizeof(SerialCommandCallback));
//  strncpy(commandList[commandCount].command, command, SERIALCOMMAND_MAXCOMMANDLENGTH);
//  commandList[commandCount].function = function;
//  commandCount++;
//}
/**
 * This sets up a handler to be called in the event that the receveived command string
 * isn't in the list of commands.
 */
void SerialCommand::setDefaultHandler(void (*function)(const char *)) {
	defaultHandler = function;
}

/**
 * This checks the Serial stream for characters, and assembles them into a buffer.
 * When the terminator character (default '\n') is seen, it starts parsing the
 * buffer for a prefix command, and calls handlers setup by addCommand() member
 */
void SerialCommand::update(char inChar) {
#ifdef SERIALCOMMAND_DEBUG
	Serial.print(inChar);   // Echo back to serial stream
#endif

	if (inChar == '\r' || inChar == '\n') { // Check for the terminator (default '\r') meaning end of command

#ifdef SERIALCOMMAND_DEBUG
		Serial.print("Received: ");
		Serial.println(buffer);
#endif

		parseBuffer();

		clearBuffer();
	} else if (isprint(inChar)) {   // Only printable characters into the buffer
		if (bufPos < SERIALCOMMAND_BUFFER) {
			buffer[bufPos++] = inChar;  // Put character into buffer
			buffer[bufPos] = '\0';      // Null terminate
		} else {
#ifdef SERIALCOMMAND_DEBUG
			Serial.println(
					"Line buffer is full - increase SERIALCOMMAND_BUFFER");
#endif
		}
	} else if (inChar == '\b' && bufPos > 0) {
		// backspace support
		bufPos--;
		buffer[bufPos] = '\0';
	}
}

/*
 * Clear the input buffer.
 */
void SerialCommand::clearBuffer() {
	buffer[0] = '\0';
	bufPos = 0;
}

/**
 * Retrieve the next token ("word" or "argument") from the command buffer.
 * Returns NULL if no more tokens exist.
 */
char *SerialCommand::next() {
//  return strtok_r(NULL, delim, &last);
	return NULL;
}

void SerialCommand::parseBuffer() {
	char *p = buffer;
	char *pe = p + bufPos + 1;
	char *eof = pe;
	char *marker = 0;
	int cs;

	currentCommand = 0;
	nparam = 0;

%% write init;

%% write exec;
}

/*
 int main( int argc, char **argv )
 {
 int cs, res = 0;
 if ( argc > 1 ) {
 char *p = argv[1];
 char *pe = p + strlen(p) + 1;


 printf("result = %i\n", res );
 return 0;
 }
 */
