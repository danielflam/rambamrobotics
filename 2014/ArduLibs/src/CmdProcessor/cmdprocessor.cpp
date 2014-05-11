
#line 1 "cmdprocessor.rl"
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

//#define SERIALCOMMAND_DEBUG
//////////////////////////////////
//
//   MACHINE DATA
//
//
///////////////////////////////////


#line 67 "cmdprocessor.rl"



#line 2 "cmdprocessor.cpp"
static const int foo_start = 1;
static const int foo_first_final = 11;
static const int foo_error = 0;

static const int foo_en_main = 1;


#line 70 "cmdprocessor.rl"

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


#line 8 "cmdprocessor.cpp"
	{
	cs = foo_start;
	}

#line 296 "cmdprocessor.rl"


#line 11 "cmdprocessor.cpp"
	{
	if ( p == pe )
		goto _test_eof;
	switch ( cs )
	{
st1:
	if ( ++p == pe )
		goto _test_eof1;
case 1:
	switch( (*p) ) {
		case 9: goto st1;
		case 32: goto st1;
	}
	if ( (*p) > 90 ) {
		if ( 97 <= (*p) && (*p) <= 122 )
			goto tr2;
	} else if ( (*p) >= 65 )
		goto tr2;
	goto st0;
tr6:
#line 46 "cmdprocessor.rl"
	{
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
	goto st0;
#line 44 "cmdprocessor.cpp"
st0:
cs = 0;
	goto _out;
tr2:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st2;
st2:
	if ( ++p == pe )
		goto _test_eof2;
case 2:
#line 56 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto tr3;
		case 9: goto tr4;
		case 10: goto tr3;
		case 13: goto tr3;
		case 32: goto tr4;
	}
	if ( (*p) < 65 ) {
		if ( 48 <= (*p) && (*p) <= 57 )
			goto st2;
	} else if ( (*p) > 90 ) {
		if ( 97 <= (*p) && (*p) <= 122 )
			goto st2;
	} else
		goto st2;
	goto st0;
tr3:
#line 37 "cmdprocessor.rl"
	{
		onCmdEnd(marker, p);
	}
	goto st11;
tr16:
#line 33 "cmdprocessor.rl"
	{
		processNumberend(marker, p);
	}
	goto st11;
st11:
	if ( ++p == pe )
		goto _test_eof11;
case 11:
#line 86 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto st11;
		case 10: goto st11;
		case 13: goto st11;
		case 59: goto tr23;
	}
	goto tr6;
tr23:
#line 41 "cmdprocessor.rl"
	{
		//execute cmd
		onCmdlineComplete(marker, p);
	}
	goto st3;
st3:
	if ( ++p == pe )
		goto _test_eof3;
case 3:
#line 103 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 9: goto st3;
		case 32: goto st3;
	}
	if ( (*p) > 90 ) {
		if ( 97 <= (*p) && (*p) <= 122 )
			goto tr8;
	} else if ( (*p) >= 65 )
		goto tr8;
	goto tr6;
tr8:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st4;
st4:
	if ( ++p == pe )
		goto _test_eof4;
case 4:
#line 122 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto tr3;
		case 9: goto tr9;
		case 10: goto tr3;
		case 13: goto tr3;
		case 32: goto tr9;
	}
	if ( (*p) < 65 ) {
		if ( 48 <= (*p) && (*p) <= 57 )
			goto st4;
	} else if ( (*p) > 90 ) {
		if ( 97 <= (*p) && (*p) <= 122 )
			goto st4;
	} else
		goto st4;
	goto tr6;
tr9:
#line 37 "cmdprocessor.rl"
	{
		onCmdEnd(marker, p);
	}
	goto st5;
tr17:
#line 33 "cmdprocessor.rl"
	{
		processNumberend(marker, p);
	}
	goto st5;
st5:
	if ( ++p == pe )
		goto _test_eof5;
case 5:
#line 152 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto st11;
		case 9: goto st5;
		case 10: goto st11;
		case 13: goto st11;
		case 32: goto st5;
		case 43: goto tr13;
		case 45: goto tr13;
	}
	if ( 48 <= (*p) && (*p) <= 57 )
		goto tr14;
	goto tr6;
tr13:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st6;
st6:
	if ( ++p == pe )
		goto _test_eof6;
case 6:
#line 173 "cmdprocessor.cpp"
	if ( 48 <= (*p) && (*p) <= 57 )
		goto st7;
	goto tr6;
tr14:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st7;
st7:
	if ( ++p == pe )
		goto _test_eof7;
case 7:
#line 185 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto tr16;
		case 9: goto tr17;
		case 10: goto tr16;
		case 13: goto tr16;
		case 32: goto tr17;
	}
	if ( 48 <= (*p) && (*p) <= 57 )
		goto st7;
	goto tr6;
tr4:
#line 37 "cmdprocessor.rl"
	{
		onCmdEnd(marker, p);
	}
	goto st8;
tr22:
#line 33 "cmdprocessor.rl"
	{
		processNumberend(marker, p);
	}
	goto st8;
st8:
	if ( ++p == pe )
		goto _test_eof8;
case 8:
#line 209 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto st11;
		case 9: goto st8;
		case 10: goto st11;
		case 13: goto st11;
		case 32: goto st8;
		case 43: goto tr19;
		case 45: goto tr19;
	}
	if ( 48 <= (*p) && (*p) <= 57 )
		goto tr20;
	goto st0;
tr19:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st9;
st9:
	if ( ++p == pe )
		goto _test_eof9;
case 9:
#line 230 "cmdprocessor.cpp"
	if ( 48 <= (*p) && (*p) <= 57 )
		goto st10;
	goto st0;
tr20:
#line 29 "cmdprocessor.rl"
	{
		marker = p;
	}
	goto st10;
st10:
	if ( ++p == pe )
		goto _test_eof10;
case 10:
#line 242 "cmdprocessor.cpp"
	switch( (*p) ) {
		case 0: goto tr16;
		case 9: goto tr22;
		case 10: goto tr16;
		case 13: goto tr16;
		case 32: goto tr22;
	}
	if ( 48 <= (*p) && (*p) <= 57 )
		goto st10;
	goto st0;
	}
	_test_eof1: cs = 1; goto _test_eof; 
	_test_eof2: cs = 2; goto _test_eof; 
	_test_eof11: cs = 11; goto _test_eof; 
	_test_eof3: cs = 3; goto _test_eof; 
	_test_eof4: cs = 4; goto _test_eof; 
	_test_eof5: cs = 5; goto _test_eof; 
	_test_eof6: cs = 6; goto _test_eof; 
	_test_eof7: cs = 7; goto _test_eof; 
	_test_eof8: cs = 8; goto _test_eof; 
	_test_eof9: cs = 9; goto _test_eof; 
	_test_eof10: cs = 10; goto _test_eof; 

	_test_eof: {}
	if ( p == eof )
	{
	switch ( cs ) {
	case 11: 
#line 41 "cmdprocessor.rl"
	{
		//execute cmd
		onCmdlineComplete(marker, p);
	}
	break;
	case 3: 
	case 4: 
	case 5: 
	case 6: 
	case 7: 
#line 46 "cmdprocessor.rl"
	{
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
	break;
#line 293 "cmdprocessor.cpp"
	}
	}

	_out: {}
	}

#line 298 "cmdprocessor.rl"
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
