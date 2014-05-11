#ifndef _CMDPROCESSOR_H
#define _CMDPROCESSOR_H


#include <Arduino.h>

#include <string.h>


// Size of the input buffer in bytes (maximum length of one command plus arguments)
#define SERIALCOMMAND_BUFFER 64
// Maximum length of a command excluding the terminating null
#define SERIALCOMMAND_MAXCOMMANDLENGTH 8

// Uncomment the next line to run the library in debug mode (verbose messages)
//#define SERIALCOMMAND_DEBUG


struct SerialCommandCallback {
  const char * command;
  void (*function)();
};                                    // Data structure to hold Command/Handler function key-value pairs


class SerialCommand {
  public:
    SerialCommand(SerialCommandCallback * commandList_, char term_ = '\n');      // Constructor
//    void addCommand(const char *command, void(*function)());  // Add a command to the processing dictionary.
    void setDefaultHandler(void (*function)(const char *));   // A handler to call when no valid command received.

	void update(char inChar);

    void clearBuffer();   // Clears the input buffer.
    char *next();         // Returns pointer to next token found in command buffer (for getting arguments to commands).

	int getParam(int i){return param[i];}
	int getParamCount(){return nparam;}
  private:
    // Command/handler dictionary
    SerialCommandCallback *commandList;   // Actual definition for command/handler array
    byte commandCount;

	void processNumberend(char * marker, char* current);
	void onCmdEnd(char * marker, char* current);
	void onCmdlineComplete(char * marker, char* current);
	void parseBuffer();

    // Pointer to the default handler function
    void (*defaultHandler)(const char *);

    char delim[2]; // null-terminated list of character to be used as delimeters for tokenizing (default " ")
    char term;     // Character that signals end of command (default '\n')

    char buffer[SERIALCOMMAND_BUFFER + 1]; // Buffer of stored characters while waiting for terminator character
    byte bufPos;                        // Current position in the buffer
    char *last;                         // State variable used by strtok_r during processing
	void (*currentCommand)();
	int param[10];
	int nparam;
};


#endif
