/**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**80**/

#ifndef APPLEHAND_ERRORS_H_
#define APPLEHAND_ERRORS_H_

// The size of the error message array - it must be larger than arrays below!
const uint8_t ERROR_MESSAGE_LENGTH = 16;

static void resetError(char message[ERROR_MESSAGE_LENGTH]) {
  
  message[0] =  'N';
  message[1] =  'o';
  message[2] =  ' ';
  message[3] =  'e';
  message[4] =  'r';
  message[5] =  'r';
  message[6] =  'o';
  message[7] =  'r';
  message[8] =  '.';
  message[9] =  '\0';
}

static void createIMUError(uint8_t num_finger,
                           char message[ERROR_MESSAGE_LENGTH]) {
  message[0] =  'F';
  message[1] =  '0' + num_finger;
  message[2] =  ' ';
  message[3] =  'I';
  message[4] =  'M';
  message[5] =  'U';
  message[6] =  ' ';
  message[7] =  'e';
  message[8] =  'r';
  message[9] =  'r';
  message[10] = 'o';
  message[11] = 'r';
  message[12] = '.';
  message[13] = '\0';
}

static void createPressureError(uint8_t num_finger,
                                uint8_t num_pressure,
                                char message[ERROR_MESSAGE_LENGTH]) {
  message[0] =  'F';
  message[1] =  '0' + num_finger;
  message[2] =  ' ';
  message[3] =  'P';
  message[4] =  '0' + num_pressure;
  message[5] =  ' ';
  message[6] =  'e';
  message[7] =  'r';
  message[8] =  'r';
  message[9] =  'o';
  message[10] = 'r';
  message[11] = '.';
  message[12] = '\0';
}

static void createMotorError(uint8_t num_motor,
                             char message[ERROR_MESSAGE_LENGTH]) {
  message[0] =  'F';
  message[1] =  '0' + num_motor;
  message[2] =  ' ';
  message[3] =  'm';
  message[4] =  'o';
  message[5] =  't';
  message[6] =  'o';
  message[7] =  'r';
  message[8] =  ' ';
  message[9] =  'e';
  message[10] = 'r';
  message[11] = 'r';
  message[12] = 'o';
  message[13] = 'r';
  message[14] = '.';
  message[15] = '\0';
}

#endif
