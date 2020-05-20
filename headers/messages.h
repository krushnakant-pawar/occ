#ifndef MESSAGES_H
#define MESSAGES_H
typedef struct MSGFORMAT{
  int prio;     // Priority of message
  QString msg;      // Text message
  QString imgPath;  // Image path for message
}MsgFormat;
#endif // MESSAGES_H
