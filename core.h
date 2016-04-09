#ifndef _CORE
#define _CORE
#include "utils.h"
#include "match.h"

bool extractFeatures(char *type);

void performMatching(char *type);

void surfaceFitting(char *type);

void getIdentityMatches();

bool printConfigFile();
#endif // _CORE

