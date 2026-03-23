#ifndef TRS80CAS_H_INCLUDED
#define TRS80CAS_H_INCLUDED

#include "configs.h"

#ifdef Use_CAS

bool trs80cas_detect_and_init();
void trs80cas_process();

#endif // Use_CAS

#endif // TRS80CAS_H_INCLUDED
