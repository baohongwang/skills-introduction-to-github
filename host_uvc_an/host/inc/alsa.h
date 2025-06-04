#ifndef _ALSA_H
#define _ALSA_H

#include <alsa/asoundlib.h>
#include "datatype.h"
#include "common.h"

typedef void (*ff_reorder_func)(const void *, void *, int);

#define ALSA_BUFFER_SIZE_MAX 131072

#endif /* _ALSA_H */
