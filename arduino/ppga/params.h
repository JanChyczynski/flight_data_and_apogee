#ifndef PARAMS
#define PARAMS

typedef float altitude_t;
typedef int my_time_t;

const int PAST_SAMPLE_COMPARED = 5;
const int NEG_STREAK_REQ = 3;
const altitude_t MIN_ALT = 5; //meter
const altitude_t MAX_ALT_DIFF_REQ = 0.2; //meter
const altitude_t MAX_ALT_DIFF_DEFINITE = 1.0; //meter
const int SAMPLES_IN_NOISE_FILTER = 5;

#endif
