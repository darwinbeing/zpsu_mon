#ifndef PSUCTRL_EVENT_H
#define PSUCTRL_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

struct psuctrl_data_event {
        char volts[8];
        char amps[8];
        char watts[8];
        char energy[8];
        int is_kWh;
};

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
