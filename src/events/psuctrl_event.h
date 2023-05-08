#ifndef PSUCTRL_EVENT_H
#define PSUCTRL_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

struct psuctrl_data_event {
        float volts;
        float amps;
        float watts;
        float energy;
        int is_kWh;
};

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
