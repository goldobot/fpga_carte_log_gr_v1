#ifndef _ASSERV_H_
#define _ASSERV_H_

#ifndef uint32_t
typedef unsigned int uint32_t;
#endif
#ifndef uint16_t
typedef unsigned short int uint16_t;
#endif
#ifndef uint8_t
typedef unsigned char uint8_t;
#endif

#define GA_ACTIVE_MASK        0x00000001
#define GA_AT_HOME_MASK       0x00000002
#define GA_AT_HOME_PREV_MASK  0x00000004

typedef struct _goldo_asserv {
  uint32_t flags;
  int homing_pos;
  int asserv_target;
  int asserv_pos;
  int asserv_old_err;
  int asserv_delta_err;
  int asserv_err;
  int asserv_sigma_err;
  int asserv_command;
  int Kp;
  int Ki;
  int Kd;
  uint32_t mot_reg;
  uint32_t enc_reg;
  uint32_t sw_reg;
  uint32_t sw_mask;
} goldo_asserv_t;

void init_asserv (struct _goldo_asserv *_ga, uint32_t _mot_reg, uint32_t _enc_reg, uint32_t _sw_reg, uint32_t _sw_mask);
inline void set_Kp (struct _goldo_asserv *_ga, int _new_Kp) {_ga->Kp=_new_Kp;}
inline void set_Ki (struct _goldo_asserv *_ga, int _new_Ki) {_ga->Ki=_new_Ki;}
inline void set_Kd (struct _goldo_asserv *_ga, int _new_Kd) {_ga->Kd=_new_Kd;}
void enable_asserv (struct _goldo_asserv *_ga, int _enabled);
void do_step_asserv (struct _goldo_asserv *_ga);
int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id);

#endif /* ASSERV_H_ */
