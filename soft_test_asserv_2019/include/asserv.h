#ifndef _ASSERV_H_
#define _ASSERV_H_

#include "types.h"

#define GA_ACTIVE_MASK        0x00000001
#define GA_AT_HOME_MASK       0x00000002
#define GA_AT_HOME_PREV_MASK  0x00000004
#define GA_HOMING_ACTION_MASK 0x00000008

#define GA_ENABLE(_ga) do {_ga->flags =_ga->flags|GA_ACTIVE_MASK;} while (0)
#define GA_DISABLE(_ga) do {_ga->flags =_ga->flags&(~GA_ACTIVE_MASK);} while (0)
#define GA_ENABLED(_ga) (_ga->flags&GA_ACTIVE_MASK)


typedef struct _goldo_asserv {
  uint32_t flags;
  int homing_abs_pos;
  int limit_abs_pos;
  int max_range;
  int pwm_clamp;
  int asserv_abs_target;
  int asserv_abs_pos;
  int asserv_speed_est;
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
void set_Kp (struct _goldo_asserv *_ga, int _new_Kp);
void set_Ki (struct _goldo_asserv *_ga, int _new_Ki);
void set_Kd (struct _goldo_asserv *_ga, int _new_Kd);
void enable_asserv (struct _goldo_asserv *_ga, int _enabled);
int asserv_is_enabled (struct _goldo_asserv *_ga);
int get_abs_pos (struct _goldo_asserv *_ga);
int get_rel_pos (struct _goldo_asserv *_ga);
void do_step_asserv (struct _goldo_asserv *_ga);
int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id);
void jump_to_rel_target (struct _goldo_asserv *_ga, int _target);
void go_to_rel_target (struct _goldo_asserv *_ga, int _target);
void start_homing (struct _goldo_asserv *_ga);

#endif /* ASSERV_H_ */
