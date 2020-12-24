#ifndef _ASSERV_H_
#define _ASSERV_H_

#include "types.h"

#define GA_FLAG_MASK              0x000000ff
#define GA_FLAG_HOMING_DONE       0x00000001
#define GA_FLAG_AT_HOME           0x00000002
#define GA_FLAG_AT_HOME_PREV      0x00000004

#define GA_ERROR_MASK             0x0000ff00
#define GA_ERROR_BLOCK            0x00000100

#define GA_STATE_MASK             0xff000000
#define GA_STATE_DISABLED         0x00000000
#define GA_STATE_HOMING           0x01000000
#define GA_STATE_HOLD_POS         0x02000000
#define GA_STATE_GOTO_POS         0x03000000
#define GA_STATE_ERROR            0xff000000

#define GA_CMD_SET_ENABLE         0x10000000
#define GA_CMD_SET_KP             0x20000000
#define GA_CMD_SET_KI             0x30000000
#define GA_CMD_SET_KD             0x40000000
#define GA_CMD_DO_HOMING          0x50000000
#define GA_CMD_JUMP_TARGET        0x60000000
#define GA_CMD_GOTO_TARGET        0x70000000
#define GA_CMD_SET_RANGE_CLAMP    0x80000000
#define GA_CMD_SET_BLTRIG_SPEED   0x90000000
#define GA_CMD_DEBUG5             0xa0000000
#define GA_CMD_DEBUG4             0xb0000000
#define GA_CMD_DEBUG3             0xc0000000
#define GA_CMD_DEBUG2             0xd0000000
#define GA_CMD_DEBUG1             0xe0000000
#define GA_CMD_RESET_ERROR        0xf0000000

/*
  NOTE robot 2020:
   left  register stack @ : 0x80008500
   right register stack @ : 0x80008510
*/

typedef struct _goldo_asserv {
  uint32_t flags;

  int conf_max_range;
  int conf_pwm_clamp;
  int conf_goto_speed;
  int conf_Kp;
  int conf_Ki;
  int conf_Kd;
  int conf_block_trig;

  int st_homing_abs_pos;
  int st_abs_target_final;
  int st_abs_target;
  int st_abs_pos;
  int st_speed_est;
  int st_asserv_old_err;
  int st_asserv_delta_err;
  int st_asserv_err;
  int st_asserv_sigma_err;
  int st_asserv_output;
  int st_block_cnt;
  int st_debug_ts;

  uint32_t mot_reg;
  uint32_t enc_reg;
  uint32_t cmd_reg;
  uint32_t sta_reg;
  uint32_t pos_reg;
  uint32_t dbg_reg;
  uint32_t sw_reg;
  uint32_t sw_mask;
} goldo_asserv_t;

void init_asserv (struct _goldo_asserv *_ga, uint32_t _mot_reg, uint32_t _enc_reg, uint32_t _cmd_reg, uint32_t _sta_reg, uint32_t _pos_reg, uint32_t _dbg_reg, uint32_t _sw_reg, uint32_t _sw_mask);
int asserv_active (struct _goldo_asserv *_ga);
void enable_asserv (struct _goldo_asserv *_ga, int _enabled);
void set_Kp (struct _goldo_asserv *_ga, int _new_Kp);
void set_Ki (struct _goldo_asserv *_ga, int _new_Ki);
void set_Kd (struct _goldo_asserv *_ga, int _new_Kd);
int get_abs_pos (struct _goldo_asserv *_ga);
int get_rel_pos (struct _goldo_asserv *_ga);
void process_asserv_cmd (struct _goldo_asserv *_ga);
void do_step_asserv (struct _goldo_asserv *_ga);
int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id);
int jump_to_rel_target (struct _goldo_asserv *_ga, int _target);
int go_to_rel_target (struct _goldo_asserv *_ga, int _target);
void start_homing (struct _goldo_asserv *_ga);
void reset_error (struct _goldo_asserv *_ga);

#endif /* ASSERV_H_ */
