//-*-C++-*-

#include "uart.h"
#include "leds.h"
#include "sleep.h"
#include "leon.h"

#include "robot_leon.h"

#include "asserv.h"

int abs(int v) {return (v>=0)?v:-v;}
int asserv_state_is (struct _goldo_asserv *_ga, uint32_t _state);
void asserv_state_set (struct _goldo_asserv *_ga, uint32_t _state);


void init_asserv (struct _goldo_asserv *_ga, uint32_t _mot_reg, uint32_t _enc_reg,
                  uint32_t _cmd_reg, uint32_t _sta_reg, uint32_t _pos_reg, uint32_t _dbg_reg,
                  uint32_t _sw_reg, uint32_t _sw_mask)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  _ga->flags = 0; /*GA_STATE_DISABLED*/

  _ga->conf_max_range = 0x2000;
  _ga->conf_pwm_clamp = 0x1c0;
  _ga->conf_goto_speed = 40;
  _ga->conf_Kp = 0x00030000;
  _ga->conf_Ki = 0x00000400;
  _ga->conf_Kd = 0x00030000;
  _ga->conf_block_trig = 80;

  _ga->st_homing_abs_pos = 0;
  _ga->st_abs_target = 0;
  _ga->st_abs_target_final = 0;
  _ga->st_abs_pos = 0;
  _ga->st_speed_est = 0;
  _ga->st_asserv_old_err = 0;
  _ga->st_asserv_delta_err = 0;
  _ga->st_asserv_err = 0;
  _ga->st_asserv_sigma_err = 0;
  _ga->st_asserv_output = 0;
  _ga->st_block_cnt = 0;
  _ga->st_debug_ts = 0;

  _ga->mot_reg = _mot_reg;
  _ga->enc_reg = _enc_reg;
  _ga->cmd_reg = _cmd_reg;
  _ga->sta_reg = _sta_reg;
  _ga->pos_reg = _pos_reg;
  _ga->dbg_reg = _dbg_reg;
  _ga->sw_reg = _sw_reg;
  _ga->sw_mask = _sw_mask;

  robot_reg[_ga->mot_reg] = 0;
}


int asserv_active (struct _goldo_asserv *_ga) 
{
  uint32_t state;
  state = _ga->flags & GA_STATE_MASK;

  switch (state) {
  case GA_STATE_DISABLED:
  case GA_STATE_HOMING:
    return 0;
  case GA_STATE_HOLD_POS:
  case GA_STATE_GOTO_POS:
  case GA_STATE_ERROR:
    return 1;
  }
  return 0;
}


int asserv_state_is (struct _goldo_asserv *_ga, uint32_t _state) 
{
  return ((_ga->flags & GA_STATE_MASK)==_state);
}


void enable_asserv (struct _goldo_asserv *_ga, int _enabled)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  robot_reg[_ga->mot_reg] = 0;
  if (_enabled==1) 
  {
    _ga->st_abs_target = _ga->st_abs_pos;
    _ga->st_abs_target_final = _ga->st_abs_pos;
    asserv_state_set (_ga, GA_STATE_HOLD_POS);
  }
  else
  {
    asserv_state_set (_ga, GA_STATE_DISABLED);
  }
}


void asserv_state_set (struct _goldo_asserv *_ga, uint32_t _state) 
{
  _ga->flags = (_ga->flags & (~GA_STATE_MASK)) | (_state & GA_STATE_MASK);
}


void set_Kp (struct _goldo_asserv *_ga, int _new_Kp) 
{
  _ga->conf_Kp=_new_Kp;
}


void set_Ki (struct _goldo_asserv *_ga, int _new_Ki) 
{
  _ga->conf_Ki=_new_Ki;
}


void set_Kd (struct _goldo_asserv *_ga, int _new_Kd) 
{
  _ga->conf_Kd=_new_Kd;
}


void reset_error (struct _goldo_asserv *_ga)
{
  _ga->flags = _ga->flags & (~GA_ERROR_BLOCK);
}


int get_abs_pos (struct _goldo_asserv *_ga) 
{
  return _ga->st_abs_pos;
}


int get_rel_pos (struct _goldo_asserv *_ga) 
{
  return (_ga->st_abs_pos-_ga->st_homing_abs_pos);
}


void process_asserv_cmd (struct _goldo_asserv *_ga)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
  uint32_t cmd_reg = _ga->cmd_reg;
  uint32_t my_val32;
  uint32_t cmd_code;
  uint32_t cmd_param;
  int cmd_ok = 0;

  my_val32  = robot_reg[cmd_reg];
  cmd_code  = my_val32 & 0xf0000000;
  cmd_param = my_val32 & 0x0fffffff;
  if (cmd_param & 0x08000000) cmd_param |= 0xf0000000;

  if (cmd_code==0) return;

  switch (cmd_code) {
  case GA_CMD_SET_ENABLE:
    enable_asserv (_ga, cmd_param);
    cmd_ok = 1;
    break;
  case GA_CMD_SET_KP:
    set_Kp (_ga, cmd_param);
    cmd_ok = 1;
    break;
  case GA_CMD_SET_KI:
    set_Ki (_ga, cmd_param);
    cmd_ok = 1;
    break;
  case GA_CMD_SET_KD:
    set_Kd (_ga, cmd_param);
    cmd_ok = 1;
    break;
  case GA_CMD_DO_HOMING:
    start_homing (_ga);
    cmd_ok = 1;
    break;
  case GA_CMD_JUMP_TARGET:
    cmd_ok = jump_to_rel_target (_ga, cmd_param);
    break;
  case GA_CMD_GOTO_TARGET:
    cmd_ok = go_to_rel_target (_ga, cmd_param);
    break;
  case GA_CMD_SET_RANGE_CLAMP:
    {
      uint32_t new_max_range = (cmd_param & 0x0fff0000)>>16;
      uint32_t new_pwm_clamp = cmd_param & 0x0000ffff;
      if (new_max_range!=0) _ga->conf_max_range = new_max_range;
      if (new_pwm_clamp!=0) _ga->conf_pwm_clamp = new_pwm_clamp;
    }
    cmd_ok = 1;
    break;
  case GA_CMD_SET_BLTRIG_SPEED:
    {
      uint32_t new_block_trig = (cmd_param & 0x0fff0000)>>16;
      uint32_t new_goto_speed = cmd_param & 0x0000ffff;
      if (new_block_trig!=0) _ga->conf_block_trig = new_block_trig;
      if (new_goto_speed!=0) _ga->conf_goto_speed = new_goto_speed;
    }
    cmd_ok = 1;
    break;
  case GA_CMD_RESET_ERROR:
    reset_error (_ga);
    cmd_ok = 1;
    break;
  default:
    cmd_ok = 0;
    break;
  }

  if (cmd_ok)
    robot_reg[cmd_reg] = 0x00031337;
  else
    robot_reg[cmd_reg] = 0x0badcafe;
}


void do_step_asserv (struct _goldo_asserv *_ga)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
  uint32_t my_val32;
  uint32_t robot_timer_val=0;
  uint32_t robot_timer_val_ms=0;
  int tmp_int;
  int tmp_uint;
  int conf_pwm_clamp = _ga->conf_pwm_clamp;
  uint32_t mot_reg = _ga->mot_reg;
  uint32_t enc_reg = _ga->enc_reg;
  uint32_t sta_reg = _ga->sta_reg;
  uint32_t pos_reg = _ga->pos_reg;
  uint32_t dbg_reg = _ga->dbg_reg;
  uint32_t sw_reg  = _ga->sw_reg;
  uint32_t sw_mask = _ga->sw_mask;

  int old_abs_pos = _ga->st_abs_pos;

  /* lecture position et estimation de vitesse */
  {
    _ga->st_abs_pos = robot_reg[enc_reg];

    _ga->st_speed_est = _ga->st_abs_pos - old_abs_pos;
  }

  /* echelon de vitesse + detection de fin de traj (TODO : rampe de vitesse) */
  if (asserv_state_is (_ga, GA_STATE_GOTO_POS))
  {
    /* incrementation de la consigne de position */
    if (abs(_ga->st_abs_target-_ga->st_abs_target_final)<_ga->conf_goto_speed)
    {
      _ga->st_abs_target = _ga->st_abs_target_final;
    }
    else if (_ga->st_abs_target<_ga->st_abs_target_final)
    {
      _ga->st_abs_target += _ga->conf_goto_speed;
    }
    else /*if (_ga->st_abs_target>_ga->st_abs_target_final)*/
    {
      _ga->st_abs_target -= _ga->conf_goto_speed;
    }

    /* detection de la fin de l'action */
    if (abs(_ga->st_abs_pos-_ga->st_abs_target_final)<_ga->conf_goto_speed)
    {
      asserv_state_set (_ga, GA_STATE_HOLD_POS);
    }
  }

  /* application du PID */
  if (asserv_active(_ga)) 
  {
    _ga->st_asserv_err = _ga->st_abs_target - _ga->st_abs_pos;

    _ga->st_asserv_delta_err = _ga->st_asserv_err - _ga->st_asserv_old_err;

    _ga->st_asserv_sigma_err += _ga->st_asserv_err;
    if (_ga->st_asserv_sigma_err>16383) _ga->st_asserv_sigma_err=16383;
    if (_ga->st_asserv_sigma_err<-16383) _ga->st_asserv_sigma_err=-16383;

    _ga->st_asserv_output = 
      (_ga->conf_Kp*_ga->st_asserv_err +
       _ga->conf_Ki*_ga->st_asserv_sigma_err +
       _ga->conf_Kd*_ga->st_asserv_delta_err)>>16;
    if (_ga->st_asserv_output>conf_pwm_clamp) _ga->st_asserv_output=conf_pwm_clamp;
    if (_ga->st_asserv_output<-conf_pwm_clamp) _ga->st_asserv_output=-conf_pwm_clamp;

    robot_reg[mot_reg] = (unsigned int) _ga->st_asserv_output;

    _ga->st_asserv_old_err = _ga->st_asserv_err;
  } 
  else 
  {
    _ga->st_asserv_sigma_err = 0;
  }

  /* homing */
  if ((_ga->flags & GA_FLAG_AT_HOME)!=0)
    _ga->flags = _ga->flags | GA_FLAG_AT_HOME_PREV;
  else
    _ga->flags = _ga->flags & (~GA_FLAG_AT_HOME_PREV);

  my_val32 = robot_reg[sw_reg];
  if ((my_val32&sw_mask)!=0)
    _ga->flags = _ga->flags | GA_FLAG_AT_HOME;
  else
    _ga->flags = _ga->flags & (~GA_FLAG_AT_HOME);

  if (((_ga->flags & GA_FLAG_AT_HOME_PREV)==0) && ((_ga->flags & GA_FLAG_AT_HOME)!=0))
  {
    //uart_putchar ( '.' );
    //uart_putchar ( 0xa );
    if (asserv_state_is (_ga, GA_STATE_HOMING))
    {
      _ga->st_homing_abs_pos = _ga->st_abs_pos;
      _ga->flags = _ga->flags | GA_FLAG_HOMING_DONE;
      _ga->st_abs_target = _ga->st_abs_pos;
      _ga->st_abs_target_final = _ga->st_abs_target;
      asserv_state_set (_ga, GA_STATE_HOLD_POS);
      //uart_printhex ( ga->st_homing_abs_pos );
      //uart_putchar ( 0xa );
    }
    //robot_reg[mot_reg] = 0;
  }

  /* detection du blockage */
  if (asserv_state_is (_ga, GA_STATE_GOTO_POS))
  {
    if (abs(_ga->st_abs_target-_ga->st_abs_pos)>_ga->conf_goto_speed)
    {
      _ga->st_block_cnt++;
    }
    else
    {
      _ga->st_block_cnt = 0;
    }
  }

  if (_ga->st_block_cnt > _ga->conf_block_trig)
  {
    _ga->flags = _ga->flags | GA_ERROR_BLOCK;
    _ga->st_block_cnt = 0;
    _ga->st_abs_target_final = _ga->st_abs_pos;
    _ga->st_abs_target = _ga->st_abs_pos;
    asserv_state_set (_ga, GA_STATE_HOLD_POS);
  }

  /* m.a.j des registres d'interface avec le nouvel etat */
  robot_reg[sta_reg] = _ga->flags;
  tmp_int = _ga->st_abs_pos - _ga->st_homing_abs_pos;
  tmp_uint = tmp_int;
  robot_reg[pos_reg] = tmp_uint;
#if 1 /* FIXME : DEBUG */
  robot_timer_val = robot_reg[R_ROBOT_TIMER];
  robot_timer_val_ms = robot_timer_val/1000;
  robot_timer_val_ms -= _ga->st_debug_ts;
  robot_reg[dbg_reg] = (robot_timer_val_ms<<16) | (tmp_uint&0x0000ffff);
#endif
}


int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id)
{
  int test_O;

  test_O = (_Ip*_ga->conf_Kp + _Ii*_ga->conf_Ki + _Id*_ga->conf_Kd)>>16;

  return test_O;
}


int security_check_ok(struct _goldo_asserv *_ga, int _target)
{
  if (!asserv_active(_ga))
    return 0;

  if (!(_ga->flags & GA_FLAG_HOMING_DONE))
    return 0;

#if 0 /* FIXME : TODO : enable this? */
  if ((_ga->flags & GA_ERROR_MASK))
    return 0;
#endif

  if ((_target<0) || (_target>_ga->conf_max_range))
    return 0;

  return 1;
}


int jump_to_rel_target (struct _goldo_asserv *_ga, int _target)
{
  int new_abs_target = _target + _ga->st_homing_abs_pos;

  if (!security_check_ok(_ga, _target)) return 0;

  _ga->st_abs_target_final = new_abs_target;
  _ga->st_abs_target = _ga->st_abs_target_final;

  asserv_state_set (_ga, GA_STATE_HOLD_POS);

#if 1 /* FIXME : DEBUG */
  {
    volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
    _ga->st_debug_ts = robot_reg[R_ROBOT_TIMER]/1000;
  }
#endif

  return 1;
}


int go_to_rel_target (struct _goldo_asserv *_ga, int _target)
{
  int new_abs_target = _target + _ga->st_homing_abs_pos;

  if (!security_check_ok(_ga, _target)) return 0;

  _ga->st_abs_target_final = new_abs_target;
  _ga->st_abs_target = _ga->st_abs_pos;

  asserv_state_set (_ga, GA_STATE_GOTO_POS);

  return 1;
}


void start_homing (struct _goldo_asserv *_ga)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  asserv_state_set (_ga, GA_STATE_HOMING);
  robot_reg[_ga->mot_reg] = -_ga->conf_pwm_clamp/4;
}



