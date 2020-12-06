//-*-C++-*-

#include "uart.h"
#include "leds.h"
#include "sleep.h"
#include "leon.h"

#include "robot_leon.h"

#include "asserv.h"

void init_asserv (struct _goldo_asserv *_ga, uint32_t _mot_reg, uint32_t _enc_reg, uint32_t _sw_reg, uint32_t _sw_mask)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  _ga->flags = 0;
  _ga->homing_abs_pos = 0;
  _ga->limit_abs_pos = 0x600;
  _ga->max_range = 0x600;
  _ga->pwm_clamp = 0x100;
  _ga->asserv_abs_target = 0;
  _ga->asserv_abs_pos = 0;
  _ga->asserv_speed_est = 0;
  _ga->asserv_old_err = 0;
  _ga->asserv_delta_err = 0;
  _ga->asserv_err = 0;
  _ga->asserv_sigma_err = 0;
  _ga->asserv_command = 0;
#if 1 /* FIXME : DEBUG : EXPERIMENTAL */
  _ga->Kp = 0x00010000;
  _ga->Ki = 0x00000200;
  _ga->Kd = 0x00010000;
#else
  _ga->Kp = 0x00008000;
  _ga->Ki = 0x00002000;
  _ga->Kd = 0x00000000;
#endif
  _ga->mot_reg = _mot_reg;
  _ga->enc_reg = _enc_reg;
  _ga->sw_reg = _sw_reg;
  _ga->sw_mask = _sw_mask;

  robot_reg[_ga->mot_reg] = 0;
}


void set_Kp (struct _goldo_asserv *_ga, int _new_Kp) 
{
  _ga->Kp=_new_Kp;
}


void set_Ki (struct _goldo_asserv *_ga, int _new_Ki) 
{
  _ga->Ki=_new_Ki;
}


void set_Kd (struct _goldo_asserv *_ga, int _new_Kd) 
{
  _ga->Kd=_new_Kd;
}


void enable_asserv (struct _goldo_asserv *_ga, int _enabled)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  robot_reg[_ga->mot_reg] = 0;
  if (_enabled==1) 
  {
    GA_ENABLE(_ga);
  }
  else
  {
    GA_DISABLE(_ga);
  }
}


int asserv_is_enabled (struct _goldo_asserv *_ga) 
{
  return GA_ENABLED(_ga);
}


int get_abs_pos (struct _goldo_asserv *_ga) 
{
  return _ga->asserv_abs_pos;
}


int get_rel_pos (struct _goldo_asserv *_ga) 
{
  return (_ga->asserv_abs_pos-_ga->homing_abs_pos);
}


void do_step_asserv (struct _goldo_asserv *_ga)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
  uint32_t my_val32;
  int pwm_clamp = _ga->pwm_clamp;
  uint32_t mot_reg = _ga->mot_reg;
  uint32_t enc_reg = _ga->enc_reg;
  uint32_t sw_reg  = _ga->sw_reg;
  uint32_t sw_mask = _ga->sw_mask;

  int old_abs_pos = _ga->asserv_abs_pos;

  {
    _ga->asserv_abs_pos = robot_reg[enc_reg];

    _ga->asserv_speed_est = _ga->asserv_abs_pos - old_abs_pos;
  }

  if (GA_ENABLED(_ga)) 
  {
    _ga->asserv_err = _ga->asserv_abs_target - _ga->asserv_abs_pos;

    _ga->asserv_delta_err = _ga->asserv_err - _ga->asserv_old_err;

    _ga->asserv_sigma_err += _ga->asserv_err;
    if (_ga->asserv_sigma_err>16383) _ga->asserv_sigma_err=16383;
    if (_ga->asserv_sigma_err<-16383) _ga->asserv_sigma_err=-16383;

#if 0 /* FIXME : DEBUG : EXPERIMENTAL */
    if (_ga->asserv_sigma_err>10) _ga->asserv_sigma_err-=9;
    if (_ga->asserv_sigma_err>1)  _ga->asserv_sigma_err-=1;
    if (_ga->asserv_sigma_err<10) _ga->asserv_sigma_err+=9;
    if (_ga->asserv_sigma_err<1)  _ga->asserv_sigma_err+=1;
#endif

    _ga->asserv_command = 
      (_ga->Kp*_ga->asserv_err + _ga->Ki*_ga->asserv_sigma_err + _ga->Kd*_ga->asserv_delta_err)>>16;
    if (_ga->asserv_command>pwm_clamp) _ga->asserv_command=pwm_clamp;
    if (_ga->asserv_command<-pwm_clamp) _ga->asserv_command=-pwm_clamp;

    robot_reg[mot_reg] = (unsigned int) _ga->asserv_command;

    _ga->asserv_old_err = _ga->asserv_err;
  } 
  else 
  {
    _ga->asserv_sigma_err = 0;
  }


  if ((_ga->flags&GA_AT_HOME_MASK)!=0)
    _ga->flags =_ga->flags|GA_AT_HOME_PREV_MASK;
  else
    _ga->flags =_ga->flags&(~GA_AT_HOME_PREV_MASK);

  my_val32 = robot_reg[sw_reg];
  if ((my_val32&sw_mask)!=0)
    _ga->flags =_ga->flags|GA_AT_HOME_MASK;
  else
    _ga->flags =_ga->flags&(~GA_AT_HOME_MASK);

  if (((_ga->flags&GA_AT_HOME_PREV_MASK)==0) && ((_ga->flags&GA_AT_HOME_MASK)!=0))
  {
    //uart_putchar ( '.' );
    //uart_putchar ( 0xa );
    GA_DISABLE(_ga);
    robot_reg[mot_reg] = 0;
    if ((_ga->flags&GA_HOMING_ACTION_MASK)!=0)
    {
      _ga->flags =_ga->flags&(~GA_HOMING_ACTION_MASK);
      _ga->homing_abs_pos = robot_reg[enc_reg];
      _ga->asserv_abs_target = _ga->homing_abs_pos;
      _ga->limit_abs_pos = _ga->homing_abs_pos + _ga->max_range;
      //uart_printhex ( ga->homing_abs_pos );
      //uart_putchar ( 0xa );
    }
  }

  if (GA_ENABLED(_ga)) 
  {
    if (_ga->asserv_abs_pos>_ga->limit_abs_pos)
    {
      GA_DISABLE(_ga);
      robot_reg[mot_reg] = 0;
    }
  }

}


int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id)
{
  int test_O;

  test_O = (_Ip*_ga->Kp + _Ii*_ga->Ki + _Id*_ga->Kd)>>16;

  return test_O;
}


void jump_to_rel_target (struct _goldo_asserv *_ga, int _target)
{
  if (!GA_ENABLED(_ga)) return;

  _ga->asserv_abs_target = _target + _ga->homing_abs_pos;
}


void go_to_rel_target (struct _goldo_asserv *_ga, int _target)
{
  if (!GA_ENABLED(_ga)) return;

  /* FIXME : TODO */
}


void start_homing (struct _goldo_asserv *_ga)
{
  volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;

  GA_DISABLE(_ga);
  _ga->flags =_ga->flags|GA_HOMING_ACTION_MASK;
  robot_reg[_ga->mot_reg] = 0xffffffc0;
}



