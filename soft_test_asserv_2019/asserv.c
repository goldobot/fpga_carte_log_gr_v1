//-*-C++-*-

#include "uart.h"
#include "leds.h"
#include "sleep.h"
#include "leon.h"

#include "robot_leon.h"

#include "asserv.h"

void init_asserv (struct _goldo_asserv *_ga, uint32_t _mot_reg, uint32_t _enc_reg, uint32_t _sw_reg, uint32_t _sw_mask)
{
  _ga->flags = 0;
  _ga->homing_pos = 0;
  _ga->asserv_target = 0;
  _ga->asserv_pos = 0;
  _ga->asserv_old_err = 0;
  _ga->asserv_delta_err = 0;
  _ga->asserv_err = 0;
  _ga->asserv_sigma_err = 0;
  _ga->asserv_command = 0;
  _ga->Kp = 0x00008000;
  _ga->Ki = 0x00002000;
  _ga->Kd = 0x00000000;
  _ga->mot_reg = _mot_reg;
  _ga->enc_reg = _enc_reg;
  _ga->sw_reg = _sw_reg;
  _ga->sw_mask = _sw_mask;
}


void enable_asserv (struct _goldo_asserv *_ga, int _enabled)
{
  /* FIXME : TODO */
}


void do_step_asserv (struct _goldo_asserv *_ga)
{
  /* FIXME : TODO */
}


int do_test_asserv (struct _goldo_asserv *_ga, int _Ip, int _Ii, int _Id)
{
  int test_Ip;
  int test_Ii;
  int test_Id;
  int test_O;

  /* FIXME : TODO */

  return test_O;
}



