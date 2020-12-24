//-*-C++-*-

#include "uart.h"
#include "leds.h"
#include "sleep.h"
#include "leon.h"

#include "robot_leon.h"

#include "asserv.h"

/* FIXME : DEBUG : ne pas supprimer! */
unsigned int big_bad_buf[4] = {0};

uint8_t __attribute__ ((aligned (4))) uart_byte;

unsigned int i2c_test_data;

uint32_t read_test_32b (uint32_t *the_addr)
{
  register volatile uint32_t my_val32;
  register volatile uint32_t my_result;

  my_val32 = *the_addr;
  my_result = my_val32;

  return my_result;
}

void write_test_32b (uint32_t *the_addr, uint32_t the_val)
{
  register uint32_t my_val32;

  my_val32 = (uint32_t) the_val;
  *the_addr = my_val32;
}


#define INPUT_BUF_SZ 16
char input_buf[INPUT_BUF_SZ] = "0123456789012345";
int ib_index = 0;

#define IS_IDDLE     0
#define IS_WAIT_CMD  1
#define IS_EDIT_BUF  2

void print_input_buf()
{
  int i;
  int test_val;

  uart_putchar ( 0x0d );
  for (i=0; i<INPUT_BUF_SZ; i++) {
    test_val=input_buf[i];
    if ((test_val>=0x20) && (test_val<0x80))
      uart_putchar ( input_buf[i] );
    else
      uart_putchar ( '.' );
  }
}

int convert_input_buf_to_int()
{
  int i;
  int test_val;
  int result_val = 0;
  int result_sign = 1;
  int blank_space = 1;

  for (i=0; i<INPUT_BUF_SZ; i++) {
    test_val=input_buf[i];
    if ((test_val==0x20) || (test_val==0x5f)) { /* ' ' or '_' */
      if (blank_space) continue; else break;
    } else if (test_val==0x2d) { /* '-' */
      if (!blank_space) break;
      result_sign = -1;
      blank_space = 0;
    } else if ((test_val>=0x30) && (test_val<=0x39)) {
      result_val = result_val*10 + (test_val-0x30);
    } else {
      break;
    }
  }

  result_val = result_sign * result_val;

  return result_val;
}

int convert_input_buf_to_hexint()
{
  int i;
  int test_val;
  int result_val = 0;
  int result_sign = 1;
  int blank_space = 1;

  for (i=0; i<INPUT_BUF_SZ; i++) {
    test_val=input_buf[i];
    if ((test_val==0x20) || (test_val==0x5f)) { /* ' ' or '_' */
      if (blank_space) continue; else break;
    } else if (test_val==0x2d) { /* '-' */
      if (!blank_space) break;
      result_sign = -1;
      blank_space = 0;
    } else if ((test_val>=0x30) && (test_val<=0x39)) {
      result_val = result_val*16 + (test_val-0x30);
    } else if ((test_val>=0x41) && (test_val<=0x46)) {
      result_val = result_val*16 + (test_val-0x41) + 10;
    } else if ((test_val>=0x61) && (test_val<=0x66)) {
      result_val = result_val*16 + (test_val-0x61) + 10;
    } else {
      break;
    }
  }

  result_val = result_sign * result_val;

  return result_val;
}

void edit_input_buf ()
{
  struct lregs *hw = ( struct lregs * )( PREGS );
  volatile int* leds_reg = ( volatile int* ) LEDS_BASE_ADDR;
  unsigned int my_uartstatus1;
  unsigned int my_uartdata1;
  unsigned int mask = 0xff;
  unsigned int leds = mask;
  uint32_t *my_p;
  int i;

  for (i=0;i<INPUT_BUF_SZ;i++) {
    input_buf[i] = '_';
  }

  ib_index=0;

  for (;;) {

    my_uartstatus1 = hw->uartstatus1;
    if ( my_uartstatus1 & UART_STATUS_DR ) {
      my_uartdata1 = hw->uartdata1;
      uart_byte = my_uartdata1;

      if ((uart_byte=='>') || (uart_byte==0x0a) || (uart_byte==0x0d)) {
        uart_putchar ( '>' );
        uart_putchar ( 0xa );
        return;
      } else {
        unsigned int word_shift, byte_shift;
        unsigned int actual_val, local_mask, uart_val;

        //input_buf[ib_index++] = uart_byte;

        uart_putchar ( uart_byte );
        uart_val = uart_byte;

        word_shift = (ib_index>>2)<<2;
        byte_shift = ib_index - word_shift;
        switch (byte_shift) {
        case 0:
          local_mask = 0x00ffffff;
          uart_val = uart_val<<24;
          break;
        case 1:
          local_mask = 0xff00ffff;
          uart_val = uart_val<<16;
          break;
        case 2:
          local_mask = 0xffff00ff;
          uart_val = uart_val<<8;
          break;
        case 3:
          local_mask = 0xffffff00;
          uart_val = uart_val<<0;
          break;
        default:
          local_mask = 0x00ffffff;
          uart_val = uart_val<<24;
        } /* switch (byte_shift) */

        my_p = ((uint32_t *)((char *)input_buf+word_shift));
        actual_val = *my_p;
        *my_p = (actual_val&local_mask) | uart_val;

        ib_index++;
        if (ib_index >= INPUT_BUF_SZ) {
          uart_putchar ( '>' );
          uart_putchar ( 0xa );
          return;
        }

      } /* if ((uart_byte=='>') || (uart_byte==0x0a) || (uart_byte==0x0d)) */

    } /* for (;;) */

    sleep ( 50 );

    asm ( "nop" );
    *leds_reg = leds & 0xff;
    asm ( "nop" );
    leds ^= mask;
    asm ( "nop" );

  }

  uart_putchar ( '!' );
  uart_putchar ( 0xa );
  return;
}

#define ROBOT_SAMPLING_INT  10000 /* in microseconds */

struct _goldo_asserv ga_left;
struct _goldo_asserv ga_right;

void reset_asserv ()
{
  init_asserv (&ga_left, 
               /* _mot_reg */ 0x125, 
               /* _enc_reg */ 0x81,
               /* _cmd_reg */ 0x140, 
               /* _sta_reg */ 0x141, 
               /* _pos_reg */ 0x142, 
               /* _dbg_reg */ 0x143, 
               /* _sw_reg  */ 0x139, 
               /* _sw_mask */ 0x00008000);
  init_asserv (&ga_right, 
               /* _mot_reg */ 0x127, 
               /* _enc_reg */ 0x89,
               /* _cmd_reg */ 0x144, 
               /* _sta_reg */ 0x145, 
               /* _pos_reg */ 0x146, 
               /* _dbg_reg */ 0x147, 
               /* _sw_reg  */ 0x139, 
               /* _sw_mask */ 0x00008000);
}

int main ()
{
    struct lregs *hw = ( struct lregs * )( PREGS );
    volatile int* leds_reg = ( volatile int* ) LEDS_BASE_ADDR;
    unsigned int mask = 0xff;
    unsigned int leds = mask;
    uint32_t loop_cnt=0;
    volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
    uint32_t robot_timer_val=0;
    uint32_t robot_timer_val_ms=0;
    uint32_t robot_sync_barrier=0;

    uint32_t my_val32;
    uint32_t mem_test_addr;
    uint32_t mem_test_data;

    i2c_test_data = 0;

    uart_init ( B115200 );


    uart_putchar ( 0xa );
    uart_putstring ( "Robot GOLDO - soft asserv 2020 - RELEASE" );
    uart_putchar ( 0xa );

    uart_putchar ( 0xa );
    loop_cnt=0;

    /* robot reset */
    robot_reg[R_ROBOT_RESET] = 1;
    robot_reg[R_ROBOT_RESET] = 0;

    reset_asserv();

    for (;;) {
      robot_timer_val = robot_reg[R_ROBOT_TIMER];
      robot_sync_barrier = robot_timer_val + ROBOT_SAMPLING_INT;

      asm ( "nop" );
      *leds_reg = leds & 0xff;
      asm ( "nop" );

      unsigned int my_uartstatus1 = hw->uartstatus1;
      if ( my_uartstatus1 & UART_STATUS_DR ) {
        unsigned int my_uartdata1 = hw->uartdata1;

        uart_byte = my_uartdata1;

        if (uart_byte=='@') {
          uart_putstring ( "@ : " );
          edit_input_buf ();
          mem_test_addr = convert_input_buf_to_hexint();
          uart_putchar ( 0xa );
        }

        if (uart_byte=='$') {
          uart_putstring ( "$ : " );
          edit_input_buf ();
          mem_test_data = convert_input_buf_to_hexint();
          uart_putchar ( 0xa );
        }

        if (uart_byte=='R') {
          my_val32 = read_test_32b((uint32_t *) mem_test_addr);
          uart_putstring ( "@0x" );
          uart_printhex ( mem_test_addr );
          uart_putstring ( " : 0x" );
          uart_printhex ( my_val32 );
          uart_putchar ( 0xa );
        }

        if (uart_byte=='W') {
          write_test_32b((uint32_t *) mem_test_addr, mem_test_data);
          uart_putstring ( "0x" );
          uart_printhex ( mem_test_data );
          uart_putstring ( "=> @0x" );
          uart_printhex ( mem_test_addr );
          uart_putchar ( 0xa );
        }

        if ((uart_byte=='%')) { /* robot reset */
          uart_putstring ( "RESET" );
          uart_putchar ( 0xa );
          robot_reg[R_ROBOT_RESET] = 1;
          robot_reg[R_ROBOT_RESET] = 0;

          /* ROBOT 2020 */
          reset_asserv();

          robot_timer_val = robot_reg[R_ROBOT_TIMER];
          robot_sync_barrier = robot_timer_val + ROBOT_SAMPLING_INT;
        }


        /*************************/
        /*** ASSERV 2020 DEBUG ***/
        /*************************/
        {
          if (uart_byte=='?') {
            uart_putstring ( "target : " );
            uart_putchar ( 0xa );
            uart_putstring ( "a : " );
            my_val32 = ga_left.st_abs_target;
            uart_printint ( my_val32 );
            uart_putchar ( 0xa );
            uart_putstring ( "r : " );
            my_val32 = ga_left.st_abs_target - ga_left.st_homing_abs_pos;
            uart_printint ( my_val32 );
            uart_putchar ( 0xa );
            uart_putstring ( "pos : " );
            uart_putchar ( 0xa );
            uart_putstring ( "a : " );
            my_val32 = get_abs_pos (&ga_left);
            uart_printint ( my_val32 );
            uart_putchar ( 0xa );
            uart_putstring ( "r : " );
            my_val32 = get_rel_pos (&ga_left);
            uart_printint ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = robot_reg[ga_left.sw_reg];
            uart_putstring ( "gpio: 0x" );
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.flags;
            uart_putstring ( "flags: 0x" );
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );

            uart_putstring ( "PID:" );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_Kp;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_Ki;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_Kd;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );

            uart_putstring ( "DEBUG:" );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_max_range;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_pwm_clamp;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_goto_speed;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
            my_val32 = ga_left.conf_block_trig;
            uart_printhex ( my_val32 );
            uart_putchar ( 0xa );
          }

        } /*** ASSERV 2020 DEBUG ***/

      } /* if ( my_uartstatus1 & UART_STATUS_DR ) */

      process_asserv_cmd(&ga_left);
      process_asserv_cmd(&ga_right);

      do_step_asserv (&ga_left);
      do_step_asserv (&ga_right);

      robot_timer_val = robot_reg[R_ROBOT_TIMER];
      robot_timer_val_ms = robot_timer_val/1000;

      asm ( "nop" );
      leds ^= mask;
      asm ( "nop" );

      loop_cnt++;

      do {
        robot_timer_val = robot_reg[R_ROBOT_TIMER];
      } while (robot_timer_val < robot_sync_barrier);
    }

    return 0;
}
