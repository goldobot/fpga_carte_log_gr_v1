//-*-C++-*-

#include "uart.h"
#include "leds.h"
#include "sleep.h"
#include "leon.h"

#include "robot_leon.h"

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

int main () {
    struct lregs *hw = ( struct lregs * )( PREGS );
    volatile int* leds_reg = ( volatile int* ) LEDS_BASE_ADDR;
    unsigned int mask = 0xff;
    unsigned int leds = mask;
    uint32_t loop_cnt=0;
    volatile uint32_t* robot_reg = ( volatile int* ) ROBOT_BASE_ADDR;
    uint32_t robot_timer_val=0;
    uint32_t robot_timer_val_ms=0;
    uint32_t robot_sync_barrier=0;
    int pwd_state;

    uint32_t my_val32;
    uint32_t mem_test_addr;
    uint32_t mem_test_data;

    /* ROBOT 2018 */
    unsigned int robot2018_rack_state = 0;
    unsigned int robot2018_rack_cmd = 0;
    unsigned int robot2018_stp_switch0 = 0;
    unsigned int robot2018_rack_limit_l = 17745;
    unsigned int robot2018_rack_limit_r = 15006;
    unsigned int robot2018_stp_cur_pos = 0x4000;
    unsigned int robot2018_stp_target_pos = 0x4000;
    int robot2018_rack_offset_l =  20; /* 0x00000020 test OK */
    int robot2018_rack_offset_c = -18; /* 0xffffff10 test OK */
    int robot2018_rack_offset_r = -75; /* 0xffffffb8 test OK */


    i2c_test_data = 0;

    uart_init ( B115200 );

#if 0 /* FIXME : DEBUG */
    leds = 0xaa;
    pwd_state = 0;
    for (;;) {
      asm ( "nop" );
      *leds_reg = leds & 0xff;
      asm ( "nop" );

      sleep ( 100 );

      asm ( "nop" );
      leds ^= mask;
      asm ( "nop" );

      unsigned int my_uartstatus1 = hw->uartstatus1;
      if ( my_uartstatus1 & UART_STATUS_DR ) {
        unsigned int my_uartdata1 = hw->uartdata1;
        uart_byte = my_uartdata1;
        switch (pwd_state) {
        case 0:
          if (uart_byte=='g') pwd_state = 1; else pwd_state = 0;
          break;
        case 1:
          if (uart_byte=='o') pwd_state = 2; else pwd_state = 0;
          break;
        case 2:
          if (uart_byte=='l') pwd_state = 3; else pwd_state = 0;
          break;
        case 3:
          if (uart_byte=='d') pwd_state = 4; else pwd_state = 0;
          break;
        case 4:
          if (uart_byte=='o') pwd_state = 5; else pwd_state = 0;
          break;
        }
      }

      if (pwd_state == 5) break;
    }
#endif

    uart_putchar ( 0xa );
    uart_putstring ( "Robot GOLDO - soft GR actionneurs 2018" );
    uart_putchar ( 0xa );
    uart_putstring ( "Fonctions OK :" );
    uart_putchar ( 0xa );
    uart_putstring ( "   @ : adresse de test AHB/APB" );
    uart_putchar ( 0xa );
    uart_putstring ( "   $ : data de test AHB/APB" );
    uart_putchar ( 0xa );
    uart_putstring ( "   R : test lecture 32b AHB/APB" );
    uart_putchar ( 0xa );
    uart_putstring ( "   W : test ecriture 32b AHB/APB" );
    uart_putchar ( 0xa );
    uart_putstring ( "   + : increment debug reg" );
    uart_putchar ( 0xa );
    uart_putstring ( "   - : decrement debug reg" );
    uart_putchar ( 0xa );
    uart_putstring ( "   ? : debug esclave i2c" );
    uart_putchar ( 0xa );
    uart_putstring ( "   w : ecrire ds trace i2c" );
    uart_putchar ( 0xa );
    uart_putstring ( "   r : lire ds bstr i2c" );
    uart_putchar ( 0xa );
    uart_putstring ( "   ! : charger nouveau soft" );
    uart_putchar ( 0xa );
    uart_putstring ( "   % : robot reset" );
    uart_putchar ( 0xa );

    uart_putchar ( 0xa );
    loop_cnt=0;

    /* robot reset */
    robot_reg[R_ROBOT_RESET] = 1;
    robot_reg[R_ROBOT_RESET] = 0;

    /* vitesse max pour le stepper */
    robot_reg[0x130] = 0x00080000;

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

        if ((uart_byte=='?') || (uart_byte=='w') || (uart_byte=='r')) {
          /* debug i2c (slave) */
          unsigned int i2c_val;
          uart_putstring ( "DEBUG I2C: " );
          uart_putchar ( 0xa );
          if ((uart_byte=='r')) {
            uart_putstring ( " bstr data: " );
            i2c_test_data = robot_reg[R_ROBOT_I2C_BSTR_D];
            uart_printhex ( i2c_test_data );
            uart_putchar ( 0xa );
          }
          if ((uart_byte=='w')) {
            // robot_reg[R_ROBOT_I2C_TRACE_D] = robot_timer_val;
            uart_putstring ( " write to trace: " );
            uart_printhex ( i2c_test_data );
            uart_putchar ( 0xa );
            robot_reg[R_ROBOT_I2C_TRACE_D] = i2c_test_data;
          }
          uart_putstring ( " trace status: " );
          i2c_val = robot_reg[R_ROBOT_I2C_TRACE_CS];
          uart_printhex ( i2c_val );
          uart_putchar ( 0xa );
          uart_putstring ( " trace data dbg: " );
          i2c_val = robot_reg[R_ROBOT_I2C_TRACE_D];
          uart_printhex ( i2c_val );
          uart_putchar ( 0xa );
          uart_putstring ( " bstr status: " );
          i2c_val = robot_reg[R_ROBOT_I2C_BSTR_CS];
          uart_printhex ( i2c_val );
          uart_putchar ( 0xa );
        }

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

        if (uart_byte=='+') {
          mem_test_addr = 0x80008008;
          my_val32 = read_test_32b((uint32_t *) mem_test_addr);
          my_val32++;
          write_test_32b((uint32_t *) mem_test_addr, my_val32);
          uart_putstring ( "0x" );
          uart_printhex ( my_val32 );
          uart_putchar ( 0xa );
        }

        if (uart_byte=='-') {
          mem_test_addr = 0x80008008;
          my_val32 = read_test_32b((uint32_t *) mem_test_addr);
          my_val32--;
          write_test_32b((uint32_t *) mem_test_addr, my_val32);
          uart_putstring ( "0x" );
          uart_printhex ( my_val32 );
          uart_putchar ( 0xa );
        }

        if ((uart_byte=='!')) { /* load new soft */
          uart_putchar ( '!' );
          uart_putchar ( 0xa );
          asm ( "call 0x10000000" ); /* call load_bitstream */
        }

        if ((uart_byte=='%')) { /* robot reset */
          uart_putstring ( "RESET" );
          uart_putchar ( 0xa );
          robot_reg[R_ROBOT_RESET] = 1;
          robot_reg[R_ROBOT_RESET] = 0;

          /* ROBOT 2018 */
          robot2018_rack_state = 0;
          robot2018_rack_cmd = 0;
          robot2018_stp_switch0 = 0;
          robot2018_rack_limit_l = 17745 - ( 20);
          robot2018_rack_limit_r = 15006 - (-75);
          robot2018_stp_cur_pos = 0x4000;
          robot2018_stp_target_pos = 0x4000;
          robot2018_rack_offset_l =  20;
          robot2018_rack_offset_c = -18;
          robot2018_rack_offset_r = -75;

          robot_timer_val = robot_reg[R_ROBOT_TIMER];
          robot_sync_barrier = robot_timer_val + ROBOT_SAMPLING_INT;
        }
      }

      robot2018_rack_cmd = robot_reg[0x13c];
      robot2018_stp_switch0 = robot_reg[0x138] & 0x00000001;

      if ( (robot2018_rack_cmd != 0) || (robot2018_rack_cmd != 0x42424242) ) {
        switch (robot2018_rack_cmd) {
        case 1:
          /* droite */
          robot_reg[0x131] = robot2018_rack_limit_r + robot2018_rack_offset_r;
          robot2018_rack_state = 1;
          robot_reg[0x13c] = 0x42424242;
          break;
        case 2:
          /* centre */
          robot_reg[0x131] = (robot2018_rack_limit_r+robot2018_rack_limit_l)/2 
            + robot2018_rack_offset_c;
          robot2018_rack_state = 2;
          robot_reg[0x13c] = 0x42424242;
          break;
        case 3:
          /* gauche */
          robot_reg[0x131] = robot2018_rack_limit_l + robot2018_rack_offset_l;
          robot2018_rack_state = 3;
          robot_reg[0x13c] = 0x42424242;
          break;
        case 11:
          /* recallage droite */
          robot_reg[0x131] = 0x3400;
          robot2018_rack_state = 11;
          robot_reg[0x13c] = 0x42424242;
          break;
        case 13:
          /* recallage gauche */
          robot_reg[0x131] = 0x4c00;
          robot2018_rack_state = 13;
          robot_reg[0x13c] = 0x42424242;
          break;
        case 21:
          /* forcer rack_limit droite */
          robot2018_rack_limit_r = robot_reg[0x13d];
          robot_reg[0x13c] = 0;
          break;
        case 23:
          /* forcer rack_limit gauche */
          robot2018_rack_limit_l = robot_reg[0x13d];
          robot_reg[0x13c] = 0;
          break;
        case 31:
          /* offset droite */
          robot2018_rack_offset_r = robot_reg[0x13d];
          robot_reg[0x13c] = 0;
          break;
        case 32:
          /* offset centre */
          robot2018_rack_offset_c = robot_reg[0x13d];
          robot_reg[0x13c] = 0;
          break;
        case 33:
          /* offset gauche */
          robot2018_rack_offset_l = robot_reg[0x13d];
          robot_reg[0x13c] = 0;
          break;
        case 42:
          uart_putstring ( "RESET" );
          uart_putchar ( 0xa );
          robot_reg[R_ROBOT_RESET] = 1;
          robot_reg[R_ROBOT_RESET] = 0;

          /* ROBOT 2018 */
          robot2018_rack_state = 0;
          robot2018_rack_cmd = 0;
          robot2018_stp_switch0 = 0;
          robot2018_rack_limit_l = 17745;
          robot2018_rack_limit_r = 15006;
          robot2018_stp_cur_pos = 0x4000;
          robot2018_stp_target_pos = 0x4000;
          robot2018_rack_offset_l =  20;
          robot2018_rack_offset_c = -18;
          robot2018_rack_offset_r = -75;

          robot_timer_val = robot_reg[R_ROBOT_TIMER];
          robot_sync_barrier = robot_timer_val + ROBOT_SAMPLING_INT;
          break;
        case 0xdeadbeef:
          /* arret d'urgence */
          /* goldo_fpga_cmd_servo(0, 0); */
          robot_reg[0x101] = 0;
          /* goldo_fpga_cmd_servo(1, 0); */
          robot_reg[0x103] = 0;
          /* goldo_fpga_cmd_servo(2, 0); */
          robot_reg[0x105] = 0;
          /* goldo_fpga_cmd_servo(3, 0); */
          robot_reg[0x107] = 0;
          /* goldo_fpga_cmd_servo(4, 0); */
          robot_reg[0x109] = 0;
          /* goldo_fpga_cmd_servo(5, 0); */
          robot_reg[0x10b] = 0;
          /* goldo_fpga_cmd_servo(6, 0); */
          robot_reg[0x10d] = 0;
          /* goldo_fpga_cmd_servo(7, 0); */
          robot_reg[0x10f] = 0;
          /* goldo_fpga_cmd_servo(8, 0); */
          robot_reg[0x111] = 0;
          /* goldo_fpga_cmd_servo(9, 0); */
          robot_reg[0x113] = 0;
          /* goldo_fpga_cmd_servo(10, 0); */
          robot_reg[0x115] = 0;
          /* goldo_fpga_cmd_servo(11, 0); */
          robot_reg[0x117] = 0;
          /* goldo_fpga_cmd_motor(0, 0); */
          robot_reg[0x121] = 0;
          /* goldo_fpga_cmd_motor(1, 0); */
          robot_reg[0x123] = 0;
          /* goldo_fpga_cmd_motor(2, 0); */
          robot_reg[0x125] = 0;
          /* goldo_fpga_cmd_stepper(0, 0xffffffff); */
          robot_reg[0x131] = 0xffffffff;
          robot_reg[0x13c] = 0;
          break;
        }
      }

      robot2018_stp_cur_pos = robot_reg[0x131]>>16;
      robot2018_stp_target_pos = robot_reg[0x131] & 0xffff;

      switch (robot2018_rack_state) {
      case 1:
        /* depl. droite */
      case 2:
        /* depl. centre */
      case 3:
        /* depl. gauche */
        if (robot2018_stp_cur_pos == robot2018_stp_target_pos) {
          robot2018_rack_state = 0;
          robot_reg[0x13c] = 0;
        }
        break;
      case 11:
        /* recal. droite */
        if ( robot2018_stp_switch0 != 0 ) {
          robot2018_rack_limit_r = robot2018_stp_cur_pos;
          robot_reg[0x131] = 0xffffffff;
          robot2018_rack_state = 0;
          robot_reg[0x13c] = 0;
        }
        break;
      case 13:
        /* recal. gauche */
        if ( robot2018_stp_switch0 != 0 ) {
          robot2018_rack_limit_l = robot2018_stp_cur_pos;
          robot_reg[0x131] = 0xffffffff;
          robot2018_rack_state = 0;
          robot_reg[0x13c] = 0;
        }
        break;
      }

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
