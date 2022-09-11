#ifndef _ROBOT_LEON_H_
#define _ROBOT_LEON_H_


/* C structures */
#ifndef __ASSEMBLER__

/* FIXME : TODO */

#endif /* __ASSEMBLER__ */

/* robot base_addr */
#define ROBOT_BASE_ADDR      0x80008000

/* robot registers */

/* reset & timer */
#define R_ROBOT_TIMER        0x00
#define A_ROBOT_TIMER        0x0x80008000

#define R_ROBOT_RESET        0x01
#define A_ROBOT_RESET        0x80008004

#define R_ROBOT_DEBUG        0x02
#define A_ROBOT_DEBUG        0x80008008


/* i2c slave interface */
#define R_ROBOT_I2C_TRACE_CS 0x0c
#define A_ROBOT_I2C_TRACE_CS 0x80008030

#define R_ROBOT_I2C_TRACE_D  0x0d
#define A_ROBOT_I2C_TRACE_D  0x80008034

#define R_ROBOT_I2C_BSTR_CS  0x0e
#define A_ROBOT_I2C_BSTR_CS  0x80008038

#define R_ROBOT_I2C_BSTR_D   0x0f
#define A_ROBOT_I2C_BSTR_D   0x8000803c


/* main motors */
#define R_ROBOT_MOTOR_1      0x40
#define A_ROBOT_MOTOR_1      0x80008100

#define R_ROBOT_MOTOR_2      0x48
#define A_ROBOT_MOTOR_2      0x80008120


/* odometry */
#define R_ROBOT_RC_VAL_1     0x81
#define A_ROBOT_RC_VAL_1     0x80008204

#define R_ROBOT_RC_ODO_1_INC 0x83
#define A_ROBOT_RC_ODO_1_INC 0x8000820c

#define R_ROBOT_RC_SPEED_1   0x84
#define A_ROBOT_RC_SPEED_1   0x80008210

#define R_ROBOT_RC_SAMPLING  0x87
#define A_ROBOT_RC_SAMPLING  0x8000821c

#define R_ROBOT_RC_VAL_2     0x89
#define A_ROBOT_RC_VAL_2     0x80008224

#define R_ROBOT_RC_ODO_2_INC 0x8b
#define A_ROBOT_RC_ODO_2_INC 0x8000822c

#define R_ROBOT_RC_SPEED_2   0x8c
#define A_ROBOT_RC_SPEED_2   0x80008230

/* ADC */
#define R_ROBOT_ADC_CHAN_0   0xe0
#define A_ROBOT_ADC_CHAN_0   0x80008380
#define R_ROBOT_ADC_CHAN_1   0xe1
#define A_ROBOT_ADC_CHAN_1   0x80008384
#define R_ROBOT_ADC_CHAN_2   0xe2
#define A_ROBOT_ADC_CHAN_2   0x80008388
#define R_ROBOT_ADC_CHAN_3   0xe3
#define A_ROBOT_ADC_CHAN_3   0x8000838c
#define R_ROBOT_ADC_CHAN_4   0xe4
#define A_ROBOT_ADC_CHAN_4   0x80008390
#define R_ROBOT_ADC_CHAN_5   0xe5
#define A_ROBOT_ADC_CHAN_5   0x80008394
#define R_ROBOT_ADC_CHAN_6   0xe6
#define A_ROBOT_ADC_CHAN_6   0x80008398
#define R_ROBOT_ADC_CHAN_7   0xe7
#define A_ROBOT_ADC_CHAN_7   0x8000839c


#endif /* _ROBOT_LEON_H_ */
