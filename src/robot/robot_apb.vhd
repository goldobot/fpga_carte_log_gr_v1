library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned."+";
use IEEE.std_logic_unsigned."-";
use IEEE.std_logic_unsigned.conv_integer;
use IEEE.numeric_std.all;

use work.amba.all;
use work.iface.all;

use work.FPGA_VERSION.ALL;


entity robot_apb is
  port(
      pclk                : in  std_logic
    ; presetn             : in  std_logic
    ; clk_i_100           : in  std_logic
    -- APB slave
    ; paddr               : in  std_logic_vector(31 downto 0)
    ; psel                : in  std_logic
    ; penable             : in  std_logic
    ; pwrite              : in  std_logic
    ; pwdata              : in  std_logic_vector(31 downto 0)
    ; prdata              : out std_logic_vector(31 downto 0)
    -- odometry encoder interface
    ; quad_a_1            : in std_logic
    ; quad_b_1            : in std_logic
    -- hcsr04 interfaces
    ; us1_trig            : out std_logic
    ; us1_echo            : in std_logic
    ; us2_trig            : out std_logic
    ; us2_echo            : in std_logic
    ; us3_trig            : out std_logic
    ; us3_echo            : in std_logic
    -- misc actuator interfaces
    ; pwm_servo0          : out std_logic
    ; pwm_servo1          : out std_logic
    ; pwm_servo2          : out std_logic
    ; pwm_servo3          : out std_logic
    ; pwm_servo4          : out std_logic
    ; pwm_servo5          : out std_logic
    ; pwm_servo6          : out std_logic
    ; pwm_servo7          : out std_logic
    ; pwm_servo8          : out std_logic
    ; pwm_servo9          : out std_logic
    ; pwm_servo10         : out std_logic
    ; pwm_servo11         : out std_logic
    ; pwm_pump0           : out std_logic
    ; dir_pump0           : out std_logic
    ; pwm_pump1           : out std_logic
    ; dir_pump1           : out std_logic
    ; pwm_motor2          : out std_logic
    ; dir_motor2          : out std_logic
    ; stp_0_step          : out std_logic
    ; stp_0_dir           : out std_logic
    ; stp_1_step          : out std_logic
    ; stp_1_dir           : out std_logic
    ; stp_switch0         : in std_logic
    ; stp_switch1         : in std_logic
    -- I2C slave signals
    ; sda_in_slv          : in  std_logic
    ; sda_out_slv         : out std_logic
    ; sda_en_slv          : out std_logic
    ; scl_in_slv          : in  std_logic
    ; scl_out_slv         : out std_logic
    ; scl_en_slv          : out std_logic
    -- SPI slave signals
    ; spi_cs              : in std_logic
    ; spi_clk             : in std_logic
    ; spi_mosi            : in std_logic
    ; spi_miso            : out std_logic
    -- GPIO
    ; gpio_in             : in std_logic_vector(31 downto 0)
    -- debug/test
    ; debug_test          : out std_logic_vector(31 downto 0)
  );
end entity;

architecture rtl of robot_apb is

  component QuadratureCounterPorts is
    port (
      RESET               : in std_logic;
      clock               : in std_logic;
      QuadA               : in std_logic;
      QuadB               : in std_logic;
      Increment_4_12      : in std_logic_vector(15 downto 0);
      SamplingInterval    : in std_logic_vector(31 downto 0);
      AsyncReset          : in std_logic;
      CounterValue        : buffer std_logic_vector(31 downto 0);
      SpeedValue          : out std_logic_vector(31 downto 0);
      SetAux1             : in std_logic;
      SetValueAux1        : in std_logic_vector(31 downto 0);
      CounterValueAux1    : buffer std_logic_vector(31 downto 0);
      SpeedValueAux1      : out std_logic_vector(31 downto 0);

      IncrementAuxTh      : in std_logic_vector(63 downto 0);
      SetAuxTh            : in std_logic;
      SetValueAuxTh       : in std_logic_vector(63 downto 0);
      CounterValueAuxTh   : buffer std_logic_vector(63 downto 0);
      SpeedValueAuxTh     : out std_logic_vector(63 downto 0);
      IncrementAuxR       : in std_logic_vector(63 downto 0);
      SetAuxR             : in std_logic;
      SetValueAuxR        : in std_logic_vector(63 downto 0);
      CounterValueAuxR    : buffer std_logic_vector(63 downto 0);
      SpeedValueAuxR      : out std_logic_vector(63 downto 0)
    );
  end component;

  component ULTRASOUND_HCSR04 is
    port (
      RESET          : in std_logic;
      CLK            : in std_logic; -- the clock should be @ 25MHz
      ACTUAL_DIST    : out std_logic_vector (31 downto 0);
      US_PULSE       : out std_logic;
      US_RESPONSE    : in std_logic
      );
  end component;

  component SERVO is
    port (
      RESET              : in std_logic;
      CLK                : in std_logic;
      PWM_SERVO_PERIOD   : in std_logic_vector (31 downto 0);
      PWM_SERVO_PW       : in std_logic_vector (31 downto 0);
      PWM_SERVO          : out std_logic
      );
  end component;

  component PUMP is
    port (
      RESET              : in std_logic;
      CLK                : in std_logic;
      PWM_PUMP_PERIOD    : in std_logic_vector (31 downto 0);
      PWM_PUMP_PW        : in std_logic_vector (31 downto 0);
      PWM_PUMP           : out std_logic;
      DIR_PUMP           : out std_logic
      );
  end component;

  component robot_i2c_slave is
    port (
      RESET               : in std_logic;
      CLK                 : in std_logic;
      I2C_MASTER_RD       : out std_logic;
      I2C_MASTER_WR       : out std_logic;
      I2C_MASTER_ADDR     : out std_logic_vector(31 downto 0);
      I2C_MASTER_DATA     : out std_logic_vector(31 downto 0);
      I2C_SLAVE_DATA      : in std_logic_vector(31 downto 0);
      I2C_SLAVE_ACK       : in std_logic;
      I2C_SLAVE_IRQ       : out std_logic;
      TRACE_FIFO          : in std_logic_vector(31 downto 0);
      TRACE_FIFO_DEBUG    : out std_logic_vector(31 downto 0);
      TRACE_FIFO_WR       : in std_logic;
      TRACE_FIFO_FULL     : out std_logic;
      TRACE_FIFO_EMPTY    : out std_logic;
      BSTR_FIFO           : out std_logic_vector(31 downto 0);
      BSTR_FIFO_DEBUG     : out std_logic_vector(31 downto 0);
      BSTR_FIFO_RD        : in std_logic;
      BSTR_FIFO_FULL      : out std_logic;
      BSTR_FIFO_EMPTY     : out std_logic;
      SDA_IN              : in     std_logic;
      SDA_OUT             : out    std_logic;
      SDA_EN              : out    std_logic;
      SCL_IN              : in     std_logic;
      SCL_OUT             : out    std_logic;
      SCL_EN              : out    std_logic
    );
  end component;

  component robot_spi_slave is
    port (
      CLK                 : in std_logic;
      RESET               : in std_logic;
      SPI_MASTER_RD       : out std_logic;
      SPI_MASTER_WR       : out std_logic;
      SPI_MASTER_ADDR     : out std_logic_vector(31 downto 0);
      SPI_MASTER_DATA     : out std_logic_vector(31 downto 0);
      SPI_SLAVE_DATA      : in std_logic_vector(31 downto 0);
      SPI_SLAVE_BUSY      : in std_logic;
      SPI_SLAVE_ADDR      : in std_logic_vector(31 downto 0);
      SPI_SLAVE_IRQ       : out std_logic;
      DBG_MST_DATA        : out std_logic_vector(31 downto 0);
      DBG_SLV_DATA        : in std_logic_vector(31 downto 0);
      DBG_CRC_DATA        : out std_logic_vector(31 downto 0);
      DBG_CRC_WR          : out std_logic;
      SPI_CS              : in std_logic;
      SPI_CLK             : in std_logic;
      SPI_MOSI            : in std_logic;
      SPI_MISO            : out std_logic
    );
  end component;

component STEPPER_POLOLU
  port (
    RESET          : in std_logic;
    CLK            : in std_logic;
    CTRL           : in std_logic_vector (15 downto 0);
    PERIOD         : in std_logic_vector (15 downto 0);
    TARGET_POS     : in std_logic_vector (15 downto 0);
    SET_CUR_POS    : in std_logic_vector (15 downto 0);
    CUR_POS        : out std_logic_vector (15 downto 0);
    HIGH_SW        : in std_logic;
    LOW_SW         : in std_logic;
    N_ENABLE       : out std_logic;
    STEP           : out std_logic;
    DIR            : out std_logic
  );
end component;


  signal iRESET               : std_logic;

  signal iROBOT_TIMER         : std_logic_vector (31 downto 0);
  signal iROBOT_RESET         : std_logic_vector (31 downto 0);
  signal iDEBUG_REG           : std_logic_vector (31 downto 0);

  signal iTRACE_FIFO          : std_logic_vector (31 downto 0);
  signal iTRACE_FIFO_DEBUG    : std_logic_vector (31 downto 0);
  signal iTRACE_FIFO_WR       : std_logic;
  signal iTRACE_FIFO_FULL     : std_logic;
  signal iTRACE_FIFO_EMPTY    : std_logic;
  signal iBSTR_FIFO           : std_logic_vector (31 downto 0);
  signal iBSTR_FIFO_DEBUG     : std_logic_vector (31 downto 0);
  signal iBSTR_FIFO_RD        : std_logic;
  signal iBSTR_FIFO_EMPTY     : std_logic;
  signal iBSTR_FIFO_FULL      : std_logic;

  signal iODO_SAMPLING_INT    : std_logic_vector (31 downto 0);
  signal iODO_ASYNC_RESET     : std_logic;
  signal iODO_1_INC           : std_logic_vector (15 downto 0);
  signal iODO_1_POS           : std_logic_vector (31 downto 0);
  signal iODO_1_SPEED         : std_logic_vector (31 downto 0);

  signal iUS1_ACTUAL_DIST     : std_logic_vector (31 downto 0);
  signal iUS2_ACTUAL_DIST     : std_logic_vector (31 downto 0);
  signal iUS3_ACTUAL_DIST     : std_logic_vector (31 downto 0);

  signal iSERVO0_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO0_PW           : std_logic_vector (31 downto 0);
  signal iSERVO1_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO1_PW           : std_logic_vector (31 downto 0);
  signal iSERVO2_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO2_PW           : std_logic_vector (31 downto 0);
  signal iSERVO3_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO3_PW           : std_logic_vector (31 downto 0);
  signal iSERVO4_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO4_PW           : std_logic_vector (31 downto 0);
  signal iSERVO5_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO5_PW           : std_logic_vector (31 downto 0);
  signal iSERVO6_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO6_PW           : std_logic_vector (31 downto 0);
  signal iSERVO7_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO7_PW           : std_logic_vector (31 downto 0);
  signal iSERVO8_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO8_PW           : std_logic_vector (31 downto 0);
  signal iSERVO9_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iSERVO9_PW           : std_logic_vector (31 downto 0);
  signal iSERVO10_PWM_PERIOD  : std_logic_vector (31 downto 0);
  signal iSERVO10_PW          : std_logic_vector (31 downto 0);
  signal iSERVO11_PWM_PERIOD  : std_logic_vector (31 downto 0);
  signal iSERVO11_PW          : std_logic_vector (31 downto 0);

  signal iPUMP0_PWM_PERIOD    : std_logic_vector (31 downto 0);
  signal iPUMP0_PW            : std_logic_vector (31 downto 0);
  signal iPUMP1_PWM_PERIOD    : std_logic_vector (31 downto 0);
  signal iPUMP1_PW            : std_logic_vector (31 downto 0);
  signal iMOTOR2_PWM_PERIOD   : std_logic_vector (31 downto 0);
  signal iMOTOR2_PW           : std_logic_vector (31 downto 0);

  signal iSTP_POL_0_CTRL      : std_logic_vector (15 downto 0);
  signal iSTP_POL_0_PERIOD    : std_logic_vector (15 downto 0);
  signal iSTP_POL_0_TARGET_POS  : std_logic_vector (15 downto 0);
  signal iSTP_POL_0_CUR_POS   : std_logic_vector (15 downto 0);
  signal iSTP_POL_0_SET_CUR_POS : std_logic_vector (15 downto 0);
--  signal iSTP_POL_0_NEN       : std_logic; -- FIXME : TODO
  signal iSTP_POL_0_STEP      : std_logic;
  signal iSTP_POL_0_DIR       : std_logic;

  signal iSTP_POL_1_CTRL      : std_logic_vector (15 downto 0);
  signal iSTP_POL_1_PERIOD    : std_logic_vector (15 downto 0);
  signal iSTP_POL_1_TARGET_POS  : std_logic_vector (15 downto 0);
  signal iSTP_POL_1_CUR_POS   : std_logic_vector (15 downto 0);
  signal iSTP_POL_1_SET_CUR_POS : std_logic_vector (15 downto 0);
--  signal iSTP_POL_1_NEN       : std_logic; -- FIXME : TODO
  signal iSTP_POL_1_STEP      : std_logic;
  signal iSTP_POL_1_DIR       : std_logic;

  signal iROBOT2018_BAL0      : std_logic_vector (31 downto 0);
  signal iROBOT2018_BAL1      : std_logic_vector (31 downto 0);
  signal iROBOT2018_BAL2      : std_logic_vector (31 downto 0);
  signal iROBOT2018_BAL3      : std_logic_vector (31 downto 0);

  signal iI2C_MASTER_RD       : std_logic;
  signal iI2C_MASTER_WR       : std_logic;
  signal iI2C_MASTER_ADDR     : std_logic_vector (31 downto 0);
  signal iI2C_MASTER_DATA     : std_logic_vector (31 downto 0);
  signal iI2C_SLAVE_DATA      : std_logic_vector (31 downto 0);

  signal iMST_READ            : std_logic;
  signal iMST_WRITE           : std_logic;
  signal iMST_ADDR            : std_logic_vector (31 downto 0);
  signal iMST_WDATA           : std_logic_vector (31 downto 0);
  signal iMST_RDATA           : std_logic_vector (31 downto 0);

  signal iSPI_MASTER_RD       : std_logic;
  signal iSPI_MASTER_WR       : std_logic;
  signal iSPI_MASTER_ADDR     : std_logic_vector (31 downto 0);
  signal iSPI_MASTER_DATA     : std_logic_vector (31 downto 0);
  signal iSPI_SLAVE_BUSY      : std_logic;
  signal iSPI_SLAVE_DATA      : std_logic_vector (31 downto 0);
  signal iSPI_SLAVE_ADDR      : std_logic_vector (31 downto 0);
  signal iSPI_DBG_MST_DATA    : std_logic_vector (31 downto 0);
  signal iSPI_DBG_SLV_DATA    : std_logic_vector (31 downto 0);

begin

  iRESET <= iROBOT_RESET(0) or (not presetn);

  debug_test <= iDEBUG_REG;

  -- FIXME : DEBUG
  iODO_ASYNC_RESET <= iDEBUG_REG(30);
  
-- FIXME : TODO ++
--  c_us1 : ULTRASOUND_HCSR04
--    port map (
--      RESET => iRESET,
--      CLK => pclk,
--      ACTUAL_DIST => iUS1_ACTUAL_DIST,
--      US_PULSE => us1_trig,
--      US_RESPONSE => us1_echo
--    );
-- FIXME : TODO ==
  us1_trig <= '0';
  iUS1_ACTUAL_DIST <= (others => '0');
-- FIXME : TODO --

-- FIXME : TODO ++
--  c_us2 : ULTRASOUND_HCSR04
--    port map (
--      RESET => iRESET,
--      CLK => pclk,
--      ACTUAL_DIST => iUS2_ACTUAL_DIST,
--      US_PULSE => us2_trig,
--      US_RESPONSE => us2_echo
--    );
-- FIXME : TODO ==
  us2_trig <= '0';
  iUS2_ACTUAL_DIST <= (others => '0');
-- FIXME : TODO --

-- FIXME : TODO ++
--  c_us3 : ULTRASOUND_HCSR04
--    port map (
--      RESET => iRESET,
--      CLK => pclk,
--      ACTUAL_DIST => iUS3_ACTUAL_DIST,
--      US_PULSE => us3_trig,
--      US_RESPONSE => us3_echo
--    );
-- FIXME : TODO ==
  us3_trig <= '0';
  iUS3_ACTUAL_DIST <= (others => '0');
-- FIXME : TODO --

  c_quad_1 : QuadratureCounterPorts
    port map (
      RESET => iRESET,
      clock => pclk,
      QuadA => quad_a_1,
      QuadB => quad_b_1,
      Increment_4_12 => iODO_1_INC,
      SamplingInterval => iODO_SAMPLING_INT,
      AsyncReset => iODO_ASYNC_RESET,
      CounterValue => iODO_1_POS,
      SpeedValue => iODO_1_SPEED,
      SetAux1 => '0',
      SetValueAux1 => (others => '0'),
      CounterValueAux1 => open,
      SpeedValueAux1 => open,

      IncrementAuxTh => (others => '0'),
      SetAuxTh => '0',
      SetValueAuxTh => (others => '0'),
      CounterValueAuxTh => open,
      SpeedValueAuxTh => open,
      IncrementAuxR => (others => '0'),
      SetAuxR => '0',
      SetValueAuxR => (others => '0'),
      CounterValueAuxR => open,
      SpeedValueAuxR => open
    );

  c_servo0 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO0_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO0_PW,
      PWM_SERVO => pwm_servo0
    );

  c_servo1 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO1_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO1_PW,
      PWM_SERVO => pwm_servo1
    );

  c_servo2 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO2_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO2_PW,
      PWM_SERVO => pwm_servo2
    );

  c_servo3 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO3_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO3_PW,
      PWM_SERVO => pwm_servo3
    );

  c_servo4 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO4_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO4_PW,
      PWM_SERVO => pwm_servo4
    );

  c_servo5 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO5_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO5_PW,
      PWM_SERVO => pwm_servo5
    );

  c_servo6 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO6_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO6_PW,
      PWM_SERVO => pwm_servo6
    );

  c_servo7 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO7_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO7_PW,
      PWM_SERVO => pwm_servo7
    );

  c_servo8 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO8_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO8_PW,
      PWM_SERVO => pwm_servo8
    );

  c_servo9 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO9_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO9_PW,
      PWM_SERVO => pwm_servo9
    );

  c_servo10 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO10_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO10_PW,
      PWM_SERVO => pwm_servo10
    );

  c_servo11 : SERVO
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_SERVO_PERIOD => iSERVO11_PWM_PERIOD,
      PWM_SERVO_PW => iSERVO11_PW,
      PWM_SERVO => pwm_servo11
    );

  c_pump0 : PUMP
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_PUMP_PERIOD => iPUMP0_PWM_PERIOD,
      PWM_PUMP_PW => iPUMP0_PW,
      PWM_PUMP => pwm_pump0,
      DIR_PUMP => dir_pump0
    );

  c_pump1 : PUMP
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_PUMP_PERIOD => iPUMP1_PWM_PERIOD,
      PWM_PUMP_PW => iPUMP1_PW,
      PWM_PUMP => pwm_pump1,
      DIR_PUMP => dir_pump1
    );

  c_motor2 : PUMP
    port map (
      RESET => iRESET,
      CLK => pclk,
      PWM_PUMP_PERIOD => iMOTOR2_PWM_PERIOD,
      PWM_PUMP_PW => iMOTOR2_PW,
      PWM_PUMP => pwm_motor2,
      DIR_PUMP => dir_motor2
    );

  c_stepper_pololu0 : STEPPER_POLOLU
    port map (
      RESET => iRESET,
      CLK => pclk,
      CTRL => iSTP_POL_0_CTRL,
      PERIOD => iSTP_POL_0_PERIOD,
      TARGET_POS => iSTP_POL_0_TARGET_POS,
      CUR_POS => iSTP_POL_0_CUR_POS,
      SET_CUR_POS => iSTP_POL_0_SET_CUR_POS,
      HIGH_SW => '0',
      LOW_SW => '0',
      N_ENABLE => open,
      STEP => stp_0_step,
      DIR => stp_0_dir
    );

  c_stepper_pololu1 : STEPPER_POLOLU
    port map (
      RESET => iRESET,
      CLK => pclk,
      CTRL => iSTP_POL_1_CTRL,
      PERIOD => iSTP_POL_1_PERIOD,
      TARGET_POS => iSTP_POL_1_TARGET_POS,
      CUR_POS => iSTP_POL_1_CUR_POS,
      SET_CUR_POS => iSTP_POL_1_SET_CUR_POS,
      HIGH_SW => '0',
      LOW_SW => '0',
      N_ENABLE => open,
      STEP => stp_1_step,
      DIR => stp_1_dir
    );

  c_robot_spi_slave : ROBOT_SPI_SLAVE
    port map (
--      CLK => clk_i_100,
      CLK => pclk,
      RESET => iRESET,
      SPI_MASTER_RD => iSPI_MASTER_RD,
      SPI_MASTER_WR => iSPI_MASTER_WR,
      SPI_MASTER_ADDR => iSPI_MASTER_ADDR,
      SPI_MASTER_DATA => iSPI_MASTER_DATA,
      SPI_SLAVE_DATA => iSPI_SLAVE_DATA,
      SPI_SLAVE_BUSY => iSPI_SLAVE_BUSY,
      SPI_SLAVE_ADDR => iSPI_SLAVE_ADDR,
      SPI_SLAVE_IRQ => open,
      DBG_MST_DATA => iSPI_DBG_MST_DATA,
      DBG_SLV_DATA => iSPI_DBG_SLV_DATA,
      DBG_CRC_DATA => open,
      DBG_CRC_WR => open,
      SPI_CS => spi_cs,
      SPI_CLK => spi_clk,
      SPI_MOSI => spi_mosi,
      SPI_MISO => spi_miso
      );

  c_robot_i2c_slave : robot_i2c_slave
    port map (
      RESET => iRESET,
      CLK => pclk,
      -- generic internal interface 
      I2C_MASTER_RD => iI2C_MASTER_RD,
      I2C_MASTER_WR => iI2C_MASTER_WR,
      I2C_MASTER_ADDR => iI2C_MASTER_ADDR,
      I2C_MASTER_DATA => iI2C_MASTER_DATA,
      I2C_SLAVE_DATA => iI2C_SLAVE_DATA,
      I2C_SLAVE_ACK => '1', -- FIXME : TODO
      I2C_SLAVE_IRQ => open, -- FIXME : TODO
      -- trace fifo
      TRACE_FIFO => iTRACE_FIFO,
      TRACE_FIFO_DEBUG => iTRACE_FIFO_DEBUG,
      TRACE_FIFO_WR => iTRACE_FIFO_WR,
      TRACE_FIFO_FULL => iTRACE_FIFO_FULL,
      TRACE_FIFO_EMPTY => iTRACE_FIFO_EMPTY,
      -- bitstream fifo
      BSTR_FIFO => iBSTR_FIFO,
      BSTR_FIFO_DEBUG => iBSTR_FIFO_DEBUG,
      BSTR_FIFO_RD => iBSTR_FIFO_RD,
      BSTR_FIFO_FULL => iBSTR_FIFO_FULL,
      BSTR_FIFO_EMPTY => iBSTR_FIFO_EMPTY,
      -- I2C (external) interface
      SDA_IN => sda_in_slv,
      SDA_OUT => sda_out_slv,
      SDA_EN => sda_en_slv,
      SCL_IN => scl_in_slv,
      SCL_OUT => scl_out_slv,
      SCL_EN => scl_en_slv
    );


-- timer process
  timer_proc : process (iRESET, pclk)
    variable local_counter : integer := 0;
  begin
    if iRESET = '1' then
      iROBOT_TIMER <= (others => '0');
      local_counter := 0;
    elsif rising_edge(pclk) then
      if ( local_counter = 24 ) then
        iROBOT_TIMER <= iROBOT_TIMER + 1;
        local_counter := 0;
      else
        local_counter := local_counter + 1;
      end if;
    end if;
  end process;
  

---- Multiplexeur pour les 3 interfaces master : APB, I2C et SPI
--  iMST_READ  <= '1' when (((psel='1') and (penable='1') and (pwrite='0')) or
--                (iI2C_MASTER_RD='1') or (iSPI_MASTER_RD='1')) else '0';
--  iMST_WRITE <= '1' when (((psel='1') and (penable='1') and (pwrite='1')) or
--                (iI2C_MASTER_WR='1') or (iSPI_MASTER_WR='1')) else '0';
--  iMST_ADDR  <= paddr when ((psel='1') and (penable='1')) else
--                iSPI_MASTER_ADDR when (iDEBUG_REG(31) = '1') else
--                iI2C_MASTER_ADDR;
--  iMST_WDATA <= pwdata when ((psel='1') and (penable='1') and (pwrite='1')) else
--                iSPI_MASTER_DATA when (iDEBUG_REG(31) = '1') else
--                iI2C_MASTER_DATA;
--  prdata          <= iMST_RDATA;
--  iI2C_SLAVE_DATA <= iMST_RDATA;
--  iSPI_SLAVE_DATA <= iMST_RDATA;

-- Multiplexeur pour les 2 interfaces master : APB et SPI
  iMST_READ  <= '1' when (((psel='1') and (penable='1') and (pwrite='0')) or
                (iSPI_MASTER_RD='1')) else '0';
  iMST_WRITE <= '1' when (((psel='1') and (penable='1') and (pwrite='1')) or
                (iSPI_MASTER_WR='1')) else '0';
  iMST_ADDR  <= paddr when ((psel='1') and (penable='1')) else
                iSPI_MASTER_ADDR;
  iSPI_SLAVE_ADDR <= iMST_ADDR;
  iMST_WDATA <= pwdata when ((psel='1') and (penable='1') and (pwrite='1')) else
                iSPI_MASTER_DATA;
  prdata          <= iMST_RDATA;
  iSPI_SLAVE_BUSY <= '1' when ((psel='1') and (penable='1')) else '0';
  iSPI_SLAVE_DATA <= iMST_RDATA;

-- APB Write process
  write_proc : process (presetn, pclk)
  begin
    if presetn = '0' then
      iROBOT_RESET       <= (others => '0');

      iDEBUG_REG         <= X"00000042";

      iODO_SAMPLING_INT  <= X"000F423F"; -- 40 ms avec pclk=25MHz
      iODO_1_INC         <= X"1000";

      iSERVO0_PWM_PERIOD <= X"00040000";
      iSERVO0_PW         <= (others => '0');
      iSERVO1_PWM_PERIOD <= X"00040000";
      iSERVO1_PW         <= (others => '0');
      iSERVO2_PWM_PERIOD <= X"00040000";
      iSERVO2_PW         <= (others => '0');
      iSERVO3_PWM_PERIOD <= X"00040000";
      iSERVO3_PW         <= (others => '0');
      iSERVO4_PWM_PERIOD <= X"00040000";
      iSERVO4_PW         <= (others => '0');
      iSERVO5_PWM_PERIOD <= X"00040000";
      iSERVO5_PW         <= (others => '0');
      iSERVO6_PWM_PERIOD <= X"00040000";
      iSERVO6_PW         <= (others => '0');
      iSERVO7_PWM_PERIOD <= X"00040000";
      iSERVO7_PW         <= (others => '0');
      iSERVO8_PWM_PERIOD <= X"00040000";
      iSERVO8_PW         <= (others => '0');
      iSERVO9_PWM_PERIOD <= X"00040000";
      iSERVO9_PW         <= (others => '0');
      iSERVO10_PWM_PERIOD <= X"00040000";
      iSERVO10_PW         <= (others => '0');
      iSERVO11_PWM_PERIOD <= X"00040000";
      iSERVO11_PW         <= (others => '0');

      iPUMP0_PWM_PERIOD  <= X"00000200";
      iPUMP0_PW          <= (others => '0');
      iPUMP1_PWM_PERIOD  <= X"00000200";
      iPUMP1_PW          <= (others => '0');
      iMOTOR2_PWM_PERIOD <= X"00000200";
      iMOTOR2_PW         <= (others => '0');

      iTRACE_FIFO        <= (others => '0');
      iTRACE_FIFO_WR     <= '0';

      iSTP_POL_0_CTRL     <= X"0000";
      iSTP_POL_0_PERIOD   <= X"0010";
      iSTP_POL_0_TARGET_POS  <= X"FFFF";
      iSTP_POL_0_SET_CUR_POS <= X"FFFF";

      iSTP_POL_1_CTRL     <= X"0000";
      iSTP_POL_1_PERIOD   <= X"0010";
      iSTP_POL_1_TARGET_POS  <= X"FFFF";
      iSTP_POL_1_SET_CUR_POS <= X"FFFF";

      iROBOT2018_BAL0     <= X"00000000";
      iROBOT2018_BAL1     <= X"00000000";
      iROBOT2018_BAL2     <= X"00000000";
      iROBOT2018_BAL3     <= X"00000000";

-- FIXME : DEBUG ++
      iSPI_DBG_SLV_DATA  <= (others => '0');
-- FIXME : DEBUG --

    elsif rising_edge(pclk) then
      if (iMST_WRITE = '1') then
        case iMST_ADDR(11 downto 2) is
          -- timer & reset
          when "0000000000" => -- 0x80008000 -- robot_reg[0x00]
            null;
          when "0000000001" => -- 0x80008004 -- robot_reg[0x01]
            iROBOT_RESET <= iMST_WDATA;
          when "0000000010" => -- 0x80008008 -- robot_reg[0x02]
            iDEBUG_REG <= iMST_WDATA;
          when "0000000011" => -- 0x8000800c -- robot_reg[0x03]
            null;

          -- was uart test in 2016
          when "0000000100" => -- 0x80008010 -- robot_reg[0x04]
            null; -- <available>
          when "0000000101" => -- 0x80008014 -- robot_reg[0x05]
            null; -- <available>
          when "0000000110" => -- 0x80008018 -- robot_reg[0x06]
            null; -- <available>
          when "0000000111" => -- 0x8000801c -- robot_reg[0x07]
            null; -- <available>

          -- i2c slave
          when "0000001000" => -- 0x80008020 -- robot_reg[0x08]
            null; -- FIXME : TODO : control (from APB)
          when "0000001001" => -- 0x80008024 -- robot_reg[0x09]
            null; -- status (from I2C master)
          when "0000001010" => -- 0x80008028 -- robot_reg[0x0a]
            null; -- ADDR from I2C master
          when "0000001011" => -- 0x8000802c -- robot_reg[0x0b]
            null; -- FIXME : TODO : DATA from APB
          when "0000001100" => -- 0x80008030 -- robot_reg[0x0c]
            null; -- FIXME : TODO : TRACE control
          when "0000001101" => -- 0x80008034 -- robot_reg[0x0d]
            iTRACE_FIFO <= iMST_WDATA; -- TRACE FIFO wr
            iTRACE_FIFO_WR <= '1';
          when "0000001110" => -- 0x80008038 -- robot_reg[0x0e]
            null; -- FIXME : TODO : BSTR control
          when "0000001111" => -- 0x8000803c -- robot_reg[0x0f]
            null; -- BSTR read-only for APB

          -- was motors in 2016
          when "0001000000" => -- 0x80008100 -- robot_reg[0x40]
            null; -- <available>
          when "0001001000" => -- 0x80008120 -- robot_reg[0x48]
            null; -- <available>

          -- was odometry in 2016
          when "0010000001" => -- 0x80008204 -- robot_reg[0x81]
            null; -- <available>
          when "0010000011" => -- 0x8000820c -- robot_reg[0x83]
            iODO_1_INC <= iMST_WDATA(15 downto 0);
          when "0010000100" => -- 0x80008210 -- robot_reg[0x84]
            null; -- <available>
          when "0010000111" => -- 0x8000821c -- robot_reg[0x87]
            iODO_SAMPLING_INT <= iMST_WDATA;
          when "0010001001" => -- 0x80008224 -- robot_reg[0x89]
            null; -- <available>
          when "0010001011" => -- 0x8000822c -- robot_reg[0x8b]
            null; -- <available>
          when "0010001100" => -- 0x80008230 -- robot_reg[0x8c]
            null; -- <available>
          when "0010001111" => -- 0x8000823c -- robot_reg[0x8f]
            null; -- <available>

          -- was "advanced" odometry in 2016
          when "0010010001" => -- 0x80008244 -- robot_reg[0x91]
            null; -- <available>
          when "0010010011" => -- 0x8000824c -- robot_reg[0x93]
            null; -- <available>
          when "0010010100" => -- 0x80008250 -- robot_reg[0x94]
            null; -- <available>
          when "0010010111" => -- 0x8000825c -- robot_reg[0x97]
            null; -- <available>
          when "0010011001" => -- 0x80008264 -- robot_reg[0x99]
            null; -- <available>
          when "0010011011" => -- 0x8000826c -- robot_reg[0x9b]
            null; -- <available>
          when "0010011100" => -- 0x80008270 -- robot_reg[0x9c]
            null; -- <available>
          when "0010011111" => -- 0x8000827c -- robot_reg[0x9f]
            null; -- <available>

          when "0010100001" => -- 0x80008284 -- robot_reg[0xa1]
            null; -- <available>
          when "0010100011" => -- 0x8000828c -- robot_reg[0xa3]
            null; -- <available>
          when "0010100100" => -- 0x80008290 -- robot_reg[0xa4]
            null; -- <available>
          when "0010100111" => -- 0x8000829c -- robot_reg[0xa7]
            null; -- <available>
          when "0010101001" => -- 0x800082a4 -- robot_reg[0xa9]
            null; -- <available>
          when "0010101011" => -- 0x800082ac -- robot_reg[0xab]
            null; -- <available>
          when "0010101100" => -- 0x800082b0 -- robot_reg[0xac]
            null; -- <available>
          when "0010101111" => -- 0x800082bc -- robot_reg[0xaf]
            null; -- <available>

          -- was 64bit multiplier in 2016
          when "0010110000" => -- 0x800082c0 -- robot_reg[0xb0]
            null; -- <available>
          when "0010110001" => -- 0x800082c4 -- robot_reg[0xb1]
            null; -- <available>
          when "0010110010" => -- 0x800082c8 -- robot_reg[0xb2]
            null; -- <available>
          when "0010110011" => -- 0x800082cc -- robot_reg[0xb3]
            null; -- <available>

          -- was sin_cos in 2016
          when "0010111000" => -- 0x800082e0 -- robot_reg[0xb8]
            null; -- <available>
          when "0010111001" => -- 0x800082e4 -- robot_reg[0xb9]
            null; -- <available>
          when "0010111010" => -- 0x800082e8 -- robot_reg[0xba]
            null; -- <available>
          when "0010111011" => -- 0x800082ec -- robot_reg[0xbb]
            null; -- <available>

          -- ultrasound
          when "0011000000" => -- 0x80008300 -- robot_reg[0xc0]
            null; -- iUS1_ACTUAL_DIST
          when "0011000001" => -- 0x80008304 -- robot_reg[0xc1]
            null; -- iUS2_ACTUAL_DIST
          when "0011000010" => -- 0x80008308 -- robot_reg[0xc2]
            null; -- iUS3_ACTUAL_DIST
          when "0011000011" => -- 0x8000830c -- robot_reg[0xc3]
            null; -- <available>

          when "0011000100" => -- 0x80008310 -- robot_reg[0xc4]
            null; -- <available>
          when "0011000101" => -- 0x80008314 -- robot_reg[0xc5]
            null; -- <available>
          when "0011000110" => -- 0x80008318 -- robot_reg[0xc6]
            null; -- <available>
          when "0011000111" => -- 0x8000831c -- robot_reg[0xc7]
            null; -- <available>

          when "0011001000" => -- 0x80008320 -- robot_reg[0xc8]
            null; -- <available>
          when "0011001001" => -- 0x80008324 -- robot_reg[0xc9]
            null; -- <available>
          when "0011001010" => -- 0x80008328 -- robot_reg[0xca]
            null; -- <available>
          when "0011001011" => -- 0x8000832c -- robot_reg[0xcb]
            null; -- <available>

          when "0011001100" => -- 0x80008330 -- robot_reg[0xcc]
            null; -- <available>
          when "0011001101" => -- 0x80008334 -- robot_reg[0xcd]
            null; -- <available>
          when "0011001110" => -- 0x80008338 -- robot_reg[0xce]
            null; -- <available>
          when "0011001111" => -- 0x8000833c -- robot_reg[0xcf]
            null; -- <available>

          -- was gps in 2016
          when "0011010000" => -- 0x80008340 -- robot_reg[0xd0]
            null; -- <available>
          when "0011010001" => -- 0x80008344 -- robot_reg[0xd1]
            null; -- <available>
          when "0011010010" => -- 0x80008348 -- robot_reg[0xd2]
            null; -- <available>
          when "0011010011" => -- 0x8000834c -- robot_reg[0xd3]
            null; -- <available>
          when "0011010100" => -- 0x80008350 -- robot_reg[0xd4]
            null; -- <available>
          when "0011010101" => -- 0x80008354 -- robot_reg[0xd5]
            null; -- <available>
          when "0011010110" => -- 0x80008358 -- robot_reg[0xd6]
            null; -- <available>
          when "0011010111" => -- 0x8000835c -- robot_reg[0xd7]
            null; -- <available>

          -- ROBOT_SPI_SLAVE : esclave SPI
          when "0011011000" => -- 0x80008360 -- robot_reg[0xd8]
            iSPI_DBG_SLV_DATA <= iMST_WDATA;
          when "0011011001" => -- 0x80008364 -- robot_reg[0xd9]
            null; -- <available> -- iSPI_DBG_MST_DATA
          when "0011011010" => -- 0x80008368 -- robot_reg[0xda]
            null; -- <available>
          when "0011011011" => -- 0x8000836c -- robot_reg[0xdb]
            null; -- <available>
          when "0011011100" => -- 0x80008370 -- robot_reg[0xdc]
            null; -- <available>
          when "0011011101" => -- 0x80008374 -- robot_reg[0xdd]
            null; -- <available>
          when "0011011110" => -- 0x80008378 -- robot_reg[0xde]
            null; -- <available>
          when "0011011111" => -- 0x8000837c -- robot_reg[0xdf]
            null; -- <available>

          -- SERVO PETIT ROBOT 2018
          when "0100000000" => -- 0x80008400 -- robot_reg[0x100]
            iSERVO0_PWM_PERIOD <= iMST_WDATA;
          when "0100000001" => -- 0x80008404 -- robot_reg[0x101]
            iSERVO0_PW         <= iMST_WDATA;
          when "0100000010" => -- 0x80008408 -- robot_reg[0x102]
            iSERVO1_PWM_PERIOD <= iMST_WDATA;
          when "0100000011" => -- 0x8000840c -- robot_reg[0x103]
            iSERVO1_PW         <= iMST_WDATA;
          when "0100000100" => -- 0x80008410 -- robot_reg[0x104]
            iSERVO2_PWM_PERIOD <= iMST_WDATA;
          when "0100000101" => -- 0x80008414 -- robot_reg[0x105]
            iSERVO2_PW         <= iMST_WDATA;
          when "0100000110" => -- 0x80008418 -- robot_reg[0x106]
            iSERVO3_PWM_PERIOD <= iMST_WDATA;
          when "0100000111" => -- 0x8000841c -- robot_reg[0x107]
            iSERVO3_PW         <= iMST_WDATA;
          when "0100001000" => -- 0x80008420 -- robot_reg[0x108]
            iSERVO4_PWM_PERIOD <= iMST_WDATA;
          when "0100001001" => -- 0x80008424 -- robot_reg[0x109]
            iSERVO4_PW         <= iMST_WDATA;
          when "0100001010" => -- 0x80008428 -- robot_reg[0x10a]
            iSERVO5_PWM_PERIOD <= iMST_WDATA;
          when "0100001011" => -- 0x8000842c -- robot_reg[0x10b]
            iSERVO5_PW         <= iMST_WDATA;
          when "0100001100" => -- 0x80008430 -- robot_reg[0x10c]
            iSERVO6_PWM_PERIOD <= iMST_WDATA;
          when "0100001101" => -- 0x80008434 -- robot_reg[0x10d]
            iSERVO6_PW         <= iMST_WDATA;
          when "0100001110" => -- 0x80008438 -- robot_reg[0x10e]
            iSERVO7_PWM_PERIOD <= iMST_WDATA;
          when "0100001111" => -- 0x8000843c -- robot_reg[0x10f]
            iSERVO7_PW         <= iMST_WDATA;
          when "0100010000" => -- 0x80008440 -- robot_reg[0x110]
            iSERVO8_PWM_PERIOD <= iMST_WDATA;
          when "0100010001" => -- 0x80008444 -- robot_reg[0x111]
            iSERVO8_PW         <= iMST_WDATA;
          when "0100010010" => -- 0x80008448 -- robot_reg[0x112]
            iSERVO9_PWM_PERIOD <= iMST_WDATA;
          when "0100010011" => -- 0x8000844c -- robot_reg[0x113]
            iSERVO9_PW         <= iMST_WDATA;
          when "0100010100" => -- 0x80008450 -- robot_reg[0x114]
            iSERVO10_PWM_PERIOD <= iMST_WDATA;
          when "0100010101" => -- 0x80008454 -- robot_reg[0x115]
            iSERVO10_PW         <= iMST_WDATA;
          when "0100010110" => -- 0x80008458 -- robot_reg[0x116]
            iSERVO11_PWM_PERIOD <= iMST_WDATA;
          when "0100010111" => -- 0x8000845c -- robot_reg[0x117]
            iSERVO11_PW         <= iMST_WDATA;
          when "0100011000" => -- 0x80008460 -- robot_reg[0x118]
            null; -- <available>
          when "0100011001" => -- 0x80008464 -- robot_reg[0x119]
            null; -- <available>
          when "0100011010" => -- 0x80008468 -- robot_reg[0x11a]
            null; -- <available>
          when "0100011011" => -- 0x8000846c -- robot_reg[0x11b]
            null; -- <available>
          when "0100011100" => -- 0x80008470 -- robot_reg[0x11c]
            null; -- <available>
          when "0100011101" => -- 0x80008474 -- robot_reg[0x11d]
            null; -- <available>
          when "0100011110" => -- 0x80008478 -- robot_reg[0x11e]
            null; -- <available>
          when "0100011111" => -- 0x8000847c -- robot_reg[0x11f]
            null; -- <available>

          -- MOTEURS CC PETIT ROBOT 2018
          when "0100100000" => -- 0x80008480 -- robot_reg[0x120]
            iPUMP0_PWM_PERIOD <= iMST_WDATA;
          when "0100100001" => -- 0x80008484 -- robot_reg[0x121]
            iPUMP0_PW         <= iMST_WDATA;
          when "0100100010" => -- 0x80008488 -- robot_reg[0x122]
            iPUMP1_PWM_PERIOD <= iMST_WDATA;
          when "0100100011" => -- 0x8000848c -- robot_reg[0x123]
            iPUMP1_PW         <= iMST_WDATA;
          when "0100100100" => -- 0x80008490 -- robot_reg[0x124]
            iMOTOR2_PWM_PERIOD <= iMST_WDATA;
          when "0100100101" => -- 0x80008494 -- robot_reg[0x125]
            iMOTOR2_PW         <= iMST_WDATA;
          when "0100100110" => -- 0x80008498 -- robot_reg[0x126]
            null; -- <available>
          when "0100100111" => -- 0x8000849c -- robot_reg[0x127]
            null; -- <available>
          when "0100101000" => -- 0x800084a0 -- robot_reg[0x128]
            null; -- <available>
          when "0100101001" => -- 0x800084a4 -- robot_reg[0x129]
            null; -- <available>
          when "0100101010" => -- 0x800084a8 -- robot_reg[0x12a]
            null; -- <available>
          when "0100101011" => -- 0x800084ac -- robot_reg[0x12b]
            null; -- <available>
          when "0100101100" => -- 0x800084b0 -- robot_reg[0x12c]
            null; -- <available>
          when "0100101101" => -- 0x800084b4 -- robot_reg[0x12d]
            null; -- <available>
          when "0100101110" => -- 0x800084b8 -- robot_reg[0x12e]
            null; -- <available>
          when "0100101111" => -- 0x800084bc -- robot_reg[0x12f]
            null; -- <available>

          -- MOTEURS STEPPER PETIT ROBOT 2018
          when "0100110000" => -- 0x800084c0 -- robot_reg[0x130]
            iSTP_POL_0_PERIOD   <= iMST_WDATA(31 downto 16);
            iSTP_POL_0_CTRL     <= iMST_WDATA(15 downto 0);
          when "0100110001" => -- 0x800084c4 -- robot_reg[0x131]
            iSTP_POL_0_TARGET_POS  <= iMST_WDATA(15 downto 0);
            iSTP_POL_0_SET_CUR_POS <= iMST_WDATA(31 downto 16);
          when "0100110010" => -- 0x800084c8 -- robot_reg[0x132]
            iSTP_POL_1_PERIOD   <= iMST_WDATA(31 downto 16);
            iSTP_POL_1_CTRL     <= iMST_WDATA(15 downto 0);
          when "0100110011" => -- 0x800084cc -- robot_reg[0x133]
            iSTP_POL_1_TARGET_POS  <= iMST_WDATA(15 downto 0);
            iSTP_POL_1_SET_CUR_POS <= iMST_WDATA(31 downto 16);
          when "0100110100" => -- 0x800084d0 -- robot_reg[0x134]
            null; -- <available>
          when "0100110101" => -- 0x800084d4 -- robot_reg[0x135]
            null; -- <available>
          when "0100110110" => -- 0x800084d8 -- robot_reg[0x136]
            null; -- <available>
          when "0100110111" => -- 0x800084dc -- robot_reg[0x137]
            null; -- <available>

          -- BAL & capteurs PETIT ROBOT 2018
          when "0100111000" => -- 0x800084e0 -- robot_reg[0x138]
            null; -- RO : stp_switch1 & stp_switch0;
          when "0100111001" => -- 0x800084e4 -- robot_reg[0x139]
            null; -- <available>
          when "0100111010" => -- 0x800084e8 -- robot_reg[0x13a]
            null; -- <available>
          when "0100111011" => -- 0x800084ec -- robot_reg[0x13b]
            null; -- <available>
          when "0100111100" => -- 0x800084f0 -- robot_reg[0x13c]
            iROBOT2018_BAL0 <= iMST_WDATA;
          when "0100111101" => -- 0x800084f4 -- robot_reg[0x13d]
            iROBOT2018_BAL1 <= iMST_WDATA;
          when "0100111110" => -- 0x800084f8 -- robot_reg[0x13e]
            iROBOT2018_BAL2 <= iMST_WDATA;
          when "0100111111" => -- 0x800084fc -- robot_reg[0x13f]
            iROBOT2018_BAL3 <= iMST_WDATA;

          when others =>
        end case;
      else

        iTRACE_FIFO_WR <= '0';

      end if;
    end if;
  end process;
  
-- APB Read process
  read_proc : process(presetn, iMST_READ)
  begin
    if presetn = '0' then
      iMST_RDATA <= (others => '1');
      iBSTR_FIFO_RD <= '0';
      iBSTR_FIFO_DEBUG <= (others => '0');
    elsif (iMST_READ = '1') then
      case iMST_ADDR(11 downto 2) is
        -- timer & reset
        when "0000000000" => -- 0x80008000 -- robot_reg[0x00]
          iMST_RDATA <= iROBOT_TIMER;
        when "0000000001" => -- 0x80008004 -- robot_reg[0x01]
          -- FIXME : TODO : GPIO 2018
--          iMST_RDATA <= (others => '0');
          -- FIXME : DEBUG : SPI
          iMST_RDATA <= iSPI_MASTER_ADDR;
        when "0000000010" => -- 0x80008008 -- robot_reg[0x02]
          iMST_RDATA <= iDEBUG_REG;
        when "0000000011" => -- 0x8000800c -- robot_reg[0x03]
--          iMST_RDATA <= X"54455354"; -- 'TEST' TAG
          iMST_RDATA <= VERSION;

        -- was uart test in 2016
        when "0000000100" => -- 0x80008010 -- robot_reg[0x04]
          iMST_RDATA <= (others => '0');
        when "0000000101" => -- 0x80008014 -- robot_reg[0x05]
          iMST_RDATA <= (others => '0');
        when "0000000110" => -- 0x80008018 -- robot_reg[0x06]
          iMST_RDATA <= (others => '0');
        when "0000000111" => -- 0x8000801c -- robot_reg[0x07]
          iMST_RDATA <= (others => '0');

        -- i2c slave
        when "0000001000" => -- 0x80008020 -- robot_reg[0x08]
          -- FIXME : TODO : control (from APB)
          iMST_RDATA <= (others => '0');
        when "0000001001" => -- 0x80008024 -- robot_reg[0x09]
          -- FIXME : TODO : status (from I2C master)
          iMST_RDATA <= (others => '0');
        when "0000001010" => -- 0x80008028 -- robot_reg[0x0a]
          -- FIXME : TODO : ADDR from I2C master
          iMST_RDATA <= (others => '0');
        when "0000001011" => -- 0x8000802c -- robot_reg[0x0b]
          -- FIXME : TODO : DATA from I2C master
          iMST_RDATA <= (others => '0');
        when "0000001100" => -- 0x80008030 -- robot_reg[0x0c] -- TRACE status
          iMST_RDATA <= X"0000000"&"00"&iTRACE_FIFO_EMPTY&iTRACE_FIFO_FULL;
        when "0000001101" => -- 0x80008034 -- robot_reg[0x0d]
          iMST_RDATA <= iTRACE_FIFO_DEBUG; -- TRACE write-only for APB
        when "0000001110" => -- 0x80008038 -- robot_reg[0x0e] -- BSTR status
          iMST_RDATA <= X"0000000" & "00" & iBSTR_FIFO_EMPTY & iBSTR_FIFO_FULL;
          iBSTR_FIFO_RD <= '0'; -- FIXME : TODO : refactoring (crap!..)
        when "0000001111" => -- 0x8000803c -- robot_reg[0x0f] -- BSTR FIFO rd
          iMST_RDATA <= iBSTR_FIFO;
          iBSTR_FIFO_RD <= '1';

        -- was motors in 2016
        when "0001000000" => -- 0x80008100 -- robot_reg[0x40]
          iMST_RDATA <= (others => '0');
        when "0001001000" => -- 0x80008120 -- robot_reg[0x48]
          iMST_RDATA <= (others => '0');

        -- was odometry in 2016
        when "0010000001" => -- 0x80008204 -- robot_reg[0x81]
          iMST_RDATA <= iODO_1_POS;
        when "0010000011" => -- 0x8000820c -- robot_reg[0x83]
          iMST_RDATA <= X"0000" & iODO_1_INC;
        when "0010000100" => -- 0x80008210 -- robot_reg[0x84]
          iMST_RDATA <= iODO_1_SPEED;
        when "0010000111" => -- 0x8000821c -- robot_reg[0x87]
          iMST_RDATA <= iODO_SAMPLING_INT;
        when "0010001001" => -- 0x80008224 -- robot_reg[0x89]
          iMST_RDATA <= (others => '0');
        when "0010001011" => -- 0x8000822c -- robot_reg[0x8b]
          iMST_RDATA <= (others => '0');
        when "0010001100" => -- 0x80008230 -- robot_reg[0x8c]
          iMST_RDATA <= (others => '0');
        when "0010001111" => -- 0x8000823c -- robot_reg[0x8f]
          iMST_RDATA <= (others => '0');

        -- was "advanced" odometry in 2016
        when "0010010001" => -- 0x80008244 -- robot_reg[0x91]
          iMST_RDATA <= (others => '0');
        when "0010010010" => -- 0x80008248 -- robot_reg[0x92]
          iMST_RDATA <= (others => '0');
        when "0010010011" => -- 0x8000824c -- robot_reg[0x93]
          iMST_RDATA <= (others => '0');
        when "0010010100" => -- 0x80008250 -- robot_reg[0x94]
          iMST_RDATA <= (others => '0');
        when "0010010111" => -- 0x8000825c -- robot_reg[0x97]
          iMST_RDATA <= (others => '0');
        when "0010011001" => -- 0x80008264 -- robot_reg[0x99]
          iMST_RDATA <= (others => '0');
        when "0010011010" => -- 0x80008268 -- robot_reg[0x9a]
          iMST_RDATA <= (others => '0');
        when "0010011011" => -- 0x8000826c -- robot_reg[0x9b]
          iMST_RDATA <= (others => '0');
        when "0010011100" => -- 0x80008270 -- robot_reg[0x9c]
          iMST_RDATA <= (others => '0');
        when "0010011111" => -- 0x8000827c -- robot_reg[0x9f]
          iMST_RDATA <= (others => '0');

        -- was "advanced" odometry in 2016
        when "0010100001" => -- 0x80008284 -- robot_reg[0xa1]
          iMST_RDATA <= (others => '0');
        when "0010100010" => -- 0x80008288 -- robot_reg[0xa2]
          iMST_RDATA <= (others => '0');
        when "0010100011" => -- 0x8000828c -- robot_reg[0xa3]
          iMST_RDATA <= (others => '0');
        when "0010100100" => -- 0x80008290 -- robot_reg[0xa4]
          iMST_RDATA <= (others => '0');
        when "0010100111" => -- 0x8000829c -- robot_reg[0xa7]
          iMST_RDATA <= (others => '0');
        when "0010101001" => -- 0x800082a4 -- robot_reg[0xa9]
          iMST_RDATA <= (others => '0');
        when "0010101010" => -- 0x800082a8 -- robot_reg[0xaa]
          iMST_RDATA <= (others => '0');
        when "0010101011" => -- 0x800082ac -- robot_reg[0xab]
          iMST_RDATA <= (others => '0');
        when "0010101100" => -- 0x800082b0 -- robot_reg[0xac]
          iMST_RDATA <= (others => '0');
        when "0010101111" => -- 0x800082bc -- robot_reg[0xaf]
          iMST_RDATA <= (others => '0');

        -- was 64bit multiplier in 2016
        when "0010110000" => -- 0x800082c0 -- robot_reg[0xb0]
          iMST_RDATA <= (others => '0');
        when "0010110001" => -- 0x800082c4 -- robot_reg[0xb1]
          iMST_RDATA <= (others => '0');
        when "0010110010" => -- 0x800082c8 -- robot_reg[0xb2]
          iMST_RDATA <= (others => '0');
        when "0010110011" => -- 0x800082cc -- robot_reg[0xb3]
          iMST_RDATA <= (others => '0');
        when "0010110100" => -- 0x800082d0 -- robot_reg[0xb4]
          iMST_RDATA <= (others => '0');
        when "0010110101" => -- 0x800082d4 -- robot_reg[0xb5]
          iMST_RDATA <= (others => '0');

        -- was sin_cos in 2016
        when "0010111000" => -- 0x800082e0 -- robot_reg[0xb8]
          iMST_RDATA <= (others => '0');
        when "0010111001" => -- 0x800082e4 -- robot_reg[0xb9]
          iMST_RDATA <= (others => '0');
        when "0010111010" => -- 0x800082e8 -- robot_reg[0xba]
          iMST_RDATA <= (others => '0');
        when "0010111011" => -- 0x800082ec -- robot_reg[0xbb]
          iMST_RDATA <= (others => '0');
        when "0010111100" => -- 0x800082f0 -- robot_reg[0xbc]
          iMST_RDATA <= (others => '0');
        when "0010111101" => -- 0x800082f4 -- robot_reg[0xbd]
          iMST_RDATA <= (others => '0');

        -- ultrasound
        when "0011000000" => -- 0x80008300 -- robot_reg[0xc0]
          iMST_RDATA <= iUS1_ACTUAL_DIST;
        when "0011000001" => -- 0x80008304 -- robot_reg[0xc1]
          iMST_RDATA <= iUS2_ACTUAL_DIST;
        when "0011000010" => -- 0x80008308 -- robot_reg[0xc2]
          iMST_RDATA <= iUS3_ACTUAL_DIST;
        when "0011000011" => -- 0x8000830c -- robot_reg[0xc3]
          iMST_RDATA <= (others => '0');
        when "0011000100" => -- 0x80008310 -- robot_reg[0xc4]
          iMST_RDATA <= (others => '0');
        when "0011000101" => -- 0x80008314 -- robot_reg[0xc5]
          iMST_RDATA <= (others => '0');
        when "0011000110" => -- 0x80008318 -- robot_reg[0xc6]
          iMST_RDATA <= (others => '0');
        when "0011000111" => -- 0x8000831c -- robot_reg[0xc7]
          iMST_RDATA <= (others => '0');

        when "0011001000" => -- 0x80008320 -- robot_reg[0xc8]
          iMST_RDATA <= (others => '0');
        when "0011001001" => -- 0x80008324 -- robot_reg[0xc9]
          iMST_RDATA <= (others => '0');
        when "0011001010" => -- 0x80008328 -- robot_reg[0xca]
          iMST_RDATA <= (others => '0');
        when "0011001011" => -- 0x8000832c -- robot_reg[0xcb]
          iMST_RDATA <= (others => '0');

        when "0011001100" => -- 0x80008330 -- robot_reg[0xcc]
          iMST_RDATA <= (others => '0');
        when "0011001101" => -- 0x80008334 -- robot_reg[0xcd]
          iMST_RDATA <= (others => '0');
        when "0011001110" => -- 0x80008338 -- robot_reg[0xce]
          iMST_RDATA <= (others => '0');
        when "0011001111" => -- 0x8000833c -- robot_reg[0xcf]
          iMST_RDATA <= (others => '0');

        -- was gps in 2016
        when "0011010000" => -- 0x80008340 -- robot_reg[0xd0]
          iMST_RDATA <= (others => '0');
        when "0011010001" => -- 0x80008344 -- robot_reg[0xd1]
          iMST_RDATA <= (others => '0');
        when "0011010010" => -- 0x80008348 -- robot_reg[0xd2]
          iMST_RDATA <= (others => '0');
        when "0011010011" => -- 0x8000834c -- robot_reg[0xd3]
          iMST_RDATA <= (others => '0');
        when "0011010100" => -- 0x80008350 -- robot_reg[0xd4]
          iMST_RDATA <= (others => '0');
        when "0011010101" => -- 0x80008354 -- robot_reg[0xd5]
          iMST_RDATA <= (others => '0');
        when "0011010110" => -- 0x80008358 -- robot_reg[0xd6]
          iMST_RDATA <= (others => '0');
        when "0011010111" => -- 0x8000835c -- robot_reg[0xd7]
          iMST_RDATA <= (others => '0');

        -- ROBOT_SPI_SLAVE : esclave SPI
        when "0011011000" => -- 0x80008360 -- robot_reg[0xd8]
          iMST_RDATA <= iSPI_DBG_SLV_DATA;
        when "0011011001" => -- 0x80008364 -- robot_reg[0xd9]
          iMST_RDATA <= iSPI_DBG_MST_DATA;
        when "0011011010" => -- 0x80008368 -- robot_reg[0xda]
          iMST_RDATA <= (others => '0');
        when "0011011011" => -- 0x8000836c -- robot_reg[0xdb]
          iMST_RDATA <= (others => '0');
        when "0011011100" => -- 0x80008370 -- robot_reg[0xdc]
          iMST_RDATA <= (others => '0');
        when "0011011101" => -- 0x80008374 -- robot_reg[0xdd]
          iMST_RDATA <= (others => '0');
        when "0011011110" => -- 0x80008378 -- robot_reg[0xde]
          iMST_RDATA <= (others => '0');
        when "0011011111" => -- 0x8000837c -- robot_reg[0xdf]
          iMST_RDATA <= (others => '0');

        -- SERVO PETIT ROBOT 2018
        when "0100000000" => -- 0x80008400 -- robot_reg[0x100]
          iMST_RDATA <= iSERVO0_PWM_PERIOD;
        when "0100000001" => -- 0x80008404 -- robot_reg[0x101]
          iMST_RDATA <= iSERVO0_PW;
        when "0100000010" => -- 0x80008408 -- robot_reg[0x102]
          iMST_RDATA <= iSERVO1_PWM_PERIOD;
        when "0100000011" => -- 0x8000840c -- robot_reg[0x103]
          iMST_RDATA <= iSERVO1_PW;
        when "0100000100" => -- 0x80008410 -- robot_reg[0x104]
          iMST_RDATA <= iSERVO2_PWM_PERIOD;
        when "0100000101" => -- 0x80008414 -- robot_reg[0x105]
          iMST_RDATA <= iSERVO2_PW;
        when "0100000110" => -- 0x80008418 -- robot_reg[0x106]
          iMST_RDATA <= iSERVO3_PWM_PERIOD;
        when "0100000111" => -- 0x8000841c -- robot_reg[0x107]
          iMST_RDATA <= iSERVO3_PW;
        when "0100001000" => -- 0x80008420 -- robot_reg[0x108]
          iMST_RDATA <= iSERVO4_PWM_PERIOD;
        when "0100001001" => -- 0x80008424 -- robot_reg[0x109]
          iMST_RDATA <= iSERVO4_PW;
        when "0100001010" => -- 0x80008428 -- robot_reg[0x10a]
          iMST_RDATA <= iSERVO5_PWM_PERIOD;
        when "0100001011" => -- 0x8000842c -- robot_reg[0x10b]
          iMST_RDATA <= iSERVO5_PW;
        when "0100001100" => -- 0x80008430 -- robot_reg[0x10c]
          iMST_RDATA <= iSERVO6_PWM_PERIOD;
        when "0100001101" => -- 0x80008434 -- robot_reg[0x10d]
          iMST_RDATA <= iSERVO6_PW;
        when "0100001110" => -- 0x80008438 -- robot_reg[0x10e]
          iMST_RDATA <= iSERVO7_PWM_PERIOD;
        when "0100001111" => -- 0x8000843c -- robot_reg[0x10f]
          iMST_RDATA <= iSERVO7_PW;
        when "0100010000" => -- 0x80008440 -- robot_reg[0x110]
          iMST_RDATA <= iSERVO8_PWM_PERIOD;
        when "0100010001" => -- 0x80008444 -- robot_reg[0x111]
          iMST_RDATA <= iSERVO8_PW;
        when "0100010010" => -- 0x80008448 -- robot_reg[0x112]
          iMST_RDATA <= iSERVO9_PWM_PERIOD;
        when "0100010011" => -- 0x8000844c -- robot_reg[0x113]
          iMST_RDATA <= iSERVO9_PW;
        when "0100010100" => -- 0x80008450 -- robot_reg[0x114]
          iMST_RDATA <= iSERVO10_PWM_PERIOD;
        when "0100010101" => -- 0x80008454 -- robot_reg[0x115]
          iMST_RDATA <= iSERVO10_PW;
        when "0100010110" => -- 0x80008458 -- robot_reg[0x116]
          iMST_RDATA <= iSERVO11_PWM_PERIOD;
        when "0100010111" => -- 0x8000845c -- robot_reg[0x117]
          iMST_RDATA <= iSERVO11_PW;
        when "0100011000" => -- 0x80008460 -- robot_reg[0x118]
          iMST_RDATA <= (others => '0');
        when "0100011001" => -- 0x80008464 -- robot_reg[0x119]
          iMST_RDATA <= (others => '0');
        when "0100011010" => -- 0x80008468 -- robot_reg[0x11a]
          iMST_RDATA <= (others => '0');
        when "0100011011" => -- 0x8000846c -- robot_reg[0x11b]
          iMST_RDATA <= (others => '0');
        when "0100011100" => -- 0x80008470 -- robot_reg[0x11c]
          iMST_RDATA <= (others => '0');
        when "0100011101" => -- 0x80008474 -- robot_reg[0x11d]
          iMST_RDATA <= (others => '0');
        when "0100011110" => -- 0x80008478 -- robot_reg[0x11e]
          iMST_RDATA <= (others => '0');
        when "0100011111" => -- 0x8000847c -- robot_reg[0x11f]
          iMST_RDATA <= (others => '0');

        -- MOTEURS CC PETIT ROBOT 2018
        when "0100100000" => -- 0x80008480 -- robot_reg[0x120]
          iMST_RDATA <= iPUMP0_PWM_PERIOD;
        when "0100100001" => -- 0x80008484 -- robot_reg[0x121]
          iMST_RDATA <= iPUMP0_PW;
        when "0100100010" => -- 0x80008488 -- robot_reg[0x122]
          iMST_RDATA <= iPUMP1_PWM_PERIOD;
        when "0100100011" => -- 0x8000848c -- robot_reg[0x123]
          iMST_RDATA <= iPUMP1_PW;
        when "0100100100" => -- 0x80008490 -- robot_reg[0x124]
          iMST_RDATA <= iMOTOR2_PWM_PERIOD;
        when "0100100101" => -- 0x80008494 -- robot_reg[0x125]
          iMST_RDATA <= iMOTOR2_PW;
        when "0100100110" => -- 0x80008498 -- robot_reg[0x126]
          iMST_RDATA <= (others => '0');
        when "0100100111" => -- 0x8000849c -- robot_reg[0x127]
          iMST_RDATA <= (others => '0');
        when "0100101000" => -- 0x800084a0 -- robot_reg[0x128]
          iMST_RDATA <= (others => '0');
        when "0100101001" => -- 0x800084a4 -- robot_reg[0x129]
          iMST_RDATA <= (others => '0');
        when "0100101010" => -- 0x800084a8 -- robot_reg[0x12a]
          iMST_RDATA <= (others => '0');
        when "0100101011" => -- 0x800084ac -- robot_reg[0x12b]
          iMST_RDATA <= (others => '0');
        when "0100101100" => -- 0x800084b0 -- robot_reg[0x12c]
          iMST_RDATA <= (others => '0');
        when "0100101101" => -- 0x800084b4 -- robot_reg[0x12d]
          iMST_RDATA <= (others => '0');
        when "0100101110" => -- 0x800084b8 -- robot_reg[0x12e]
          iMST_RDATA <= (others => '0');
        when "0100101111" => -- 0x800084bc -- robot_reg[0x12f]
          iMST_RDATA <= (others => '0');

        -- MOTEURS STEPPER PETIT ROBOT 2018
        when "0100110000" => -- 0x800084c0 -- robot_reg[0x130]
          iMST_RDATA <= iSTP_POL_0_PERIOD & iSTP_POL_0_CTRL;
        when "0100110001" => -- 0x800084c4 -- robot_reg[0x131]
          iMST_RDATA <= iSTP_POL_0_CUR_POS & iSTP_POL_0_TARGET_POS;
        when "0100110010" => -- 0x800084c8 -- robot_reg[0x132]
          iMST_RDATA <= iSTP_POL_1_PERIOD & iSTP_POL_1_CTRL;
        when "0100110011" => -- 0x800084cc -- robot_reg[0x133]
          iMST_RDATA <= iSTP_POL_1_CUR_POS & iSTP_POL_1_TARGET_POS;
        when "0100110100" => -- 0x800084d0 -- robot_reg[0x134]
          iMST_RDATA <= (others => '0');
        when "0100110101" => -- 0x800084d4 -- robot_reg[0x135]
          iMST_RDATA <= (others => '0');
        when "0100110110" => -- 0x800084d8 -- robot_reg[0x136]
          iMST_RDATA <= (others => '0');
        when "0100110111" => -- 0x800084dc -- robot_reg[0x137]
          iMST_RDATA <= (others => '0');

        -- BAL & capteurs PETIT ROBOT 2018
        when "0100111000" => -- 0x800084e0 -- robot_reg[0x138]
          iMST_RDATA <= X"0000000" & "00" & stp_switch1 & stp_switch0;
        -- GPIO Audran PR 2019
        when "0100111001" => -- 0x800084e4 -- robot_reg[0x139]
          iMST_RDATA <= gpio_in;
        when "0100111010" => -- 0x800084e8 -- robot_reg[0x13a]
          iMST_RDATA <= (others => '0');
        when "0100111011" => -- 0x800084ec -- robot_reg[0x13b]
          iMST_RDATA <= (others => '0');
        when "0100111100" => -- 0x800084f0 -- robot_reg[0x13c]
          iMST_RDATA <= iROBOT2018_BAL0;
        when "0100111101" => -- 0x800084f4 -- robot_reg[0x13d]
          iMST_RDATA <= iROBOT2018_BAL1;
        when "0100111110" => -- 0x800084f8 -- robot_reg[0x13e]
          iMST_RDATA <= iROBOT2018_BAL2;
        when "0100111111" => -- 0x800084fc -- robot_reg[0x13f]
          iMST_RDATA <= iROBOT2018_BAL3;

        when others =>
      end case;
    else
      iMST_RDATA <= (others => '1');
      iBSTR_FIFO_RD <= '0';
    end if;
  end process;

end architecture;
