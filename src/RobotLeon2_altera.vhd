library ieee;
use ieee.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned."+";
use IEEE.std_logic_unsigned."-";
use IEEE.std_logic_unsigned.conv_integer;
library work;

library unisim;
-- use unisim.vcomponents.all;

entity RobotLeon2_altera is
  port(
    -- reset & clock
    N_RESET               : in std_logic
    ; CLK                 : in std_logic

    -- UART
    ; UART1_RX            : in std_logic
    ; UART1_TX            : out std_logic

    -- SLAVE SPI
    ; SLV_SPI1_SCK        : in std_logic
    ; SLV_SPI1_MISO       : out std_logic
    ; SLV_SPI1_MOSI       : in std_logic

    -- PWM SERVO
    ; PWM_SERVO0          : out std_logic
    ; PWM_SERVO1          : out std_logic
    ; PWM_SERVO2          : out std_logic
    ; PWM_SERVO3          : out std_logic
    ; PWM_SERVO4          : out std_logic
    ; PWM_SERVO5          : out std_logic
-- FIXME : TODO : trouver un autre pin
--    ; PWM_SERVO6        : out std_logic
    ; PWM_SERVO7          : out std_logic
    ; PWM_SERVO8          : out std_logic
    ; PWM_SERVO9          : out std_logic
    ; PWM_SERVO10         : out std_logic
    ; PWM_SERVO11         : out std_logic

    -- PWM POMPE & MOTERUS
    ; PWM_POMPE0          : out std_logic
    ; DIR_POMPE0          : out std_logic
    ; PWM_POMPE1          : out std_logic
    ; DIR_POMPE1          : out std_logic
    ; PWM_MOTOR2          : out std_logic
    ; DIR_MOTOR2          : out std_logic

    -- STEPPER
    ; STP4_PWM            : out std_logic
    ; STP4_DIR            : out std_logic
--    ; STP5_PWM            : out std_logic -- FIXME : TODO
--    ; STP5_DIR            : out std_logic -- FIXME : TODO
    ; STP_SWITCH0         : in std_logic
    ; STP_SWITCH1         : in std_logic

    -- Encodeur tapis roulant
    ; CONVEYOR_ENC_CH_A   : in std_logic
    ; CONVEYOR_ENC_CH_B   : in std_logic

    -- GPIOs
    ; GPIO_0_IN0          : in std_logic

    ; GPIO_0_IN1          : in std_logic

    ; GPIO_003            : in std_logic

    ; GPIO_005            : in std_logic

    ; GPIO_014            : in std_logic
    ; GPIO_015            : in std_logic

    ; GPIO_018            : in std_logic
    ; GPIO_019            : in std_logic
    ; GPIO_020            : in std_logic
    ; GPIO_021            : in std_logic
    ; GPIO_022            : in std_logic
    ; GPIO_023            : in std_logic
    ; GPIO_024            : in std_logic
    ; GPIO_025            : in std_logic
    ; GPIO_026            : in std_logic
    ; GPIO_027            : in std_logic
    ; GPIO_028            : in std_logic
    ; GPIO_029            : in std_logic
    ; GPIO_030            : in std_logic

    ; GPIO_032            : in std_logic
    ; GPIO_033            : in std_logic

    ; GPIO_1_IN0          : in std_logic
    ; GPIO_100            : in std_logic
    ; GPIO_1_IN1          : in std_logic
    ; GPIO_101            : in std_logic
    ; GPIO_102            : in std_logic
    ; GPIO_103            : in std_logic
    ; GPIO_104            : in std_logic
    ; GPIO_105            : in std_logic
    ; GPIO_106            : in std_logic
    ; GPIO_107            : in std_logic
    ; GPIO_108            : in std_logic
    ; GPIO_109            : in std_logic
    ; GPIO_110            : in std_logic
    ; GPIO_111            : in std_logic
    ; GPIO_112            : in std_logic
    ; GPIO_113            : in std_logic
    ; GPIO_114            : in std_logic

    ; GPIO_116            : in std_logic

    ; GPIO_118            : in std_logic

    ; GPIO_120            : in std_logic
    ; GPIO_121            : in std_logic
    ; GPIO_122            : in std_logic
    ; GPIO_123            : in std_logic
    ; GPIO_124            : in std_logic
    ; GPIO_125            : in std_logic
    ; GPIO_126            : in std_logic
    ; GPIO_127            : in std_logic
    ; GPIO_128            : in std_logic
    ; GPIO_129            : in std_logic
    ; GPIO_130            : in std_logic
    ; GPIO_131            : in std_logic
    ; GPIO_132            : in std_logic
    ; GPIO_133            : in std_logic

    ; GPIO_2_IN0          : in std_logic
    ; GPIO_2_IN1          : in std_logic
    ; GPIO_2_IN2          : in std_logic
    ; GPIO_200            : in std_logic
    ; GPIO_201            : in std_logic
    ; GPIO_202            : in std_logic
    ; GPIO_203            : in std_logic
    ; GPIO_204            : in std_logic
    ; GPIO_205            : in std_logic
    ; GPIO_206            : in std_logic
    ; GPIO_207            : in std_logic
    ; GPIO_208            : in std_logic
    -- GPIO_209 = ALTERA_nCEO
    ; GPIO_210            : in std_logic
    ; GPIO_211            : in std_logic
    ; GPIO_212            : in std_logic

    -- LEDs
    ; LED_DEBUG_0         : out std_logic
    ; LED_DEBUG_1         : out std_logic
    ; LED_DEBUG_2         : out std_logic
    ; LED_DEBUG_3         : out std_logic
    ; LED_DEBUG_4         : out std_logic
    ; LED_DEBUG_5         : out std_logic
    ; LED_DEBUG_6         : out std_logic
    ; LED_DEBUG_7         : out std_logic

    -- Switches
    ; DIP_SW_0            : in std_logic
    ; DIP_SW_1            : in std_logic
    ; DIP_SW_2            : in std_logic
    ; DIP_SW_3            : in std_logic
    );
end entity;


architecture rtl of RobotLeon2_altera is

component core
  port (
    -- Systeme
    n_reset_i           : in std_logic;
    clk_i               : in std_logic;
    clk_i_100           : in std_logic;

    -- serial interface
    rx                  : in  std_logic;
    tx                  : out std_logic;
    drx                 : in  std_logic;
    dtx                 : out std_logic;

    -- i2c master
    i2c_mst_sda_i       : in  std_logic;
    i2c_mst_sda_o       : out std_logic;
    i2c_mst_sda_en      : out std_logic;
    i2c_mst_scl_i       : in  std_logic;
    i2c_mst_scl_o       : out std_logic;
    i2c_mst_scl_en      : out std_logic;

    -- i2c slave
    i2c_slv_sda_i       : in  std_logic;
    i2c_slv_sda_o       : out std_logic;
    i2c_slv_sda_en      : out std_logic;
    i2c_slv_scl_i       : in  std_logic;
    i2c_slv_scl_o       : out std_logic;
    i2c_slv_scl_en      : out std_logic;

    -- spi master
    sck                 : out std_logic;
    mosi                : out std_logic;
    miso                : in  std_logic;

    -- spi slave
    slv_cs              : in  std_logic;
    slv_clk             : in  std_logic;
    slv_mosi            : in  std_logic;
    slv_miso            : out std_logic;

    -- ROBOT
    -- hcsr04 interfaces
    us1_trig            : out std_logic;
    us1_echo            : in std_logic;
    us2_trig            : out std_logic;
    us2_echo            : in std_logic;
    us3_trig            : out std_logic;
    us3_echo            : in std_logic;

    -- misc actuator interfaces
    pwm_servo0          : out std_logic;
    pwm_servo1          : out std_logic;
    pwm_servo2          : out std_logic;
    pwm_servo3          : out std_logic;
    pwm_servo4          : out std_logic;
    pwm_servo5          : out std_logic;
    pwm_servo6          : out std_logic;
    pwm_servo7          : out std_logic;
    pwm_servo8          : out std_logic;
    pwm_servo9          : out std_logic;
    pwm_servo10         : out std_logic;
    pwm_servo11         : out std_logic;
    pwm_pump0           : out std_logic;
    dir_pump0           : out std_logic;
    pwm_pump1           : out std_logic;
    dir_pump1           : out std_logic;
    pwm_motor2          : out std_logic;
    dir_motor2          : out std_logic;
    stp_0_step          : out std_logic;
    stp_0_dir           : out std_logic;
    stp_1_step          : out std_logic;
    stp_1_dir           : out std_logic;
    stp_switch0         : in std_logic;
    stp_switch1         : in std_logic;

    -- LEDS
    leds                : out std_logic_vector(7 downto 0);

    -- debug/test
    debug_test          : out std_logic_vector(31 downto 0)
    );
end component;

component my_altera_pll
  port (
    inclk0 : in std_logic := '0';
    c0     : out std_logic;
    c1     : OUT std_logic
    );
end component;

signal i2c_mst_sda_i    : std_logic;
signal i2c_mst_sda_o    : std_logic;
signal i2c_mst_sda_en   : std_logic;
signal i2c_mst_scl_i    : std_logic;
signal i2c_mst_scl_o    : std_logic;
signal i2c_mst_scl_en   : std_logic;

signal i2c_slv_sda_i    : std_logic;
signal i2c_slv_sda_o    : std_logic;
signal i2c_slv_sda_en   : std_logic;
signal i2c_slv_scl_i    : std_logic;
signal i2c_slv_scl_o    : std_logic;
signal i2c_slv_scl_en   : std_logic;

signal core_leds        : std_logic_vector(7 downto 0);
signal top_leds         : std_logic_vector(7 downto 0);

signal n_reset_i        : std_logic;

signal rx1,rx2          : std_logic;
signal tx1,tx2          : std_logic;

signal clk_o            : std_logic;
signal clk_100mhz       : std_logic;

signal debug_test_out   : std_logic;
signal debug_test       : std_logic_vector(31 downto 0);

signal iSLV_SPI1_MISO   : std_logic;

signal iDEBUG_SPI       : std_logic;

begin

  -- reset management
  n_reset_i <= N_RESET;

  -- PLL
  my_pll : my_altera_pll
    port map(
      inclk0 => CLK
      , c0   => clk_o
      , c1   => clk_100mhz
      );

  -- UARTs
  rx1 <= UART1_RX;
  UART1_TX <= tx1;

  -- FIXME : TODO : UART2
  -- rx2<= drx;
  rx2 <= '1';
  -- dtx<= tx2;
  
  --core instantiation
  core_inst : core
    port map(
      n_reset_i     => n_reset_i
      , clk_i       => clk_o
      , clk_i_100   => clk_100mhz

      -- serial interface
      , rx          => rx1
      , tx          => tx1
      , drx         => rx2
      , dtx         => tx2

      -- i2c master
      -- FIXME : TODO
      -- , i2c_mst_sda_i  => i2c_mst_sda_i
      -- , i2c_mst_sda_o  => i2c_mst_sda_o
      -- , i2c_mst_sda_en => i2c_mst_sda_en
      -- , i2c_mst_scl_i  => i2c_mst_scl_i
      -- , i2c_mst_scl_o  => i2c_mst_scl_o
      -- , i2c_mst_scl_en => i2c_mst_scl_en
      , i2c_mst_sda_i  => '0'
      , i2c_mst_sda_o  => open
      , i2c_mst_sda_en => open
      , i2c_mst_scl_i  => '0'
      , i2c_mst_scl_o  => open
      , i2c_mst_scl_en => open

      -- i2c slave
      -- FIXME : TODO
      -- , i2c_slv_sda_i  => i2c_slv_sda_i
      -- , i2c_slv_sda_o  => i2c_slv_sda_o
      -- , i2c_slv_sda_en => i2c_slv_sda_en
      -- , i2c_slv_scl_i  => i2c_slv_scl_i
      -- , i2c_slv_scl_o  => i2c_slv_scl_o
      -- , i2c_slv_scl_en => i2c_slv_scl_en
      , i2c_slv_sda_i  => '0'
      , i2c_slv_sda_o  => open
      , i2c_slv_sda_en => open
      , i2c_slv_scl_i  => '0'
      , i2c_slv_scl_o  => open
      , i2c_slv_scl_en => open

      -- spi
      -- FIXME : TODO
      -- , sck      => sck
      -- , mosi     => mosi
      -- , miso     => miso
      , sck         => open
      , mosi        => open
      , miso        => '1'

      -- spi slave
      , slv_cs      => '0' -- FIXME : TODO
      , slv_clk     => SLV_SPI1_SCK
      , slv_mosi    => SLV_SPI1_MOSI
      , slv_miso    => iSLV_SPI1_MISO

      -- ROBOT
      -- hcsr04 interfaces
      -- FIXME : TODO
      , us1_trig => open
      , us1_echo => '0'
      , us2_trig => open
      , us2_echo => '0'
      , us3_trig => open
      , us3_echo => '0'

      -- misc actuator interfaces

      -- SERVOS
      , pwm_servo0   => PWM_SERVO0
      , pwm_servo1   => PWM_SERVO1
      , pwm_servo2   => PWM_SERVO2
      , pwm_servo3   => PWM_SERVO3
      , pwm_servo4   => PWM_SERVO4
      , pwm_servo5   => PWM_SERVO5
-- FIXME : TODO
--      , pwm_servo6   => PWM_SERVO2
      , pwm_servo6   => open
      , pwm_servo7   => PWM_SERVO7
      , pwm_servo8   => PWM_SERVO8
      , pwm_servo9   => PWM_SERVO9
      , pwm_servo10  => PWM_SERVO10
      , pwm_servo11  => PWM_SERVO11

      -- POMPES
      , pwm_pump0    => PWM_POMPE0
      , dir_pump0    => DIR_POMPE0
      , pwm_pump1    => PWM_POMPE1
      , dir_pump1    => DIR_POMPE1
      , pwm_motor2   => PWM_MOTOR2
      , dir_motor2   => DIR_MOTOR2

      -- STEPPERS
      , stp_0_step   => STP4_PWM
      , stp_0_dir    => STP4_DIR
      , stp_1_step   => open -- FIXME : TODO
      , stp_1_dir    => open -- FIXME : TODO
      , stp_switch0  => STP_SWITCH0
      , stp_switch1  => STP_SWITCH1

      -- LEDS
      , leds        => core_leds

      -- debug/test
      , debug_test  => debug_test
      );

-- FIXME : TODO ++
-- disble this when I2C master enabled
  i2c_mst_sda_o <= '0';
  i2c_mst_sda_en <= '0';
  i2c_mst_scl_o <= '0';
  i2c_mst_scl_en <= '0';
-- FIXME : TODO --


-- FIXME : TODO ++
---- ** i2c master **
--  I2C_SDA_MASTER <= i2c_mst_sda_o when (i2c_mst_sda_en = '1') else 'Z';
--  i2c_mst_sda_i <= I2C_SDA_MASTER;
--
--  I2C_SCL_MASTER <= i2c_mst_scl_o when (i2c_mst_scl_en = '1') else 'Z';
--  i2c_mst_scl_i <= I2C_SCL_MASTER;
--
---- ** i2c slave **
--  I2C_SDA_SLAVE <= i2c_slv_sda_o when (i2c_slv_sda_en = '1') else 'Z';
--  i2c_slv_sda_i <= I2C_SDA_SLAVE;
--
--  I2C_SCL_SLAVE <= i2c_slv_scl_o when (i2c_slv_scl_en = '1') else 'Z';
--  i2c_slv_scl_i <= I2C_SCL_SLAVE;
-- FIXME : TODO --

-- Debug --
  leds_proc : process( n_reset, clk_o )
    variable counter : integer := 0;
  begin
    if ( n_reset = '0' ) then
      top_leds <= "00000001";
      counter := 0;
      iDEBUG_SPI <= '0';
    elsif rising_edge( clk_o ) then
      if ( counter = 16000000 ) then
        top_leds <= top_leds(6 downto 0) & top_leds(7);
        counter := 0;
      else
        counter := counter + 1;
      end if;
      iDEBUG_SPI <= not iDEBUG_SPI;
    end if;
  end process leds_proc;

--leds on mainboard
  LED_DEBUG_0 <= core_leds( 0 );
  LED_DEBUG_1 <= core_leds( 1 );
  LED_DEBUG_2 <= core_leds( 2 );
  LED_DEBUG_3 <= core_leds( 3 );
  LED_DEBUG_4 <= not top_leds( 4 ) when debug_test(31)='0' else debug_test_out;
  LED_DEBUG_5 <= not top_leds( 5 ) when debug_test(31)='0' else debug_test_out;
  LED_DEBUG_6 <= not top_leds( 6 ) when debug_test(31)='0' else debug_test_out;
  LED_DEBUG_7 <= not top_leds( 7 ) when debug_test(31)='0' else debug_test_out;

  -- TEST INTEGRATION carte_log_gr_v1

  debug_test_out <= GPIO_0_IN0 when (debug_test = X"80000000") else
                    GPIO_0_IN1 when (debug_test = X"80000002") else
                    GPIO_003   when (debug_test = X"80000005") else
                    GPIO_005   when (debug_test = X"80000007") else
                    GPIO_014   when (debug_test = X"80000010") else -- FAIL !
                    GPIO_015   when (debug_test = X"80000011") else
                    GPIO_018   when (debug_test = X"80000012") else
                    GPIO_019   when (debug_test = X"80000013") else
                    GPIO_020   when (debug_test = X"80000014") else
                    GPIO_021   when (debug_test = X"80000015") else
                    GPIO_022   when (debug_test = X"80000016") else
                    GPIO_023   when (debug_test = X"80000017") else
                    GPIO_024   when (debug_test = X"80000018") else
                    GPIO_025   when (debug_test = X"80000019") else
                    GPIO_026   when (debug_test = X"8000001a") else
                    GPIO_027   when (debug_test = X"8000001b") else
                    GPIO_028   when (debug_test = X"8000001c") else
                    GPIO_029   when (debug_test = X"8000001d") else
                    GPIO_030   when (debug_test = X"8000001e") else
--                    GPIO_031   when (debug_test = X"8000001f") else
                    GPIO_032   when (debug_test = X"80000020") else
                    GPIO_033   when (debug_test = X"80000021") else
                    GPIO_1_IN0 when (debug_test = X"80000022") else
                    GPIO_100   when (debug_test = X"80000023") else
                    GPIO_1_IN1 when (debug_test = X"80000024") else
                    GPIO_101   when (debug_test = X"80000025") else
                    GPIO_102   when (debug_test = X"80000026") else
--                    GPIO_103   when (debug_test = X"80000027") else
--                    GPIO_104   when (debug_test = X"80000028") else
--                    GPIO_105   when (debug_test = X"80000029") else
                    GPIO_106   when (debug_test = X"8000002a") else
                    GPIO_107   when (debug_test = X"8000002b") else
                    GPIO_108   when (debug_test = X"8000002c") else
                    GPIO_109   when (debug_test = X"8000002d") else
                    GPIO_110   when (debug_test = X"8000002e") else
                    GPIO_111   when (debug_test = X"8000002f") else -- FAIL !
                    GPIO_112   when (debug_test = X"80000030") else
                    GPIO_113   when (debug_test = X"80000031") else
                    GPIO_114   when (debug_test = X"80000032") else
--                    GPIO_115   when (debug_test = X"80000033") else
                    GPIO_116   when (debug_test = X"80000034") else
--                    GPIO_117   when (debug_test = X"80000035") else
                    GPIO_118   when (debug_test = X"80000036") else
--                    GPIO_119   when (debug_test = X"80000037") else
                    GPIO_120   when (debug_test = X"80000038") else
                    GPIO_121   when (debug_test = X"80000039") else
                    GPIO_122   when (debug_test = X"8000003a") else
                    GPIO_123   when (debug_test = X"8000003b") else -- FAIL !
                    GPIO_124   when (debug_test = X"8000003c") else
                    GPIO_125   when (debug_test = X"8000003d") else
                    GPIO_126   when (debug_test = X"8000003e") else
                    GPIO_127   when (debug_test = X"8000003f") else
                    GPIO_128   when (debug_test = X"80000040") else
                    GPIO_129   when (debug_test = X"80000041") else
                    GPIO_130   when (debug_test = X"80000042") else
                    GPIO_131   when (debug_test = X"80000043") else -- FAIL !
                    GPIO_132   when (debug_test = X"80000044") else
                    GPIO_133   when (debug_test = X"80000045") else
                    GPIO_2_IN0 when (debug_test = X"80000046") else
                    GPIO_2_IN1 when (debug_test = X"80000047") else
                    GPIO_2_IN2 when (debug_test = X"80000048") else
                    GPIO_200   when (debug_test = X"80000049") else
                    GPIO_201   when (debug_test = X"8000004a") else
                    GPIO_202   when (debug_test = X"8000004b") else
                    GPIO_203   when (debug_test = X"8000004c") else -- FAIL !
                    GPIO_204   when (debug_test = X"8000004d") else
                    GPIO_205   when (debug_test = X"8000004e") else
                    GPIO_206   when (debug_test = X"8000004f") else
                    GPIO_207   when (debug_test = X"80000050") else
                    GPIO_208   when (debug_test = X"80000051") else
                    GPIO_210   when (debug_test = X"80000052") else
                    GPIO_211   when (debug_test = X"80000053") else
                    GPIO_212   when (debug_test = X"80000054") else
                    '1';

  SLV_SPI1_MISO <= iSLV_SPI1_MISO;
--  SPIM1_SCK  <= iDEBUG_SPI;

end architecture rtl;
