-- --========================================================================--
--
-- Module  : STEPPER_POLOLU
--
-- ----------------------------------------------------------------------------
-- Fonction : - Pulse & dir generator for Pololu driver for stepper motor
--
-- --========================================================================--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity STEPPER_POLOLU is
  port (
    -- reset & clock
    RESET          : in std_logic;
    CLK            : in std_logic;

    -- internal interface
    CTRL           : in std_logic_vector (15 downto 0);
    PERIOD         : in std_logic_vector (15 downto 0);
    TARGET_POS     : in std_logic_vector (15 downto 0);
    SET_CUR_POS    : in std_logic_vector (15 downto 0);
    CUR_POS        : out std_logic_vector (15 downto 0);

    -- external interface
    HIGH_SW        : in std_logic;
    LOW_SW         : in std_logic;
    N_ENABLE       : out std_logic;
    STEP           : out std_logic;
    DIR            : out std_logic
  );
end STEPPER_POLOLU;


-- --============================ ARCHITECTURE ==============================--

architecture arch of STEPPER_POLOLU is

-- ----------------------------------------------------------------------------
-- Component declarations
-- ----------------------------------------------------------------------------


-- ----------------------------------------------------------------------------
-- Constant declarations
-- ----------------------------------------------------------------------------

constant ST_IDDLE           : std_logic_vector(5 downto 0) := "000000";
constant ST_PULSE_LOW_SETUP : std_logic_vector(5 downto 0) := "000001";
constant ST_PULSE_HIGH      : std_logic_vector(5 downto 0) := "000010";
constant ST_PULSE_LOW       : std_logic_vector(5 downto 0) := "000011";


-- ----------------------------------------------------------------------------
-- Signal declarations
-- ----------------------------------------------------------------------------

signal iPULSE_STATE         : std_logic_vector (5 downto 0);
signal iCTRL                : std_logic_vector (15 downto 0);
signal iCUR_POS             : std_logic_vector (15 downto 0);
signal iTARGET_POS          : std_logic_vector (15 downto 0);
signal iSET_CUR_POS         : std_logic_vector (15 downto 0);
signal iSTEP                : std_logic;
signal iDIR                 : std_logic;

signal iPULSE_PERIOD        : std_logic_vector (31 downto 0);
signal iPULSE_WIDTH         : std_logic_vector (31 downto 0);

signal iPULSE_PERIOD_CNT    : std_logic_vector (31 downto 0);
signal iPULSE_WIDTH_CNT     : std_logic_vector (31 downto 0);

signal iSET_CUR_POS_FLAG    : std_logic;

signal iHIGH_SW             : std_logic;
signal iLOW_SW              : std_logic;

begin

CUR_POS <= iCUR_POS;

N_ENABLE <= not iCTRL(0);
STEP <= iSTEP;
DIR <= iDIR;

p_PulseSM : process (CLK, RESET)
begin
  if (RESET = '1') then
    iPULSE_STATE      <= ST_IDDLE;
    iCTRL             <= (others => '0');
    iCUR_POS          <= X"4000";
    iTARGET_POS       <= X"4000";
    iSET_CUR_POS      <= X"4000";
    iPULSE_PERIOD_CNT <= (others => '0');
    iPULSE_WIDTH_CNT  <= (others => '0');
    iSTEP <= '0';
    iDIR  <= '0';

    iPULSE_PERIOD <= X"0" & X"0010" & X"000";
    iPULSE_WIDTH  <= X"00001000";

    iSET_CUR_POS_FLAG <= '0';

    iHIGH_SW <= '0';
    iLOW_SW  <= '0';

  elsif (CLK'event and CLK = '1') then
    iTARGET_POS <= TARGET_POS;
    iSET_CUR_POS <= SET_CUR_POS;
    iCTRL <= CTRL;

    iPULSE_PERIOD <=  X"0" & PERIOD & X"000";
    iPULSE_WIDTH  <= X"00001000";

    iSET_CUR_POS_FLAG <= CTRL(1);

    case iPULSE_STATE is
      when ST_IDDLE =>
        iSTEP <= '0';

        if ((iSET_CUR_POS_FLAG = '1')) then
          iCUR_POS <= iSET_CUR_POS;
        end if;

        if ((iTARGET_POS /= X"FFFF") and (iCUR_POS > iTARGET_POS)) then
          iCUR_POS <= iCUR_POS - 1;
          iDIR  <= '0';
          iPULSE_PERIOD_CNT <= iPULSE_PERIOD;
          iPULSE_WIDTH_CNT  <= X"00000010";
          iPULSE_STATE <= ST_PULSE_LOW_SETUP;
        end if;

        if ((iTARGET_POS /= X"FFFF") and (iCUR_POS < iTARGET_POS)) then
          iCUR_POS <= iCUR_POS + 1;
          iDIR  <= '1';
          iPULSE_PERIOD_CNT <= iPULSE_PERIOD;
          iPULSE_WIDTH_CNT  <= X"00000010";
          iPULSE_STATE <= ST_PULSE_LOW_SETUP;
        end if;

      when ST_PULSE_LOW_SETUP =>
        iSTEP <= '0';
        if (iPULSE_WIDTH_CNT = X"00000000") then
          iPULSE_WIDTH_CNT <= iPULSE_WIDTH;
          iPULSE_STATE <= ST_PULSE_HIGH;
        elsif (iPULSE_PERIOD_CNT = X"00000000") then
          iPULSE_STATE <= ST_IDDLE;
        else
          iPULSE_PERIOD_CNT <= iPULSE_PERIOD_CNT - 1;
          iPULSE_WIDTH_CNT  <= iPULSE_WIDTH_CNT - 1;
        end if;

      when ST_PULSE_HIGH =>
        iSTEP <= '1';
        if (iPULSE_WIDTH_CNT = X"00000000") then
          iPULSE_STATE <= ST_PULSE_LOW;
        elsif (iPULSE_PERIOD_CNT = X"00000000") then
          iPULSE_STATE <= ST_IDDLE;
        else
          iPULSE_PERIOD_CNT <= iPULSE_PERIOD_CNT - 1;
          iPULSE_WIDTH_CNT  <= iPULSE_WIDTH_CNT - 1;
        end if;

      when ST_PULSE_LOW =>
        iSTEP <= '0';
        if (iPULSE_PERIOD_CNT = X"00000000") then
          iPULSE_STATE <= ST_IDDLE;
        else
          iPULSE_PERIOD_CNT <= iPULSE_PERIOD_CNT - 1;
        end if;

      when others =>
        null;
    end case;

    iHIGH_SW <= HIGH_SW;
    iLOW_SW  <= LOW_SW;
  end if;
end process p_PulseSM;

end arch;

-- --================================= End ==================================--

