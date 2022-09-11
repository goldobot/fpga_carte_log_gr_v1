-- --========================================================================--
--
-- Module  : ADC ADC128S022 INTERFACE
--
-- ----------------------------------------------------------------------------
-- Fonction : - interface for the ADC128S022 ADC from TI
--
-- --========================================================================--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

-- ----------------------------------------------------------------------------

entity ADC128S022 is
  port (
    -- reset & clock
    RESET              : in std_logic;
    CLK                : in std_logic;

    -- internal interface
    ADC_DATA0          : out std_logic_vector (15 downto 0);
    ADC_DATA1          : out std_logic_vector (15 downto 0);
    ADC_DATA2          : out std_logic_vector (15 downto 0);
    ADC_DATA3          : out std_logic_vector (15 downto 0);
    ADC_DATA4          : out std_logic_vector (15 downto 0);
    ADC_DATA5          : out std_logic_vector (15 downto 0);
    ADC_DATA6          : out std_logic_vector (15 downto 0);
    ADC_DATA7          : out std_logic_vector (15 downto 0);

    -- adc ADC128S022 interface 
    ADC_CS             : out std_logic;
    ADC_SCLK           : out std_logic;
    ADC_DIN            : out std_logic;
    ADC_DOUT           : in std_logic
  );
end ADC128S022;

-- --============================ ARCHITECTURE ==============================--

architecture arch of ADC128S022 is

-- ----------------------------------------------------------------------------
-- Component declarations
-- ----------------------------------------------------------------------------


-- ----------------------------------------------------------------------------
-- Constant declarations
-- ----------------------------------------------------------------------------

-- States of the acquire SM
constant ST_ACQ_IDDLE           : std_logic_vector(4 downto 0) := "00000";
constant ST_ACQ_BEGIN           : std_logic_vector(4 downto 0) := "00001";
constant ST_ACQ_SCLK_LOW        : std_logic_vector(4 downto 0) := "00010";
constant ST_ACQ_SCLK_HIGH       : std_logic_vector(4 downto 0) := "00011";
constant ST_ACQ_END             : std_logic_vector(4 downto 0) := "00100";


-- ----------------------------------------------------------------------------
-- Signal declarations
-- ----------------------------------------------------------------------------
-- State Vector for Acquire SM
signal iAcquireState        : std_logic_vector(4 downto 0);

-- Holds the received value
signal iAdcLatch0           : std_logic_vector(15 downto 0);
signal iAdcLatch1           : std_logic_vector(15 downto 0);
signal iAdcLatch2           : std_logic_vector(15 downto 0);
signal iAdcLatch3           : std_logic_vector(15 downto 0);
signal iAdcLatch4           : std_logic_vector(15 downto 0);
signal iAdcLatch5           : std_logic_vector(15 downto 0);
signal iAdcLatch6           : std_logic_vector(15 downto 0);
signal iAdcLatch7           : std_logic_vector(15 downto 0);

-- Input serial register
signal iAdcSR               : std_logic_vector(15 downto 0);

-- Command serial register
signal iAdcCMD              : std_logic_vector(15 downto 0);

-- Bit count for serial receive
signal iSampleCount         : std_logic_vector(4 downto 0);

-- Bit count for channel selector
signal iChanCount            : std_logic_vector(2 downto 0);

-- Clock divider (divide CLK by DIV(from 1 to 255))
signal iClkDivReg           : std_logic_vector(15 downto 0);


begin

-- ----------------------------------------------------------------------------
-- Combinatorial logic
-- ----------------------------------------------------------------------------
ADC_DATA0 <= iAdcLatch0;
ADC_DATA1 <= iAdcLatch1;
ADC_DATA2 <= iAdcLatch2;
ADC_DATA3 <= iAdcLatch3;
ADC_DATA4 <= iAdcLatch4;
ADC_DATA5 <= iAdcLatch5;
ADC_DATA6 <= iAdcLatch6;
ADC_DATA7 <= iAdcLatch7;

ADC_DIN <= iAdcCMD(15);

-- purpose: gets measure data from ADC128S022
-- type   : sequential
-- inputs : CLK, RESET, ADC_DOUT
-- outputs: ADC_CS, ADC_SCLK
p_AdcAcquire: process (CLK, RESET)
begin  -- process p_AdcAcquire
  if RESET = '1' then
    ADC_CS <= '1';
    ADC_SCLK <= '1';
    iAdcSR <= (others => '0');
    iAdcCMD <= (others => '0');
    iAdcLatch0 <= (others => '0');
    iAdcLatch1 <= (others => '0');
    iAdcLatch2 <= (others => '0');
    iAdcLatch3 <= (others => '0');
    iAdcLatch4 <= (others => '0');
    iAdcLatch5 <= (others => '0');
    iAdcLatch6 <= (others => '0');
    iAdcLatch7 <= (others => '0');
    iSampleCount <= (others => '0');
    iAcquireState <= ST_ACQ_IDDLE;
    iClkDivReg <= (others => '0');
  elsif CLK'event and CLK = '1' then
--    if iClkDivReg(7 downto 0) = X"00" then
    if iClkDivReg(2 downto 0) = "000" then
      case iAcquireState is
        when ST_ACQ_IDDLE          =>
--          if iClkDivReg = X"0800" then
          if iClkDivReg(7 downto 0) = "00000000" then
            ADC_CS <= '0';
            iAdcSR <= (others => '0');
            -- FIXME : DEBUG : CM:ADC_IN_1 => DE0_NANO:ANALOG_IN4
            --iAdcCMD <= X"2000";
            -- FIXME : DEBUG : CM:ADC_IN_2 => DE0_NANO:ANALOG_IN2
            --iAdcCMD <= X"1000";
            -- FIXME : TODO : TEST
            iAdcCMD <= "00"& iChanCount & "000" & X"00";
            iSampleCount <= (others => '0');
            iAcquireState <= ST_ACQ_BEGIN;
          end if;
        when ST_ACQ_BEGIN          =>
          ADC_SCLK <= '0';
          iSampleCount <= iSampleCount+1;
          iAcquireState <= ST_ACQ_SCLK_LOW;
        when ST_ACQ_SCLK_LOW       =>
          ADC_SCLK <= '1';
          iAdcSR <= iAdcSR(14 downto 0) & ADC_DOUT;
          iAdcCMD <= iAdcCMD(14 downto 0) & '0';
          iAcquireState <= ST_ACQ_SCLK_HIGH;
        when ST_ACQ_SCLK_HIGH      =>
          if iSampleCount = "10000" then
--            iAdcLatch <= '0' & iAdcSR(15 downto 1);
            case iChanCount is
              when "000" =>
                iAdcLatch7 <= iAdcSR;
              when "001" =>
                iAdcLatch0 <= iAdcSR;
              when "010" =>
                iAdcLatch1 <= iAdcSR;
              when "011" =>
                iAdcLatch2 <= iAdcSR;
              when "100" =>
                iAdcLatch3 <= iAdcSR;
              when "101" =>
                iAdcLatch4 <= iAdcSR;
              when "110" =>
                iAdcLatch5 <= iAdcSR;
              when "111" =>
                iAdcLatch6 <= iAdcSR;
              when others =>
                null;
            end case;
            iChanCount <= iChanCount+1;
            iAcquireState <= ST_ACQ_END;
          else
            ADC_SCLK <= '0';
            iSampleCount <= iSampleCount+1;
            iAcquireState <= ST_ACQ_SCLK_LOW;
          end if;
        when ST_ACQ_END            =>
          ADC_CS <= '1';
          iAcquireState <= ST_ACQ_IDDLE;
        when others =>
          null;
      end case;
    end if;

    iClkDivReg <= iClkDivReg+1;

  end if;
end process p_AdcAcquire;

end arch;

-- --================================= End ==================================--

