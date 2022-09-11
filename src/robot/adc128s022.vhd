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
    ADC_DATA           : out std_logic_vector (15 downto 0);

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
signal iAdcLatch            : std_logic_vector(15 downto 0);

-- Input serial register
signal iAdcSR               : std_logic_vector(15 downto 0);

-- Bit count for serial receive
signal iSampleCount         : std_logic_vector(4 downto 0);

-- Clock divider (divide CLK by DIV(from 1 to 255))
signal iClkDivReg           : std_logic_vector(15 downto 0);


begin

-- ----------------------------------------------------------------------------
-- Combinatorial logic
-- ----------------------------------------------------------------------------
ADC_DATA <= iAdcLatch;

-- FIXME : DEBUG
ADC_DIN <= '0';

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
    iAdcLatch <= (others => '0');
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
          iAcquireState <= ST_ACQ_SCLK_HIGH;
        when ST_ACQ_SCLK_HIGH      =>
          if iSampleCount = "10000" then
            -- FIXME : TODO : ADC_DOUT <= '0';
            iAdcLatch <= '0' & iAdcSR(15 downto 1);
            iAcquireState <= ST_ACQ_END;
          else
            ADC_SCLK <= '0';
            iSampleCount <= iSampleCount+1;
            iAcquireState <= ST_ACQ_SCLK_LOW;
          end if;
        when ST_ACQ_END            =>
          -- FIXME : TODO : ADC_DOUT <= '1';
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

