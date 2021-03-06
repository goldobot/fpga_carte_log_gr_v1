----------------------------------------------------------------------------
---- robot_spi_slave : esclave SPI                                      ----
----------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned."+";
use IEEE.std_logic_unsigned."-";
use IEEE.std_logic_unsigned.conv_integer;
use IEEE.numeric_std.all;

entity robot_spi_slave is
  port (
    CLK                 : in std_logic;
    RESET               : in std_logic;
    SPI_MASTER_RD       : out std_logic;
    SPI_MASTER_WR       : out std_logic;
    SPI_MASTER_ADDR     : out std_logic_vector(31 downto 0);
    SPI_MASTER_DATA     : out std_logic_vector(31 downto 0);
    SPI_SLAVE_DATA      : in std_logic_vector(31 downto 0);
    SPI_SLAVE_ACK       : in std_logic;
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
end robot_spi_slave;

architecture robot_spi_slave_arch of robot_spi_slave is

  constant zero48 : std_logic_vector(47 downto 0) := (others => '0');
  constant zero32 : std_logic_vector(31 downto 0) := (others => '0');
  constant zero16 : std_logic_vector(15 downto 0) := (others => '0');
  constant zero8  : std_logic_vector(7 downto 0)  := (others => '0');

  signal iSPI_MASTER_ADDR  : std_logic_vector(31 downto 0);
  signal iSPI_MASTER_DATA  : std_logic_vector(31 downto 0);
  signal iSPI_SLAVE_DATA   : std_logic_vector(31 downto 0);

  signal iSPI_MOSI         : std_logic;
  signal iSPI_MOSI_OLD     : std_logic;
  signal iSPI_MOSI_OLD2    : std_logic;
  signal iSPI_CLK          : std_logic;
  signal iSPI_CLK_OLD      : std_logic;

  signal iPERIOD_DETECT    : std_logic_vector(31 downto 0);
  signal iBITCNT           : std_logic_vector(7 downto 0);

  signal iRECV_SR          : std_logic_vector(47 downto 0);
  signal iSEND_SR          : std_logic_vector(47 downto 0);

  signal iRECV_DATA        : std_logic_vector(47 downto 0);

  signal iREG_SELECT       : std_logic_vector(3 downto 0);

  signal iRECV_CRC_LFSR    : std_logic_vector(7 downto 0);
  signal iSEND_CRC_LFSR    : std_logic_vector(7 downto 0);

  signal iSPI_MASTER_RD    : std_logic;
  signal iSPI_MASTER_WR    : std_logic;

  function RMAP_CalculateCRC (
    constant INCRC: in Std_Logic_Vector(7 downto 0);
    constant INBYTE: in Std_Logic_Vector(7 downto 0))
    return
    Std_Logic_Vector is
-- Same range as the two inputs
-- This variable is to hold the output CRC value.
    variable OUTCRC:
      Std_Logic_Vector(7 downto 0);
-- Internal Linear Feedback Shift Register (LFSR). Note that the
-- vector indices correspond to the powers of the Galois field
-- polynomial g(x) which are NOT the same as the indices of the
-- SpaceWire data byte.
    variable LFSR:
      Std_Logic_Vector(7 downto 0);
  begin
-- External to internal bit-order reversal to match indices.
    for i in 0 to 7 loop
      LFSR(7-i) := INCRC(i);
    end loop;
-- Left shift LFSR eight times feeding in INBYTE bit 0 first (LSB).
    for j in 0 to 7 loop
      LFSR(7 downto 0) := (LFSR(6 downto 2)) &
                          (INBYTE(j) xor LFSR(7) xor LFSR(1)) &
                          (INBYTE(j) xor LFSR(7) xor LFSR(0)) &
                          (INBYTE(j) xor LFSR(7));
    end loop;
-- Internal to external bit-order reversal to match indices.
    for i in 0 to 7 loop
      OUTCRC(7-i) := LFSR(i);
    end loop;
-- Return the updated RMAP CRC byte value.
    return OUTCRC;
  end function RMAP_CalculateCRC;

  function MyLittleCRC (
-- FIXME : DEBUG : I0 & I1 & I2 & I3 & I4 & I5 & I6 & I7 = I1 & I2 & I3 & I4 & I5 & (B xor I0 xor I6) & (B xor I0 xor I7) & (B xor I0) ??
    constant INCRC: in Std_Logic_Vector(7 downto 0);
    constant INBIT: in Std_Logic)
    return
    Std_Logic_Vector is
    variable OUTCRC:
      Std_Logic_Vector(7 downto 0);
    variable LFSR:
      Std_Logic_Vector(7 downto 0);
  begin
    for i in 0 to 7 loop
      LFSR(7-i) := INCRC(i);
    end loop;
    LFSR(7 downto 0) := (LFSR(6 downto 2)) &
                        (INBIT xor LFSR(7) xor LFSR(1)) &
                        (INBIT xor LFSR(7) xor LFSR(0)) &
                        (INBIT xor LFSR(7));
    for i in 0 to 7 loop
      OUTCRC(7-i) := LFSR(i);
    end loop;
    return OUTCRC;
  end function MyLittleCRC;

  function MySwap (
    constant INBYTE: in Std_Logic_Vector(7 downto 0))
    return
    Std_Logic_Vector is
    variable OUTBYTE:
      Std_Logic_Vector(7 downto 0);
  begin
    for i in 0 to 7 loop
      OUTBYTE(7-i) := INBYTE(i);
    end loop;
    return OUTBYTE;
  end function MySwap;

begin

  latch_proc : process (CLK, RESET)
  begin
    if RESET = '1' then
      iSPI_MOSI <= '0';
      iSPI_MOSI_OLD <= '0';
      iSPI_MOSI_OLD2 <= '0';
      iSPI_CLK <= '0';
      iSPI_CLK_OLD <= '0';
    elsif rising_edge(CLK) then
      iSPI_MOSI <= SPI_MOSI;
      iSPI_MOSI_OLD <= iSPI_MOSI;
--      iSPI_MOSI_OLD2 <= iSPI_MOSI_OLD;
      iSPI_MOSI_OLD2 <= iSPI_MOSI;
      iSPI_CLK <= SPI_CLK;
      iSPI_CLK_OLD <= iSPI_CLK;
    end if;
  end process;

  slave_spi_proc : process (CLK, RESET)
    variable iRECV_SR_NEXT : std_logic_vector(47 downto 0) := zero48;
    variable iSLV_DATA_NEXT : std_logic_vector(31 downto 0) := zero32;
    variable iSEND_CRC_LFSR_NEXT : std_logic_vector(7 downto 0) := zero8;
    variable iRECV_CRC_LFSR_NEXT : std_logic_vector(7 downto 0) := zero8;
    variable iDBG_CRC_DATA : std_logic_vector(31 downto 0) := zero32;
  begin
    if RESET = '1' then
      iRECV_SR         <= zero48;
      iSEND_SR         <= (others => '1');
      iPERIOD_DETECT   <= zero32;
      iBITCNT          <= zero8;
      iREG_SELECT      <= "0000";
      iSPI_MASTER_RD   <= '0';
      iSPI_MASTER_WR   <= '0';
      iSPI_MASTER_ADDR <= zero32;
      iSPI_MASTER_DATA <= zero32;
      iSPI_SLAVE_DATA  <= zero32;
      iRECV_CRC_LFSR   <= zero8;
      iSEND_CRC_LFSR   <= zero8;
      DBG_CRC_DATA     <= zero32;
      DBG_CRC_WR       <= '0';
    elsif rising_edge(CLK) then
      iDBG_CRC_DATA := zero32;
      if (iSPI_CLK_OLD = '0') and (iSPI_CLK = '1') then
        iPERIOD_DETECT <= zero32;
      else
        if (iPERIOD_DETECT/=X"FFFFFFF0") then
          iPERIOD_DETECT <= iPERIOD_DETECT + 1;
        end if;
      end if;
      if (iPERIOD_DETECT=X"00000080") then
        iRECV_SR    <= zero48;
        iBITCNT     <= zero8;
        iREG_SELECT <= "0000";
        iRECV_CRC_LFSR <= zero8;
      else

-- FIXME : TODO : implementer un registre de mode avec flags CPOL et CPHA
--        if (iSPI_CLK_OLD = '1') and (iSPI_CLK = '0') then
        if (iSPI_CLK_OLD = '0') and (iSPI_CLK = '1') then
          iRECV_SR_NEXT := iRECV_SR(46 downto 0) & iSPI_MOSI_OLD2;
          iRECV_SR <= iRECV_SR_NEXT;
          iBITCNT <= iBITCNT + 1;
          if (iBITCNT = X"03") then
            iREG_SELECT <= iRECV_SR_NEXT(3 downto 0);
          end if;
          if (iBITCNT = X"04") then
            if (iREG_SELECT = X"5") then
              iSPI_MASTER_RD <= '1';
            end if;
          end if;
          if (iBITCNT = X"2F") then
            case iREG_SELECT is
              when X"0" =>
                DBG_MST_DATA <= iRECV_SR_NEXT(39 downto 8);
              when X"1" =>
                null; -- FIXME : TODO : trace
              when X"2" =>
                null; -- FIXME : TODO : leon bstream
              when X"3" =>
                iSPI_MASTER_ADDR <= iRECV_SR_NEXT(39 downto 8);
              when X"4" =>
                iSPI_MASTER_DATA <= iRECV_SR_NEXT(39 downto 8);
                iSPI_MASTER_WR <= '1';
              when X"5" =>
                null; -- SPI_SLAVE_DATA
              when others =>
                null;
            end case;
            iBITCNT <= zero8;
            iRECV_CRC_LFSR <= zero8;
            iDBG_CRC_DATA := X"aaaa0000";
          else
            iRECV_CRC_LFSR_NEXT := MyLittleCRC(iRECV_CRC_LFSR,iSPI_MOSI_OLD2);
            if (iBITCNT = X"00") then
              iDBG_CRC_DATA := X"ffff" &
                               iSPI_MOSI_OLD2 & "0000000" &
                               iRECV_CRC_LFSR_NEXT;
            else
              iDBG_CRC_DATA := X"0000" &
                               iSPI_MOSI_OLD2 & "0000000" &
                               iRECV_CRC_LFSR_NEXT;
            end if;
            iRECV_CRC_LFSR <= iRECV_CRC_LFSR_NEXT;
          end if;
          DBG_CRC_DATA <= iDBG_CRC_DATA;
          DBG_CRC_WR <= '1';
        else
          DBG_CRC_WR <= '0';
        end if; -- (iSPI_CLK_OLD = '0') and (iSPI_CLK = '1')

-- FIXME : TODO : implementer un registre de mode avec flags CPOL et CPHA
--        if (iSPI_CLK_OLD = '1') and (iSPI_CLK = '0') then
        if (iSPI_CLK_OLD = '0') and (iSPI_CLK = '1') then
          if (iBITCNT = X"00") then
            iSEND_SR         <= (others => '1');
            iSEND_CRC_LFSR_NEXT := X"E0";
--            iDBG_CRC_DATA := X"ffff" &
--                             '1' & "0000000" &
--                             iSEND_CRC_LFSR_NEXT;
          elsif (iBITCNT = X"01") then
            iSEND_SR         <= (others => '1');
            iSEND_CRC_LFSR_NEXT := X"90";
--            iDBG_CRC_DATA := X"0000" &
--                             '1' & "0000000" &
--                             iSEND_CRC_LFSR_NEXT;
          elsif (iBITCNT = X"07") then
            case iREG_SELECT is
              when X"0" =>
                iSLV_DATA_NEXT := DBG_SLV_DATA;
              when X"1" =>
                iSLV_DATA_NEXT := X"55AA55AA"; -- FIXME : TODO : trace
              when X"2" =>
                iSLV_DATA_NEXT := X"55AA55AA"; -- FIXME : TODO : leon bstream
              when X"3" =>
                iSLV_DATA_NEXT := iSPI_MASTER_ADDR;
              when X"4" =>
                iSLV_DATA_NEXT := iSPI_MASTER_DATA;
              when X"5" =>
                iSLV_DATA_NEXT := iSPI_SLAVE_DATA;
              when others =>
                iSLV_DATA_NEXT := X"55AA55AA";
            end case;
            iSEND_CRC_LFSR_NEXT := MyLittleCRC(iSEND_CRC_LFSR,iSEND_SR(47));
--            iDBG_CRC_DATA := X"0000" &
--                             iSEND_SR(47) & "0000000" &
--                             iSEND_CRC_LFSR_NEXT;
            iSEND_SR <= iSLV_DATA_NEXT & X"FFFF";
          elsif (iBITCNT = X"27") then
            iSEND_CRC_LFSR_NEXT := MyLittleCRC(iSEND_CRC_LFSR,iSEND_SR(47));
--            iDBG_CRC_DATA := X"5555" &
--                             iSEND_SR(47) & "0000000" &
--                             iSEND_CRC_LFSR_NEXT;
            if ((iREG_SELECT = X"0") or
                (iREG_SELECT = X"1") or
                (iREG_SELECT = X"5")) then
              iSEND_SR <= MySwap(iSEND_CRC_LFSR_NEXT) & X"FFFFFFFFFF";
            else
              iSEND_SR <= MySwap(iRECV_CRC_LFSR_NEXT) & X"FFFFFFFFFF";
            end if;
          else
            iSEND_CRC_LFSR_NEXT := MyLittleCRC(iSEND_CRC_LFSR,iSEND_SR(47));
--            iDBG_CRC_DATA := X"0000" &
--                             iSEND_SR(47) & "0000000" &
--                             iSEND_CRC_LFSR_NEXT;
            iSEND_SR <= iSEND_SR(46 downto 0) & '1';
          end if;
          iSEND_CRC_LFSR <= iSEND_CRC_LFSR_NEXT;
--          DBG_CRC_DATA <= iDBG_CRC_DATA;
--          DBG_CRC_WR <= '1';
--        else
--          DBG_CRC_WR <= '0';
        end if; -- (iSPI_CLK_OLD = '1') and (iSPI_CLK = '0')

      end if;
      if (iSPI_MASTER_WR = '1') then
        iSPI_MASTER_WR <= '0';
      end if;
      if (iSPI_MASTER_RD = '1') then
        iSPI_SLAVE_DATA <= SPI_SLAVE_DATA;
        iSPI_MASTER_RD <= '0';
      end if;
    end if;
  end process;

  SPI_MASTER_ADDR  <= iSPI_MASTER_ADDR;

  SPI_MASTER_DATA  <= iSPI_MASTER_DATA;

  SPI_MASTER_WR    <= iSPI_MASTER_WR;
--  SPI_MASTER_WR    <= '0';

  SPI_MASTER_RD    <= iSPI_MASTER_RD;

  SPI_MISO <= iSEND_SR(47);
--  SPI_MISO <= '1';

  SPI_SLAVE_IRQ <= '0';

end robot_spi_slave_arch;
