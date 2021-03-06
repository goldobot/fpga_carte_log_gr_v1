-- autogenerated by /home/lionelt/sss/trunk/tools/generate_ram.php 256 8
-- on 2011-09-15T11:39:47+02:00
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.core_config.all;

entity ram_256x8 is port (
  NRST:in  std_logic;
  CLK: in  std_logic;
  WEN: in  std_logic;
  ADD: in  std_logic_vector(7 downto 0);
  DI:  in  std_logic_vector(7 downto 0);
  DO:  out std_logic_vector(7 downto 0)
  );
end entity ram_256x8;

architecture rtl of ram_256x8 is

  type TYPE_MEM is array(0 to 255) of bit_vector(7 downto 0);

begin

  CLKN <= not(CLK);

  xilinx_gen : if CORE_TECH = xilinx generate
    xilinx_ram : process (CLK)
      variable mem : TYPE_MEM;
    begin
      if falling_edge(CLK) then
        if WEN = '0' then
          mem(to_integer(unsigned(ADD))) := to_bitvector(DI);
        end if;
        DO <= to_stdlogicvector(mem(to_integer(unsigned(ADD))));
      end if;
    end process;
  end generate;

end architecture;
