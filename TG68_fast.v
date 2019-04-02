// File /home/thomas/sandbox/tg68/VHDL/TG68_fast.vhd translated with vhd2vl v3.0 VHDL to Verilog RTL translator
// vhd2vl settings:
//  * Verilog Module Declaration Style: 2001

// vhd2vl is Free (libre) Software:
//   Copyright (C) 2001 Vincenzo Liguori - Ocean Logic Pty Ltd
//     http://www.ocean-logic.com
//   Modifications Copyright (C) 2006 Mark Gonzales - PMC Sierra Inc
//   Modifications (C) 2010 Shankar Giri
//   Modifications Copyright (C) 2002-2017 Larry Doolittle
//     http://doolittle.icarus.com/~larry/vhd2vl/
//   Modifications (C) 2017 Rodrigo A. Melo
//
//   vhd2vl comes with ABSOLUTELY NO WARRANTY.  Always check the resulting
//   Verilog for correctness, ideally with a formal verification tool.
//
//   You are welcome to redistribute vhd2vl under certain conditions.
//   See the license (GPLv2) file included with the source for details.

// The result of translation follows.  Its copyright status should be
// considered unchanged from the original VHDL.

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//                                                                          --
// This is the 68000 software compatible Kernal of TG68                     --
//                                                                          --
// Copyright (c) 2007-2010 Tobias Gubener <tobiflex@opencores.org>          -- 
//                                                                          --
// This source file is free software: you can redistribute it and/or modify --
// it under the terms of the GNU Lesser General Public License as published --
// by the Free Software Foundation, either version 3 of the License, or     --
// (at your option) any later version.                                      --
//                                                                          --
// This source file is distributed in the hope that it will be useful,      --
// but WITHOUT ANY WARRANTY; without even the implied warranty of           --
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            --
// GNU General Public License for more details.                             --
//                                                                          --
// You should have received a copy of the GNU General Public License        --
// along with this program.  If not, see <http://www.gnu.org/licenses/>.    --
//                                                                          --
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//
// Revision 1.08 2010/06/14
// Bugfix Movem with regmask==xFFFF
// Add missing Illegal $4AFC
//
// Revision 1.07 2009/10/02
// Bugfix Movem with regmask==x0000
//
// Revision 1.06 2009/02/10
// Bugfix shift and rotations opcodes when the bitcount and the data are in the same register:
// Example lsr.l D2,D2
// Thanks to Peter Graf for report
//
// Revision 1.05 2009/01/26
// Implement missing RTR
// Thanks to Peter Graf for report
//
// Revision 1.04 2007/12/29
// size improvement
// change signal "microaddr" to one hot state machine
//
// Revision 1.03 2007/12/21
// Thanks to Andreas Ehliar
// Split regfile to use blockram for registers
// insert "WHEN OTHERS => null;" on END CASE; 
//
// Revision 1.02 2007/12/17
// Bugfix jsr  nn.w
//
// Revision 1.01 2007/11/28
// add MOVEP
// Bugfix Interrupt in MOVEQ
//
// Revision 1.0 2007/11/05
// Clean up code and first release
//
// known bugs/todo:
// Add CHK INSTRUCTION
// full decode ILLEGAL INSTRUCTIONS
// Add FC Output
// add odd Address test
// add TRACE
// no timescale needed

module TG68_fast(
input wire clk,
input wire reset,
input wire clkena_in,
input wire [15:0] data_in,
input wire [2:0] IPL,
input wire test_IPL,
output wire [31:0] address,
output reg [15:0] data_write,
output wire [1:0] state_out,
output wire UDS,
output wire LDS,
output reg decodeOPC,
output wire wr
);

//low active
//only for debugging



reg [1:0] state;
wire clkena;
reg [31:0] TG68_PC;
reg [31:0] TG68_PC_add;
reg [31:0] memaddr;
reg [31:0] memaddr_in;
reg [31:0] ea_data;
reg ea_data_OP1;
reg setaddrlong;
reg [31:0] OP1out; reg [31:0] OP2out;
reg [15:0] OP1outbrief;
reg [31:0] OP1in;
reg [31:0] data_write_tmp;
wire [31:0] Xtmp;
reg [31:0] PC_dataa; reg [31:0] PC_datab; wire [31:0] PC_result;
wire setregstore;
reg [1:0] datatype;
reg longread;
reg longreaddirect;
reg long_done;
reg nextpass;
reg setnextpass;
reg setdispbyte;
reg setdisp;
reg setdispbrief;
reg regdirectsource;
reg endOPC;
reg postadd;
reg presub;
reg [31:0] addsub_a;
reg [31:0] addsub_b;
reg [31:0] addsub_q;
reg [31:0] briefext;
reg setbriefext;
reg addsub;
reg [3:0] c_in;
reg [2:0] c_out;
reg [33:0] add_result;
reg [2:0] addsub_ofl;
reg [2:0] flag_z;
reg [15:0] last_data_read;
reg [31:0] data_read;
reg [31:0] registerin;
reg [31:0] reg_QA;
reg [31:0] reg_QB;
reg Hwrena; reg Lwrena;
reg Regwrena;
reg [6:0] rf_dest_addr;
reg [6:0] rf_source_addr;
reg [6:0] rf_dest_addr_tmp;
reg [6:0] rf_source_addr_tmp;
reg [15:0] opcode;
wire [1:0] laststate;
reg [1:0] setstate;
wire [31:0] mem_address;
reg [31:0] memaddr_a;
wire [31:0] mem_data_read;
wire [31:0] mem_data_write;
reg set_mem_rega;
wire [31:0] data_read_ram;
wire [7:0] data_read_uart;
wire [31:0] counter_reg;
reg TG68_PC_br8;
reg TG68_PC_brw;
reg TG68_PC_nop;
reg setgetbrief;
reg getbrief;
reg [15:0] brief;
reg dest_areg;
reg source_areg;
reg data_is_source;
reg set_store_in_tmp;
wire store_in_tmp;
reg write_back;
reg setaddsub;
reg setstackaddr;
reg writePC;
reg writePC_add;
reg set_TG68_PC_dec;
reg [1:0] TG68_PC_dec;
reg directPC;
reg set_directPC;
reg execOPC;
reg fetchOPC;
reg [15:0] Flags;  //T.S..III ...XNZVC
wire [3:0] set_Flags;  //NZVC
reg exec_ADD;
reg exec_OR;
reg exec_AND;
reg exec_EOR;
reg exec_MOVE;
reg exec_MOVEQ;
reg exec_MOVESR;
reg exec_DIRECT;
reg exec_ADDQ;
reg exec_CMP;
reg exec_ROT;
reg exec_exg;
reg exec_swap;
reg exec_write_back;
reg exec_tas;
reg exec_EXT;
reg exec_ABCD;
reg exec_SBCD;
reg exec_MULU;
reg exec_DIVU;
reg exec_Scc;
reg exec_CPMAW;
reg set_exec_ADD;
reg set_exec_OR;
reg set_exec_AND;
reg set_exec_EOR;
reg set_exec_MOVE;
reg set_exec_MOVEQ;
reg set_exec_MOVESR;
reg set_exec_ADDQ;
reg set_exec_CMP;
reg set_exec_ROT;
reg set_exec_tas;
reg set_exec_EXT;
reg set_exec_ABCD;
reg set_exec_SBCD;
reg set_exec_MULU;
reg set_exec_DIVU;
reg set_exec_Scc;
reg set_exec_CPMAW;
reg condition;
reg OP2out_one;
reg OP1out_zero;
reg ea_to_pc;
reg ea_build;
reg ea_only;
reg get_ea_now;
reg source_lowbits;
reg dest_hbits;
reg rot_rot;
reg rot_lsb;
reg rot_msb;
reg rot_XC;
reg set_rot_nop;
reg rot_nop;
reg [31:0] rot_out;
reg [1:0] rot_bits;
reg [5:0] rot_cnt;
reg [5:0] set_rot_cnt;
reg movem_busy;
reg set_movem_busy;
reg movem_addr;
reg [3:0] movem_regaddr;
reg [15:0] movem_mask;
reg set_get_movem_mask;
reg get_movem_mask;
reg maskzero;
reg test_maskzero;
reg [7:0] movem_muxa;
reg [3:0] movem_muxb;
reg [1:0] movem_muxc;
reg movem_presub;
reg save_memaddr;
reg [4:0] movem_bits;
wire [31:0] ea_calc_b;
reg set_mem_addsub;
wire [1:0] bit_bits;
reg [4:0] bit_number_reg;
reg [4:0] bit_number;
reg exec_Bits;
reg [31:0] bits_out;
reg one_bit_in;
reg one_bit_out;
reg set_get_bitnumber;
reg get_bitnumber;
reg mem_byte;
reg wait_mem_byte;
reg movepl;
reg movepw;
reg set_movepl;
reg set_movepw;
reg set_direct_data;
reg use_direct_data;
reg direct_data;
reg set_get_extendedOPC;
reg get_extendedOPC;
reg [1:0] setstate_delay;
reg [1:0] setstate_mux;
reg use_XZFlag;
reg use_XFlag;
reg [8:0] dummy_a;
reg [5:0] niba_l;
reg [5:0] niba_h;
reg niba_lc;
reg niba_hc;
wire bcda_lc;
wire bcda_hc;
reg [8:0] dummy_s;
reg [5:0] nibs_l;
reg [5:0] nibs_h;
reg nibs_lc;
reg nibs_hc;
reg [31:0] dummy_mulu;
reg [31:0] dummy_div;
reg [16:0] dummy_div_sub;
reg [16:0] dummy_div_over;
reg set_V_Flag;
reg OP1sign;
wire set_sign;
wire sign;
reg sign2;
reg muls_msb;
reg [31:0] mulu_reg;
reg [31:0] div_reg;
reg div_sign;
reg [31:0] div_quot;
wire div_ovl;
wire pre_V_Flag;
reg set_vectoraddr;
reg writeSR;
reg trap_illegal;
reg trap_priv;
reg trap_1010;
reg trap_1111;
reg trap_trap;
reg trap_trapv;
reg trap_interrupt;
reg trapmake;
reg trapd;  //   signal trap_PC        : std_logic_vector(31 downto 0);
reg [15:0] trap_SR;
reg set_directSR;
reg directSR;
reg set_directCCR;
reg directCCR;
reg set_stop;
reg stop;
reg [31:0] trap_vector;
reg to_USP;
reg from_USP;
reg to_SR;
reg from_SR;
reg illegal_write_mode;
reg illegal_read_mode;
reg illegal_byteaddr;
reg use_SP;
reg no_Flags;
wire [2:0] IPL_nr;
reg [2:0] rIPL_nr;
reg interrupt;
reg SVmode;
reg trap_chk;
reg [2:0] test_delay;
reg set_PCmarker;
reg PCmarker;
reg set_Z_error;
reg Z_error;
parameter [6:0]
  idle = 0,
  nop = 1,
  ld_nn = 2,
  st_nn = 3,
  ld_dAn1 = 4,
  ld_dAn2 = 5,
  ld_AnXn1 = 6,
  ld_AnXn2 = 7,
  ld_AnXn3 = 8,
  st_dAn1 = 9,
  st_dAn2 = 10,
  st_AnXn1 = 11,
  st_AnXn2 = 12,
  st_AnXn3 = 13,
  bra1 = 14,
  bra2 = 15,
  bsr1 = 16,
  bsr2 = 17,
  dbcc1 = 18,
  dbcc2 = 19,
  movem = 20,
  andi = 21,
  op_AxAy = 22,
  cmpm = 23,
  link = 24,
  int1 = 25,
  int2 = 26,
  int3 = 27,
  int4 = 28,
  rte = 29,
  trap1 = 30,
  trap2 = 31,
  trap3 = 32,
  movep1 = 33,
  movep2 = 34,
  movep3 = 35,
  movep4 = 36,
  movep5 = 37,
  init1 = 38,
  init2 = 39,
  mul1 = 40,
  mul2 = 41,
  mul3 = 42,
  mul4 = 43,
  mul5 = 44,
  mul6 = 45,
  mul7 = 46,
  mul8 = 47,
  mul9 = 48,
  mul10 = 49,
  mul11 = 50,
  mul12 = 51,
  mul13 = 52,
  mul14 = 53,
  mul15 = 54,
  div1 = 55,
  div2 = 56,
  div3 = 57,
  div4 = 58,
  div5 = 59,
  div6 = 60,
  div7 = 61,
  div8 = 62,
  div9 = 63,
  div10 = 64,
  div11 = 65,
  div12 = 66,
  div13 = 67,
  div14 = 68,
  div15 = 69;

reg [6:0] micro_state;
reg [6:0] next_micro_state;

reg [15:0] regfile_low[0:16];
reg [15:0] regfile_high[0:16];
wire [31:0] RWindex_A;
wire [31:0] RWindex_B;

  //---------------------------------------------------------------------------
  // Registerfile
  //---------------------------------------------------------------------------
  assign RWindex_A = {rf_dest_addr[4],rf_dest_addr[3:0] ^ 4'b1111};
  assign RWindex_B = {rf_source_addr[4],rf_source_addr[3:0] ^ 4'b1111};
  always @(negedge clk) begin
    if(clkena == 1'b1) begin
      reg_QA <= {regfile_high[RWindex_A],regfile_low[RWindex_A]};
      reg_QB <= {regfile_high[RWindex_B],regfile_low[RWindex_B]};
    end
  end

  always @(posedge clk) begin
    if(clkena == 1'b1) begin
      if(Lwrena == 1'b1) begin
        regfile_low[RWindex_A] <= registerin[15:0];
      end
      if(Hwrena == 1'b1) begin
        regfile_high[RWindex_A] <= registerin[31:16];
      end
    end
  end

  assign address = state == 2'b00 ? TG68_PC : state == 2'b01 ? 32'hffffffff : memaddr;
  assign LDS = (datatype != 2'b00 || state == 2'b00 || memaddr[0] == 1'b1) && state != 2'b01 ? 1'b0 : 1'b1;
  assign UDS = (datatype != 2'b00 || state == 2'b00 || memaddr[0] == 1'b0) && state != 2'b01 ? 1'b0 : 1'b1;
  assign state_out = state;
  assign wr = state == 2'b11 ? 1'b0 : 1'b1;
  assign IPL_nr =  ~IPL;
  //---------------------------------------------------------------------------
  // "ALU"
  //---------------------------------------------------------------------------
  always @(addsub_a, addsub_b, addsub, add_result, c_in) begin
    if(addsub == 1'b1) begin
      //ADD
      add_result <= ({1'b0,addsub_a,c_in[0]}) + ({1'b0,addsub_b,c_in[0]});
    end
    else begin
      //SUB
      add_result <= ({1'b0,addsub_a,1'b0}) - ({1'b0,addsub_b,c_in[0]});
    end
    addsub_q <= add_result[32:1];
    c_in[1] <= add_result[9] ^ addsub_a[8] ^ addsub_b[8];
    c_in[2] <= add_result[17] ^ addsub_a[16] ^ addsub_b[16];
    c_in[3] <= add_result[33];
    addsub_ofl[0] <= c_in[1] ^ add_result[8] ^ addsub_a[7] ^ addsub_b[7];
    //V Byte
    addsub_ofl[1] <= c_in[2] ^ add_result[16] ^ addsub_a[15] ^ addsub_b[15];
    //V Word
    addsub_ofl[2] <= c_in[3] ^ add_result[32] ^ addsub_a[31] ^ addsub_b[31];
    //V Long
    c_out <= c_in[3:1];
  end

  //---------------------------------------------------------------------------
  // MEM_IO 
  //---------------------------------------------------------------------------
  always @(posedge clk, posedge reset, posedge clkena_in, posedge opcode, posedge rIPL_nr, posedge longread, posedge get_extendedOPC, posedge memaddr, posedge memaddr_a, posedge set_mem_addsub, posedge movem_presub, posedge movem_busy, posedge state, posedge PCmarker, posedge execOPC, posedge datatype, posedge setdisp, posedge setdispbrief, posedge briefext, posedge setdispbyte, posedge brief, posedge set_mem_rega, posedge reg_QA, posedge setaddrlong, posedge data_read, posedge decodeOPC, posedge TG68_PC, posedge data_in, posedge long_done, posedge last_data_read, posedge mem_byte, posedge data_write_tmp, posedge addsub_q, posedge set_vectoraddr, posedge trap_vector, posedge interrupt) begin
    if((clkena_in &  ~longread &  ~get_extendedOPC) == 1'b1) begin
      trap_vector[31:8] <= {24{1'b0}};
      //		IF trap_addr_fault='1' THEN
      //			trap_vector(7 downto 0) <= X"08";
      //		END IF;	
      //		IF trap_addr_error='1' THEN
      //			trap_vector(7 downto 0) <= X"0C";
      //		END IF;	
      if(trap_illegal == 1'b1) begin
        trap_vector[7:0] <= 8'h10;
      end
      if(z_error == 1'b1) begin
        trap_vector[7:0] <= 8'h14;
      end
      //				IF trap_chk='1' THEN
      //					trap_vector(7 downto 0) <= X"18";
      //				END IF;	
      if(trap_trapv == 1'b1) begin
        trap_vector[7:0] <= 8'h1C;
      end
      if(trap_priv == 1'b1) begin
        trap_vector[7:0] <= 8'h20;
      end
      //		IF trap_trace='1' THEN
      //			trap_vector(7 downto 0) <= X"24";
      //		END IF;	
      if(trap_1010 == 1'b1) begin
        trap_vector[7:0] <= 8'h28;
      end
      if(trap_1111 == 1'b1) begin
        trap_vector[7:0] <= 8'h2C;
      end
      if(trap_trap == 1'b1) begin
        trap_vector[7:2] <= {2'b10,opcode[3:0]};
      end
      if(interrupt == 1'b1) begin
        trap_vector[7:2] <= {3'b011,rIPL_nr};
      end
    end
  end

  always @(clk, reset, clkena_in, opcode, rIPL_nr, longread, get_extendedOPC, memaddr, memaddr_a, set_mem_addsub, movem_presub, movem_busy, state, PCmarker, execOPC, datatype, setdisp, setdispbrief, briefext, setdispbyte, brief, set_mem_rega, reg_QA, setaddrlong, data_read, decodeOPC, TG68_PC, data_in, long_done, last_data_read, mem_byte, data_write_tmp, addsub_q, set_vectoraddr, trap_vector, interrupt) begin
    memaddr_a[3:0] <= 4'b0000;
    memaddr_a[7:4] <= {4{memaddr_a[3]}};
    memaddr_a[15:8] <= {8{memaddr_a[7]}};
    memaddr_a[31:16] <= {16{memaddr_a[15]}};
    if(movem_presub == 1'b1) begin
      if(movem_busy == 1'b1 || longread == 1'b1) begin
        memaddr_a[3:0] <= 4'b1110;
      end
    end
    else if(state[1] == 1'b1 || (get_extendedOPC == 1'b1 && PCmarker == 1'b1)) begin
      memaddr_a[1] <= 1'b1;
    end
    else if(execOPC == 1'b1) begin
      if(datatype == 2'b10) begin
        memaddr_a[3:0] <= 4'b1100;
      end
      else begin
        memaddr_a[3:0] <= 4'b1110;
      end
    end
    else if(setdisp == 1'b1) begin
      if(setdispbrief == 1'b1) begin
        memaddr_a <= briefext;
      end
      else if(setdispbyte == 1'b1) begin
        memaddr_a[7:0] <= brief[7:0];
      end
      else begin
        memaddr_a[15:0] <= brief;
      end
    end
    memaddr_in <= memaddr + memaddr_a;
    if(longread == 1'b0) begin
      if(set_mem_addsub == 1'b1) begin
        memaddr_in <= addsub_q;
      end
      else if(set_vectoraddr == 1'b1) begin
        memaddr_in <= trap_vector;
      end
      else if(interrupt == 1'b1) begin
        memaddr_in <= {28'b1111111111111111111111111111,rIPL_nr,1'b0};
      end
      else if(set_mem_rega == 1'b1) begin
        memaddr_in <= reg_QA;
      end
      else if(setaddrlong == 1'b1 && longread == 1'b0) begin
        memaddr_in <= data_read;
      end
      else if(decodeOPC == 1'b1) begin
        memaddr_in <= TG68_PC;
      end
    end
    data_read[15:0] <= data_in;
    data_read[31:16] <= {16{data_in[15]}};
    if(long_done == 1'b1) begin
      data_read[31:16] <= last_data_read;
    end
    if(mem_byte == 1'b1 && memaddr[0] == 1'b0) begin
      data_read[7:0] <= data_in[15:8];
    end
    if(longread == 1'b1) begin
      data_write <= data_write_tmp[31:16];
    end
    else begin
      data_write[7:0] <= data_write_tmp[7:0];
      if(mem_byte == 1'b1) begin
        data_write[15:8] <= data_write_tmp[7:0];
      end
      else begin
        data_write[15:8] <= data_write_tmp[15:8];
        if(datatype == 2'b00) begin
          data_write[7:0] <= data_write_tmp[15:8];
        end
      end
    end
  end

  always @(posedge clk, posedge reset, posedge clkena_in, posedge opcode, posedge rIPL_nr, posedge longread, posedge get_extendedOPC, posedge memaddr, posedge memaddr_a, posedge set_mem_addsub, posedge movem_presub, posedge movem_busy, posedge state, posedge PCmarker, posedge execOPC, posedge datatype, posedge setdisp, posedge setdispbrief, posedge briefext, posedge setdispbyte, posedge brief, posedge set_mem_rega, posedge reg_QA, posedge setaddrlong, posedge data_read, posedge decodeOPC, posedge TG68_PC, posedge data_in, posedge long_done, posedge last_data_read, posedge mem_byte, posedge data_write_tmp, posedge addsub_q, posedge set_vectoraddr, posedge trap_vector, posedge interrupt) begin
    if(reset == 1'b0) begin
      longread <= 1'b0;
      long_done <= 1'b0;
    end else begin
      if(clkena_in == 1'b1) begin
        last_data_read <= data_in;
        long_done <= longread;
        if(get_extendedOPC == 1'b0 || (get_extendedOPC == 1'b1 && PCmarker == 1'b1)) begin
          memaddr <= memaddr_in;
        end
        if(get_extendedOPC == 1'b0) begin
          if(((setstate_mux[1] == 1'b1 && datatype == 2'b10) || longreaddirect == 1'b1) && longread == 1'b0 && interrupt == 1'b0) begin
            longread <= 1'b1;
          end
          else begin
            longread <= 1'b0;
          end
        end
      end
    end
  end

  //---------------------------------------------------------------------------
  // brief
  //---------------------------------------------------------------------------
  always @(clk, brief, OP1out) begin
    if(brief[11] == 1'b1) begin
      OP1outbrief <= OP1out[31:16];
    end
    else begin
      OP1outbrief <= {16{OP1out[15]}};
    end
  end

  always @(posedge clk, posedge brief, posedge OP1out) begin
    if(clkena == 1'b1) begin
      briefext <= {OP1outbrief,OP1out[15:0]};
      //				CASE brief(10 downto 9) IS
      //					WHEN "00" => briefext <= OP1outbrief&OP1out(15 downto 0);
      //					WHEN "01" => briefext <= OP1outbrief(14 downto 0)&OP1out(15 downto 0)&'0';
      //					WHEN "10" => briefext <= OP1outbrief(13 downto 0)&OP1out(15 downto 0)&"00";
      //					WHEN "11" => briefext <= OP1outbrief(12 downto 0)&OP1out(15 downto 0)&"000";
      //				END CASE;
    end
  end

  //---------------------------------------------------------------------------
  // PC Calc + fetch opcode
  //---------------------------------------------------------------------------
  always @(clk, reset, opcode, TG68_PC, TG68_PC_dec, TG68_PC_br8, TG68_PC_brw, PC_dataa, PC_datab, execOPC, last_data_read, get_extendedOPC, setstate_delay, setstate) begin
    PC_dataa <= TG68_PC;
    PC_datab[2:0] <= 3'b010;
    PC_datab[7:3] <= {5{PC_datab[2]}};
    PC_datab[15:8] <= {8{PC_datab[7]}};
    PC_datab[31:16] <= {16{PC_datab[15]}};
    if(execOPC == 1'b0) begin
      if(TG68_PC_br8 == 1'b1) begin
        PC_datab[7:0] <= opcode[7:0];
      end
      if(TG68_PC_dec[1] == 1'b1) begin
        PC_datab[2] <= 1'b1;
      end
      if(TG68_PC_brw == 1'b1) begin
        PC_datab[15:0] <= last_data_read[15:0];
      end
    end
    TG68_PC_add <= PC_dataa + PC_datab;
    if(get_extendedOPC == 1'b1) begin
      setstate_mux <= setstate_delay;
    end
    else begin
      setstate_mux <= setstate;
    end
  end

  always @(posedge clk, posedge reset, posedge opcode, posedge TG68_PC, posedge TG68_PC_dec, posedge TG68_PC_br8, posedge TG68_PC_brw, posedge PC_dataa, posedge PC_datab, posedge execOPC, posedge last_data_read, posedge get_extendedOPC, posedge setstate_delay, posedge setstate) begin
    if(reset == 1'b0) begin
      opcode[15:12] <= 4'h7;
      //moveq
      opcode[8:6] <= 3'b010;
      //long
      TG68_PC <= {32{1'b0}};
      state <= 2'b01;
      decodeOPC <= 1'b0;
      fetchOPC <= 1'b0;
      endOPC <= 1'b0;
      interrupt <= 1'b0;
      trap_interrupt <= 1'b1;
      execOPC <= 1'b0;
      getbrief <= 1'b0;
      TG68_PC_dec <= 2'b00;
      directPC <= 1'b0;
      directSR <= 1'b0;
      directCCR <= 1'b0;
      stop <= 1'b0;
      exec_ADD <= 1'b0;
      exec_OR <= 1'b0;
      exec_AND <= 1'b0;
      exec_EOR <= 1'b0;
      exec_MOVE <= 1'b0;
      exec_MOVEQ <= 1'b0;
      exec_MOVESR <= 1'b0;
      exec_ADDQ <= 1'b0;
      exec_CMP <= 1'b0;
      exec_ROT <= 1'b0;
      exec_EXT <= 1'b0;
      exec_ABCD <= 1'b0;
      exec_SBCD <= 1'b0;
      exec_MULU <= 1'b0;
      exec_DIVU <= 1'b0;
      exec_Scc <= 1'b0;
      exec_CPMAW <= 1'b0;
      mem_byte <= 1'b0;
      rot_cnt <= 6'b000001;
      rot_nop <= 1'b0;
      get_extendedOPC <= 1'b0;
      get_bitnumber <= 1'b0;
      get_movem_mask <= 1'b0;
      test_maskzero <= 1'b0;
      movepl <= 1'b0;
      movepw <= 1'b0;
      test_delay <= 3'b000;
      PCmarker <= 1'b0;
    end else begin
      if(clkena_in == 1'b1) begin
        get_extendedOPC <= set_get_extendedOPC;
        get_bitnumber <= set_get_bitnumber;
        get_movem_mask <= set_get_movem_mask;
        test_maskzero <= get_movem_mask;
        setstate_delay <= setstate;
        TG68_PC_dec <= {TG68_PC_dec[0],set_TG68_PC_dec};
        if(directPC == 1'b1 && clkena == 1'b1) begin
          TG68_PC <= data_read;
        end
        else if(ea_to_pc == 1'b1 && longread == 1'b0) begin
          TG68_PC <= memaddr_in;
        end
        else if((state == 2'b00 && TG68_PC_nop == 1'b0) || TG68_PC_br8 == 1'b1 || TG68_PC_brw == 1'b1 || TG68_PC_dec[1] == 1'b1) begin
          TG68_PC <= TG68_PC_add;
        end
        if(get_bitnumber == 1'b1) begin
          bit_number_reg <= data_read[4:0];
        end
        if(clkena == 1'b1 || get_extendedOPC == 1'b1) begin
          if(set_get_extendedOPC == 1'b1) begin
            state <= 2'b00;
          end
          else if(get_extendedOPC == 1'b1) begin
            state <= setstate_mux;
          end
          else if(fetchOPC == 1'b1 || (state == 2'b10 && write_back == 1'b1 && setstate != 2'b10) || set_rot_cnt != 6'b000001 || stop == 1'b1) begin
            state <= 2'b01;
            //decode cycle, execute cycle
          end
          else begin
            state <= setstate_mux;
          end
          if(setstate_mux[1] == 1'b1 && datatype == 2'b00 && set_get_extendedOPC == 1'b0 && wait_mem_byte == 1'b0) begin
            mem_byte <= 1'b1;
          end
          else begin
            mem_byte <= 1'b0;
          end
        end
      end
      if(clkena == 1'b1) begin
        exec_ADD <= 1'b0;
        exec_OR <= 1'b0;
        exec_AND <= 1'b0;
        exec_EOR <= 1'b0;
        exec_MOVE <= 1'b0;
        exec_MOVEQ <= 1'b0;
        exec_MOVESR <= 1'b0;
        exec_ADDQ <= 1'b0;
        exec_CMP <= 1'b0;
        exec_ROT <= 1'b0;
        exec_ABCD <= 1'b0;
        exec_SBCD <= 1'b0;
        fetchOPC <= 1'b0;
        exec_CPMAW <= 1'b0;
        endOPC <= 1'b0;
        interrupt <= 1'b0;
        execOPC <= 1'b0;
        exec_EXT <= 1'b0;
        exec_Scc <= 1'b0;
        rot_nop <= 1'b0;
        decodeOPC <= fetchOPC;
        directPC <= set_directPC;
        directSR <= set_directSR;
        directCCR <= set_directCCR;
        exec_MULU <= set_exec_MULU;
        exec_DIVU <= set_exec_DIVU;
        movepl <= 1'b0;
        movepw <= 1'b0;
        stop <= set_stop | (stop &  ~interrupt);
        if(set_PCmarker == 1'b1) begin
          PCmarker <= 1'b1;
        end
        else if((state == 2'b10 && longread == 1'b0) || (ea_only == 1'b1 && get_ea_now == 1'b1)) begin
          PCmarker <= 1'b0;
        end
        if((decodeOPC | execOPC) == 1'b1) begin
          rot_cnt <= set_rot_cnt;
        end
        if(next_micro_state == idle && setstate_mux == 2'b00 && (setnextpass == 1'b0 || ea_only == 1'b1) && endOPC == 1'b0 && movem_busy == 1'b0 && set_movem_busy == 1'b0 && set_get_bitnumber == 1'b0) begin
          nextpass <= 1'b0;
          if((exec_write_back == 1'b0 || state == 2'b11) && set_rot_cnt == 6'b000001) begin
            endOPC <= 1'b1;
            if(Flags[10:8] < IPL_nr || IPL_nr == 3'b111) begin
              interrupt <= 1'b1;
              rIPL_nr <= IPL_nr;
            end
            else begin
              if(stop == 1'b0) begin
                fetchOPC <= 1'b1;
              end
            end
          end
          if(exec_write_back == 1'b0 || state != 2'b11) begin
            if(stop == 1'b0) begin
              execOPC <= 1'b1;
            end
            exec_ADD <= set_exec_ADD;
            exec_OR <= set_exec_OR;
            exec_AND <= set_exec_AND;
            exec_EOR <= set_exec_EOR;
            exec_MOVE <= set_exec_MOVE;
            exec_MOVEQ <= set_exec_MOVEQ;
            exec_MOVESR <= set_exec_MOVESR;
            exec_ADDQ <= set_exec_ADDQ;
            exec_CMP <= set_exec_CMP;
            exec_ROT <= set_exec_ROT;
            exec_tas <= set_exec_tas;
            exec_EXT <= set_exec_EXT;
            exec_ABCD <= set_exec_ABCD;
            exec_SBCD <= set_exec_SBCD;
            exec_Scc <= set_exec_Scc;
            exec_CPMAW <= set_exec_CPMAW;
            rot_nop <= set_rot_nop;
          end
        end
        else begin
          if(endOPC == 1'b0 && (setnextpass == 1'b1 || (regdirectsource == 1'b1 && decodeOPC == 1'b1))) begin
            nextpass <= 1'b1;
          end
        end
        if(interrupt == 1'b1) begin
          opcode[15:12] <= 4'h7;
          //moveq
          opcode[8:6] <= 3'b010;
          //long
          //					trap_PC <= TG68_PC;
          trap_interrupt <= 1'b1;
        end
        if(fetchOPC == 1'b1) begin
          trap_interrupt <= 1'b0;
          if((test_IPL == 1'b1 && (Flags[10:8] < IPL_nr || IPL_nr == 3'b111)) || to_SR == 1'b1) begin
            //					IF (test_IPL='1' AND (Flags(10 downto 8)<IPL_nr OR IPL_nr="111")) OR to_SR='1' OR opcode(15 downto 6)="0100111011" THEN  --nur für Validator
            opcode <= 16'h60FE;
            if(to_SR == 1'b0) begin
              test_delay <= 3'b001;
            end
          end
          else begin
            opcode <= data_read[15:0];
          end
          getbrief <= 1'b0;
          //					trap_PC <= TG68_PC;
        end
        else begin
          test_delay <= {test_delay[1:0],1'b0};
          getbrief <= setgetbrief;
          movepl <= set_movepl;
          movepw <= set_movepw;
        end
        if(decodeOPC == 1'b1 || interrupt == 1'b1) begin
          trap_SR <= Flags;
        end
        if(getbrief == 1'b1) begin
          brief <= data_read[15:0];
        end
      end
    end
  end

  //---------------------------------------------------------------------------
  // handle EA_data, data_write_tmp
  //---------------------------------------------------------------------------
  always @(posedge clk, posedge reset, posedge opcode) begin
    if(reset == 1'b0) begin
      set_store_in_tmp <= 1'b0;
      exec_DIRECT <= 1'b0;
      exec_write_back <= 1'b0;
      direct_data <= 1'b0;
      use_direct_data <= 1'b0;
      Z_error <= 1'b0;
    end else begin
      if(clkena == 1'b1) begin
        direct_data <= 1'b0;
        if(endOPC == 1'b1) begin
          set_store_in_tmp <= 1'b0;
          exec_DIRECT <= 1'b0;
          exec_write_back <= 1'b0;
          use_direct_data <= 1'b0;
          Z_error <= 1'b0;
        end
        else begin
          if(set_Z_error == 1'b1) begin
            Z_error <= 1'b1;
          end
          exec_DIRECT <= set_exec_MOVE;
          if(setstate_mux == 2'b10 && write_back == 1'b1) begin
            exec_write_back <= 1'b1;
          end
        end
        if(set_direct_data == 1'b1) begin
          direct_data <= 1'b1;
          use_direct_data <= 1'b1;
        end
        if(set_exec_MOVE == 1'b1 && state == 2'b11) begin
          use_direct_data <= 1'b1;
        end
        if((exec_DIRECT == 1'b1 && state == 2'b00 && getbrief == 1'b0 && endOPC == 1'b0) || state == 2'b10) begin
          set_store_in_tmp <= 1'b1;
          ea_data <= data_read;
        end
        if(writePC_add == 1'b1) begin
          data_write_tmp <= TG68_PC_add;
        end
        else if(writePC == 1'b1 || fetchOPC == 1'b1 || interrupt == 1'b1 || (trap_trap == 1'b1 && decodeOPC == 1'b1)) begin
          //fetchOPC für Trap
          data_write_tmp <= TG68_PC;
        end
        else if(execOPC == 1'b1 || (get_ea_now == 1'b1 && ea_only == 1'b1)) begin
          //get_ea_now='1' AND ea_only='1' ist für pea
          data_write_tmp <= {registerin[31:8],registerin[7] | exec_tas,registerin[6:0]};
        end
        else if((exec_DIRECT == 1'b1 && state == 2'b10) || direct_data == 1'b1) begin
          data_write_tmp <= data_read;
          if(movepl == 1'b1) begin
            data_write_tmp[31:8] <= data_write_tmp[23:0];
          end
        end
        else if((movem_busy == 1'b1 && datatype == 2'b10 && movem_presub == 1'b1) || movepl == 1'b1) begin
          data_write_tmp <= {OP2out[15:0],OP2out[31:16]};
        end
        else if(( ~trapmake & decodeOPC) == 1'b1 || movem_busy == 1'b1 || movepw == 1'b1) begin
          data_write_tmp <= OP2out;
        end
        else if(writeSR == 1'b1) begin
          data_write_tmp[15:0] <= {trap_SR[15:8],Flags[7:0]};
        end
      end
    end
  end

  //---------------------------------------------------------------------------
  // set dest regaddr
  //---------------------------------------------------------------------------
  always @(opcode, rf_dest_addr_tmp, to_USP, Flags, trapmake, movem_addr, movem_presub, movem_regaddr, setbriefext, brief, setstackaddr, dest_hbits, dest_areg, data_is_source) begin
    rf_dest_addr <= rf_dest_addr_tmp;
    if(rf_dest_addr_tmp[3:0] == 4'b1111 && to_USP == 1'b0) begin
      rf_dest_addr[4] <= Flags[13] | trapmake;
    end
    if(movem_addr == 1'b1) begin
      if(movem_presub == 1'b1) begin
        rf_dest_addr_tmp <= {3'b000,movem_regaddr ^ 4'b1111};
      end
      else begin
        rf_dest_addr_tmp <= {3'b000,movem_regaddr};
      end
    end
    else if(setbriefext == 1'b1) begin
      rf_dest_addr_tmp <= {3'b000,brief[15:12]};
    end
    else if(setstackaddr == 1'b1) begin
      rf_dest_addr_tmp <= 7'b0001111;
    end
    else if(dest_hbits == 1'b1) begin
      rf_dest_addr_tmp <= {3'b000,dest_areg,opcode[11:9]};
    end
    else begin
      if(opcode[5:3] == 3'b000 || data_is_source == 1'b1) begin
        rf_dest_addr_tmp <= {3'b000,dest_areg,opcode[2:0]};
      end
      else begin
        rf_dest_addr_tmp <= {4'b0001,opcode[2:0]};
      end
    end
  end

  //---------------------------------------------------------------------------
  // set OP1
  //---------------------------------------------------------------------------
  always @(reg_QA, OP1out_zero, from_SR, Flags, ea_data_OP1, set_store_in_tmp, ea_data) begin
    OP1out <= reg_QA;
    if(OP1out_zero == 1'b1) begin
      OP1out <= {32{1'b0}};
    end
    else if(from_SR == 1'b1) begin
      OP1out[15:0] <= Flags;
    end
    else if(ea_data_OP1 == 1'b1 && set_store_in_tmp == 1'b1) begin
      OP1out <= ea_data;
    end
  end

  //---------------------------------------------------------------------------
  // set source regaddr
  //---------------------------------------------------------------------------
  always @(opcode, Flags, movem_addr, movem_presub, movem_regaddr, source_lowbits, source_areg, from_USP, rf_source_addr_tmp) begin
    rf_source_addr <= rf_source_addr_tmp;
    if(rf_source_addr_tmp[3:0] == 4'b1111 && from_USP == 1'b0) begin
      rf_source_addr[4] <= Flags[13];
    end
    if(movem_addr == 1'b1) begin
      if(movem_presub == 1'b1) begin
        rf_source_addr_tmp <= {3'b000,movem_regaddr ^ 4'b1111};
      end
      else begin
        rf_source_addr_tmp <= {3'b000,movem_regaddr};
      end
    end
    else if(from_USP == 1'b1) begin
      rf_source_addr_tmp <= 7'b0001111;
    end
    else if(source_lowbits == 1'b1) begin
      rf_source_addr_tmp <= {3'b000,source_areg,opcode[2:0]};
    end
    else begin
      rf_source_addr_tmp <= {3'b000,source_areg,opcode[11:9]};
    end
  end

  //---------------------------------------------------------------------------
  // set OP2
  //---------------------------------------------------------------------------
  always @(OP2out, reg_QB, opcode, datatype, OP2out_one, exec_EXT, exec_MOVEQ, EXEC_ADDQ, use_direct_data, data_write_tmp, ea_data_OP1, set_store_in_tmp, ea_data, movepl) begin
    OP2out[15:0] <= reg_QB[15:0];
    OP2out[31:16] <= {16{OP2out[15]}};
    if(OP2out_one == 1'b1) begin
      OP2out[15:0] <= 16'b1111111111111111;
    end
    else if(exec_EXT == 1'b1) begin
      if(opcode[6] == 1'b0) begin
        //ext.w
        OP2out[15:8] <= {8{OP2out[7]}};
      end
    end
    else if(use_direct_data == 1'b1) begin
      OP2out <= data_write_tmp;
    end
    else if(ea_data_OP1 == 1'b0 && set_store_in_tmp == 1'b1) begin
      OP2out <= ea_data;
    end
    else if(exec_MOVEQ == 1'b1) begin
      OP2out[7:0] <= opcode[7:0];
      OP2out[15:8] <= {8{opcode[7]}};
    end
    else if(exec_ADDQ == 1'b1) begin
      OP2out[2:0] <= opcode[11:9];
      if(opcode[11:9] == 3'b000) begin
        OP2out[3] <= 1'b1;
      end
      else begin
        OP2out[3] <= 1'b0;
      end
      OP2out[15:4] <= {12{1'b0}};
    end
    else if(datatype == 2'b10 || movepl == 1'b1) begin
      OP2out[31:16] <= reg_QB[31:16];
    end
  end

  //---------------------------------------------------------------------------
  // addsub
  //---------------------------------------------------------------------------
  always @(OP1out, OP2out, presub, postadd, execOPC, OP2out_one, datatype, use_SP, use_XZFlag, use_XFlag, Flags, setaddsub) begin
    addsub_a <= OP1out;
    addsub_b <= OP2out;
    addsub <=  ~presub;
    c_in[0] <= 1'b0;
    if(execOPC == 1'b0 && OP2out_one == 1'b0) begin
      if(datatype == 2'b00 && use_SP == 1'b0) begin
        addsub_b <= 32'b00000000000000000000000000000001;
      end
      else if(datatype == 2'b10 && (presub | postadd) == 1'b1) begin
        addsub_b <= 32'b00000000000000000000000000000100;
      end
      else begin
        addsub_b <= 32'b00000000000000000000000000000010;
      end
    end
    else begin
      if((use_XZFlag == 1'b1 || use_XFlag == 1'b1) && Flags[4] == 1'b1) begin
        c_in[0] <= 1'b1;
      end
      addsub <= setaddsub;
    end
  end

  //---------------------------------------------------------------------------
  // Write Reg
  //---------------------------------------------------------------------------
  always @(clkena, OP1in, datatype, presub, postadd, endOPC, regwrena, state, execOPC, last_data_read, movem_addr, rf_dest_addr, reg_QA, maskzero) begin
    Lwrena <= 1'b0;
    Hwrena <= 1'b0;
    registerin <= OP1in;
    if((presub == 1'b1 || postadd == 1'b1) && endOPC == 1'b0) begin
      // -(An)+
      Hwrena <= 1'b1;
      Lwrena <= 1'b1;
    end
    else if(Regwrena == 1'b1 && maskzero == 1'b0) begin
      //read (mem)
      Lwrena <= 1'b1;
      case(datatype)
      2'b00 : begin
        //BYTE
        registerin[15:8] <= reg_QA[15:8];
      end
      2'b01 : begin
        //WORD
        if(rf_dest_addr[3] == 1'b1 || movem_addr == 1'b1) begin
          Hwrena <= 1'b1;
        end
      end
      default : begin
        //LONG
        Hwrena <= 1'b1;
      end
      endcase
    end
  end

  //----------------------------------------------------------------------------
  //ALU
  //----------------------------------------------------------------------------		
  always @(opcode, OP1in, OP1out, OP2out, datatype, c_out, exec_ABCD, exec_SBCD, exec_CPMAW, exec_MOVESR, bits_out, Flags, flag_z, use_XZFlag, addsub_ofl, dummy_s, dummy_a, niba_hc, niba_h, niba_l, niba_lc, nibs_hc, nibs_h, nibs_l, nibs_lc, addsub_q, movem_addr, data_read, exec_MULU, exec_DIVU, exec_OR, exec_AND, exec_Scc, exec_EOR, exec_MOVE, exec_exg, exec_ROT, execOPC, exec_swap, exec_Bits, rot_out, dummy_mulu, dummy_div, save_memaddr, memaddr, memaddr_in, ea_only, get_ea_now) begin
    //BCD_ARITH-------------------------------------------------------------------
    //ADC
    dummy_a <= {niba_hc,niba_h[4:1] + ({1'b0,niba_hc,niba_hc,1'b0}),niba_l[4:1] + ({1'b0,niba_lc,niba_lc,1'b0})};
    niba_l <= ({1'b0,OP1out[3:0],1'b1}) + ({1'b0,OP2out[3:0],Flags[4]});
    niba_lc <= niba_l[5] | (niba_l[4] & niba_l[3]) | (niba_l[4] & niba_l[2]);
    niba_h <= ({1'b0,OP1out[7:4],1'b1}) + ({1'b0,OP2out[7:4],niba_lc});
    niba_hc <= niba_h[5] | (niba_h[4] & niba_h[3]) | (niba_h[4] & niba_h[2]);
    //SBC			
    dummy_s <= {nibs_hc,nibs_h[4:1] - ({1'b0,nibs_hc,nibs_hc,1'b0}),nibs_l[4:1] - ({1'b0,nibs_lc,nibs_lc,1'b0})};
    nibs_l <= ({1'b0,OP1out[3:0],1'b0}) - ({1'b0,OP2out[3:0],Flags[4]});
    nibs_lc <= nibs_l[5];
    nibs_h <= ({1'b0,OP1out[7:4],1'b0}) - ({1'b0,OP2out[7:4],nibs_lc});
    nibs_hc <= nibs_h[5];
    //----------------------------------------------------------------------------		
    flag_z <= 3'b000;
    OP1in <= addsub_q;
    if(movem_addr == 1'b1) begin
      OP1in <= data_read;
    end
    else if(exec_ABCD == 1'b1) begin
      OP1in[7:0] <= dummy_a[7:0];
    end
    else if(exec_SBCD == 1'b1) begin
      OP1in[7:0] <= dummy_s[7:0];
    end
    else if(exec_MULU == 1'b1) begin
      OP1in <= dummy_mulu;
    end
    else if(exec_DIVU == 1'b1 && execOPC == 1'b1) begin
      OP1in <= dummy_div;
    end
    else if(exec_OR == 1'b1) begin
      OP1in <= OP2out | OP1out;
    end
    else if(exec_AND == 1'b1 || exec_Scc == 1'b1) begin
      OP1in <= OP2out & OP1out;
    end
    else if(exec_EOR == 1'b1) begin
      OP1in <= OP2out ^ OP1out;
    end
    else if(exec_MOVE == 1'b1 || exec_exg == 1'b1) begin
      OP1in <= OP2out;
    end
    else if(exec_ROT == 1'b1) begin
      OP1in <= rot_out;
    end
    else if(save_memaddr == 1'b1) begin
      OP1in <= memaddr;
    end
    else if(get_ea_now == 1'b1 && ea_only == 1'b1) begin
      OP1in <= memaddr_in;
    end
    else if(exec_swap == 1'b1) begin
      OP1in <= {OP1out[15:0],OP1out[31:16]};
    end
    else if(exec_bits == 1'b1) begin
      OP1in <= bits_out;
    end
    else if(exec_MOVESR == 1'b1) begin
      OP1in[15:0] <= Flags;
    end
    if(use_XZFlag == 1'b1 && flags[2] == 1'b0) begin
      flag_z <= 3'b000;
    end
    else if(OP1in[7:0] == 8'b00000000) begin
      flag_z[0] <= 1'b1;
      if(OP1in[15:8] == 8'b00000000) begin
        flag_z[1] <= 1'b1;
        if(OP1in[31:16] == 16'b0000000000000000) begin
          flag_z[2] <= 1'b1;
        end
      end
    end
    //					--Flags NZVC
    if(datatype == 2'b00) begin
      //Byte
      set_flags <= {OP1IN[7],flag_z[0],addsub_ofl[0],c_out[0]};
      if(exec_ABCD == 1'b1) begin
        set_flags[0] <= dummy_a[8];
      end
      else if(exec_SBCD == 1'b1) begin
        set_flags[0] <= dummy_s[8];
      end
    end
    else if(datatype == 2'b10 || exec_CPMAW == 1'b1) begin
      //Long
      set_flags <= {OP1IN[31],flag_z[2],addsub_ofl[2],c_out[2]};
    end
    else begin
      //Word
      set_flags <= {OP1IN[15],flag_z[1],addsub_ofl[1],c_out[1]};
    end
  end

  //----------------------------------------------------------------------------
  //Flags
  //----------------------------------------------------------------------------		
  always @(posedge clk, posedge reset, posedge opcode) begin
    if(reset == 1'b0) begin
      Flags[13] <= 1'b1;
      SVmode <= 1'b1;
      Flags[10:8] <= 3'b111;
    end else begin
      if(clkena == 1'b1) begin
        if(directSR == 1'b1) begin
          Flags <= data_read[15:0];
        end
        if(directCCR == 1'b1) begin
          Flags[7:0] <= data_read[7:0];
        end
        if(interrupt == 1'b1) begin
          Flags[10:8] <= rIPL_nr;
          SVmode <= 1'b1;
        end
        if(writeSR == 1'b1 || interrupt == 1'b1) begin
          Flags[13] <= 1'b1;
        end
        if(endOPC == 1'b1 && to_SR == 1'b0) begin
          SVmode <= Flags[13];
        end
        if(execOPC == 1'b1 && to_SR == 1'b1) begin
          Flags[7:0] <= OP1in[7:0];
          //CCR
          if(datatype == 2'b01 && (opcode[14] == 1'b0 || opcode[9] == 1'b1)) begin
            //move to CCR wird als word gespeichert
            Flags[15:8] <= OP1in[15:8];
            //SR
            SVmode <= OP1in[13];
          end
        end
        else if(Z_error == 1'b1) begin
          if(opcode[8] == 1'b0) begin
            Flags[3:0] <= 4'b1000;
          end
          else begin
            Flags[3:0] <= 4'b0100;
          end
        end
        else if(no_Flags == 1'b0 && trapmake == 1'b0) begin
          if(exec_ADD == 1'b1) begin
            Flags[4] <= set_flags[0];
          end
          else if(exec_ROT == 1'b1 && rot_bits != 2'b11 && rot_nop == 1'b0) begin
            Flags[4] <= rot_XC;
          end
          if((exec_ADD | exec_CMP) == 1'b1) begin
            Flags[3:0] <= set_flags;
          end
          else if(decodeOPC == 1'b1 && set_exec_ROT == 1'b1) begin
            Flags[1] <= 1'b0;
          end
          else if(exec_DIVU == 1'b1) begin
            if(set_V_Flag == 1'b1) begin
              Flags[3:0] <= 4'b1010;
            end
            else begin
              Flags[3:0] <= {OP1IN[15],flag_z[1],2'b00};
            end
          end
          else if(exec_OR == 1'b1 || exec_AND == 1'b1 || exec_EOR == 1'b1 || exec_MOVE == 1'b1 || exec_swap == 1'b1 || exec_MULU == 1'b1) begin
            Flags[3:0] <= {set_flags[3:2],2'b00};
          end
          else if(exec_ROT == 1'b1) begin
            Flags[3:2] <= set_flags[3:2];
            Flags[0] <= rot_XC;
            if(rot_bits == 2'b00) begin
              //ASL/ASR
              Flags[1] <= (set_flags[3] ^ rot_rot) | Flags[1];
            end
          end
          else if(exec_bits == 1'b1) begin
            Flags[2] <=  ~one_bit_in;
          end
        end
      end
    end
  end

  //---------------------------------------------------------------------------
  // execute opcode
  //---------------------------------------------------------------------------
  always @(clk, reset, OP2out, opcode, fetchOPC, decodeOPC, execOPC, endOPC, nextpass, condition, set_V_flag, trapmake, trapd, interrupt, trap_interrupt, rot_nop, Z_error, c_in, rot_cnt, one_bit_in, bit_number_reg, bit_number, ea_only, get_ea_now, ea_build, datatype, exec_write_back, get_extendedOPC, Flags, SVmode, movem_addr, movem_busy, getbrief, set_exec_AND, set_exec_OR, set_exec_EOR, TG68_PC_dec, c_out, OP1out, micro_state) begin
    TG68_PC_br8 <= 1'b0;
    TG68_PC_brw <= 1'b0;
    TG68_PC_nop <= 1'b0;
    setstate <= 2'b00;
    Regwrena <= 1'b0;
    postadd <= 1'b0;
    presub <= 1'b0;
    movem_presub <= 1'b0;
    setaddsub <= 1'b1;
    setaddrlong <= 1'b0;
    setnextpass <= 1'b0;
    regdirectsource <= 1'b0;
    setdisp <= 1'b0;
    setdispbyte <= 1'b0;
    setdispbrief <= 1'b0;
    setbriefext <= 1'b0;
    setgetbrief <= 1'b0;
    longreaddirect <= 1'b0;
    dest_areg <= 1'b0;
    source_areg <= 1'b0;
    data_is_source <= 1'b0;
    write_back <= 1'b0;
    setstackaddr <= 1'b0;
    writePC <= 1'b0;
    writePC_add <= 1'b0;
    set_TG68_PC_dec <= 1'b0;
    set_directPC <= 1'b0;
    set_exec_ADD <= 1'b0;
    set_exec_OR <= 1'b0;
    set_exec_AND <= 1'b0;
    set_exec_EOR <= 1'b0;
    set_exec_MOVE <= 1'b0;
    set_exec_MOVEQ <= 1'b0;
    set_exec_MOVESR <= 1'b0;
    set_exec_ADDQ <= 1'b0;
    set_exec_CMP <= 1'b0;
    set_exec_ROT <= 1'b0;
    set_exec_EXT <= 1'b0;
    set_exec_CPMAW <= 1'b0;
    OP2out_one <= 1'b0;
    ea_to_pc <= 1'b0;
    ea_build <= 1'b0;
    get_ea_now <= 1'b0;
    rot_bits <= 2'bXX;
    set_rot_nop <= 1'b0;
    set_rot_cnt <= 6'b000001;
    set_movem_busy <= 1'b0;
    set_get_movem_mask <= 1'b0;
    save_memaddr <= 1'b0;
    set_mem_addsub <= 1'b0;
    exec_exg <= 1'b0;
    exec_swap <= 1'b0;
    exec_Bits <= 1'b0;
    set_get_bitnumber <= 1'b0;
    dest_hbits <= 1'b0;
    source_lowbits <= 1'b0;
    set_mem_rega <= 1'b0;
    ea_data_OP1 <= 1'b0;
    ea_only <= 1'b0;
    set_direct_data <= 1'b0;
    set_get_extendedOPC <= 1'b0;
    set_exec_tas <= 1'b0;
    OP1out_zero <= 1'b0;
    use_XZFlag <= 1'b0;
    use_XFlag <= 1'b0;
    set_exec_ABCD <= 1'b0;
    set_exec_SBCD <= 1'b0;
    set_exec_MULU <= 1'b0;
    set_exec_DIVU <= 1'b0;
    set_exec_Scc <= 1'b0;
    trap_illegal <= 1'b0;
    trap_priv <= 1'b0;
    trap_1010 <= 1'b0;
    trap_1111 <= 1'b0;
    trap_trap <= 1'b0;
    trap_trapv <= 1'b0;
    trapmake <= 1'b0;
    set_vectoraddr <= 1'b0;
    writeSR <= 1'b0;
    set_directSR <= 1'b0;
    set_directCCR <= 1'b0;
    set_stop <= 1'b0;
    from_SR <= 1'b0;
    to_SR <= 1'b0;
    from_USP <= 1'b0;
    to_USP <= 1'b0;
    illegal_write_mode <= 1'b0;
    illegal_read_mode <= 1'b0;
    illegal_byteaddr <= 1'b0;
    no_Flags <= 1'b0;
    set_PCmarker <= 1'b0;
    use_SP <= 1'b0;
    set_Z_error <= 1'b0;
    wait_mem_byte <= 1'b0;
    set_movepl <= 1'b0;
    set_movepw <= 1'b0;
    trap_chk <= 1'b0;
    next_micro_state <= idle;
    //----------------------------------------------------------------------------
    //Sourcepass
    //----------------------------------------------------------------------------		
    if(ea_only == 1'b0 && get_ea_now == 1'b1) begin
      setstate <= 2'b10;
    end
    if(ea_build == 1'b1) begin
      case(opcode[5:3])
            //source
      3'b010,3'b011,3'b100 : begin
        // -(An)+
        get_ea_now <= 1'b1;
        setnextpass <= 1'b1;
        if(opcode[4] == 1'b1) begin
          set_mem_rega <= 1'b1;
        end
        else begin
          set_mem_addsub <= 1'b1;
        end
        if(opcode[3] == 1'b1) begin
          //(An)+
          postadd <= 1'b1;
          if(opcode[2:0] == 3'b111) begin
            use_SP <= 1'b1;
          end
        end
        if(opcode[5] == 1'b1) begin
          // -(An)
          presub <= 1'b1;
          if(opcode[2:0] == 3'b111) begin
            use_SP <= 1'b1;
          end
        end
        if(opcode[4:3] != 2'b10) begin
          regwrena <= 1'b1;
        end
      end
      3'b101 : begin
        //(d16,An)
        next_micro_state <= ld_dAn1;
        setgetbrief <= 1'b1;
        set_mem_regA <= 1'b1;
      end
      3'b110 : begin
        //(d8,An,Xn)
        next_micro_state <= ld_AnXn1;
        setgetbrief <= 1'b1;
        set_mem_regA <= 1'b1;
      end
      3'b111 : begin
        case(opcode[2:0])
        3'b000 : begin
          //(xxxx).w
          next_micro_state <= ld_nn;
        end
        3'b001 : begin
          //(xxxx).l
          longreaddirect <= 1'b1;
          next_micro_state <= ld_nn;
        end
        3'b010 : begin
          //(d16,PC)
          next_micro_state <= ld_dAn1;
          setgetbrief <= 1'b1;
          set_PCmarker <= 1'b1;
        end
        3'b011 : begin
          //(d8,PC,Xn)
          next_micro_state <= ld_AnXn1;
          setgetbrief <= 1'b1;
          set_PCmarker <= 1'b1;
        end
        3'b100 : begin
          //#data
          setnextpass <= 1'b1;
          set_direct_data <= 1'b1;
          if(datatype == 2'b10) begin
            longreaddirect <= 1'b1;
          end
        end
        default : begin
        end
        endcase
      end
      default : begin
      end
      endcase
    end
    //----------------------------------------------------------------------------
    //prepere opcode
    //----------------------------------------------------------------------------		
    case(opcode[7:6])
    2'b00 : begin
      datatype <= 2'b00;
      //Byte
    end
    2'b01 : begin
      datatype <= 2'b01;
      //Word
    end
    default : begin
      datatype <= 2'b10;
      //Long
    end
    endcase
    if(execOPC == 1'b1 && endOPC == 1'b0 && exec_write_back == 1'b1) begin
      setstate <= 2'b11;
    end
    //----------------------------------------------------------------------------
    //test illegal mode
    //----------------------------------------------------------------------------	
    if((opcode[5:3] == 3'b111 && opcode[2:1] != 2'b00) || (opcode[5:3] == 3'b001 && datatype == 2'b00)) begin
      illegal_write_mode <= 1'b1;
    end
    if((opcode[5:2] == 4'b1111 && opcode[1:0] != 2'b00) || (opcode[5:3] == 3'b001 && datatype == 2'b00)) begin
      illegal_read_mode <= 1'b1;
    end
    if(opcode[5:3] == 3'b001 && datatype == 2'b00) begin
      illegal_byteaddr <= 1'b1;
    end
    case(opcode[15:12])
        // 0000 ----------------------------------------------------------------------------		
    4'b0000 : begin
      if(opcode[8] == 1'b1 && opcode[5:3] == 3'b001) begin
        //movep
        datatype <= 2'b00;
        //Byte
        use_SP <= 1'b1;
        no_Flags <= 1'b1;
        if(opcode[7] == 1'b0) begin
          set_exec_move <= 1'b1;
          set_movepl <= 1'b1;
        end
        if(decodeOPC == 1'b1) begin
          if(opcode[7] == 1'b0) begin
            set_direct_data <= 1'b1;
          end
          next_micro_state <= movep1;
          setgetbrief <= 1'b1;
          set_mem_regA <= 1'b1;
        end
        if(opcode[7] == 1'b0 && endOPC == 1'b1) begin
          if(opcode[6] == 1'b1) begin
            datatype <= 2'b10;
            //Long
          end
          else begin
            datatype <= 2'b01;
            //Word
          end
          dest_hbits <= 1'b1;
          regwrena <= 1'b1;
        end
      end
      else begin
        if(opcode[8] == 1'b1 || opcode[11:8] == 4'b1000) begin
          //Bits
          if(execOPC == 1'b1 && get_extendedOPC == 1'b0) begin
            if(opcode[7:6] != 2'b00 && endOPC == 1'b1) begin
              regwrena <= 1'b1;
            end
            exec_Bits <= 1'b1;
            ea_data_OP1 <= 1'b1;
          end
          //					IF get_extendedOPC='1' THEN
          //						datatype <= "01";			--Word
          //					ELS
          if(opcode[5:4] == 2'b00) begin
            datatype <= 2'b10;
            //Long
          end
          else begin
            datatype <= 2'b00;
            //Byte
            if(opcode[7:6] != 2'b00) begin
              write_back <= 1'b1;
            end
          end
          if(decodeOPC == 1'b1) begin
            ea_build <= 1'b1;
            if(opcode[8] == 1'b0) begin
              if(opcode[5:4] != 2'b00) begin
                //Dn, An
                set_get_extendedOPC <= 1'b1;
              end
              set_get_bitnumber <= 1'b1;
            end
          end
        end
        else begin
          //andi, ...xxxi	
          if(opcode[11:8] == 4'b0000) begin
            //ORI
            set_exec_OR <= 1'b1;
          end
          if(opcode[11:8] == 4'b0010) begin
            //ANDI
            set_exec_AND <= 1'b1;
          end
          if(opcode[11:8] == 4'b0100 || opcode[11:8] == 4'b0110) begin
            //SUBI, ADDI
            set_exec_ADD <= 1'b1;
          end
          if(opcode[11:8] == 4'b1010) begin
            //EORI
            set_exec_EOR <= 1'b1;
          end
          if(opcode[11:8] == 4'b1100) begin
            //CMPI
            set_exec_CMP <= 1'b1;
          end
          else if(trapmake == 1'b0) begin
            write_back <= 1'b1;
          end
          if(opcode[7] == 1'b0 && opcode[5:0] == 6'b111100 && (set_exec_AND | set_exec_OR | set_exec_EOR) == 1'b1) begin
            //SR
            //					IF opcode(7)='0' AND opcode(5 downto 0)="111100" AND (opcode(11 downto 8)="0010" OR opcode(11 downto 8)="0000" OR opcode(11 downto 8)="1010") THEN		--SR
            if(SVmode == 1'b0 && opcode[6] == 1'b1) begin
              //SR
              trap_priv <= 1'b1;
              trapmake <= 1'b1;
            end
            else begin
              from_SR <= 1'b1;
              to_SR <= 1'b1;
              if(decodeOPC == 1'b1) begin
                setnextpass <= 1'b1;
                set_direct_data <= 1'b1;
              end
            end
          end
          else begin
            if(decodeOPC == 1'b1) begin
              if(opcode[11:8] == 4'b0010 || opcode[11:8] == 4'b0000 || opcode[11:8] == 4'b0100 || opcode[11:8] == 4'b0110 || opcode[11:8] == 4'b1010 || opcode[11:8] == 4'b1100) begin
                //ADDI, EORI, CMPI
                //						IF (set_exec_AND OR set_exec_OR OR set_exec_ADD  	--ANDI, ORI, SUBI
                //						   OR set_exec_EOR OR set_exec_CMP)='1' THEN	--ADDI, EORI, CMPI
                next_micro_state <= andi;
                set_direct_data <= 1'b1;
                if(datatype == 2'b10) begin
                  longreaddirect <= 1'b1;
                end
              end
            end
            if(execOPC == 1'b1) begin
              ea_data_OP1 <= 1'b1;
              if(opcode[11:8] != 4'b1100) begin
                //CMPI 
                if(endOPC == 1'b1) begin
                  Regwrena <= 1'b1;
                end
              end
              if(opcode[11:8] == 4'b1100 || opcode[11:8] == 4'b0100) begin
                //CMPI, SUBI
                setaddsub <= 1'b0;
              end
            end
          end
        end
      end
      // 0001, 0010, 0011 -----------------------------------------------------------------		
    end
    4'b0001,4'b0010,4'b0011 : begin
      //move.b, move.l, move.w
      set_exec_MOVE <= 1'b1;
      if(opcode[8:6] == 3'b001) begin
        no_Flags <= 1'b1;
      end
      if(opcode[5:4] == 2'b00) begin
        //Dn, An
        regdirectsource <= 1'b1;
      end
      case(opcode[13:12])
      2'b01 : begin
        datatype <= 2'b00;
        //Byte
      end
      2'b10 : begin
        datatype <= 2'b10;
        //Long
      end
      default : begin
        datatype <= 2'b01;
        //Word
      end
      endcase
      source_lowbits <= 1'b1;
      // Dn=>  An=>
      if(opcode[3] == 1'b1) begin
        source_areg <= 1'b1;
      end
      if(getbrief == 1'b1 && nextpass == 1'b1) begin
        // =>(d16,An)  =>(d8,An,Xn)
        set_mem_rega <= 1'b1;
      end
      if(execOPC == 1'b1 && opcode[8:7] == 2'b00) begin
        Regwrena <= 1'b1;
      end
      if(nextpass == 1'b1 || execOPC == 1'b1 || opcode[5:4] == 2'b00) begin
        dest_hbits <= 1'b1;
        if(opcode[8:6] != 3'b000) begin
          dest_areg <= 1'b1;
        end
      end
      if(decodeOPC == 1'b1) begin
        ea_build <= 1'b1;
      end
      if(micro_state == idle && (nextpass == 1'b1 || (opcode[5:4] == 2'b00 && decodeOPC == 1'b1))) begin
        case(opcode[8:6])
                //destination
        //						WHEN "000" =>						--Dn
        //						WHEN "001" =>						--An
        3'b010,3'b011,3'b100 : begin
          //destination -(an)+
          if(opcode[7] == 1'b1) begin
            set_mem_rega <= 1'b1;
          end
          else begin
            set_mem_addsub <= 1'b1;
          end
          if(opcode[6] == 1'b1) begin
            //(An)+
            postadd <= 1'b1;
            if(opcode[11:9] == 3'b111) begin
              use_SP <= 1'b1;
            end
          end
          if(opcode[8] == 1'b1) begin
            // -(An)
            presub <= 1'b1;
            if(opcode[11:9] == 3'b111) begin
              use_SP <= 1'b1;
            end
          end
          if(opcode[7:6] != 2'b10) begin
            regwrena <= 1'b1;
          end
          setstate <= 2'b11;
          next_micro_state <= nop;
        end
        3'b101 : begin
          //(d16,An)
          next_micro_state <= st_dAn1;
          set_mem_regA <= 1'b1;
          setgetbrief <= 1'b1;
        end
        3'b110 : begin
          //(d8,An,Xn)
          next_micro_state <= st_AnXn1;
          set_mem_regA <= 1'b1;
          setgetbrief <= 1'b1;
        end
        3'b111 : begin
          case(opcode[11:9])
          3'b000 : begin
            //(xxxx).w
            next_micro_state <= st_nn;
          end
          3'b001 : begin
            //(xxxx).l
            longreaddirect <= 1'b1;
            next_micro_state <= st_nn;
          end
          default : begin
          end
          endcase
        end
        default : begin
        end
        endcase
      end
      // 0100 ----------------------------------------------------------------------------		
    end
    4'b0100 : begin
      //rts_group
      if(opcode[8] == 1'b1) begin
        //lea
        if(opcode[6] == 1'b1) begin
          //lea
          if(opcode[7] == 1'b1) begin
            ea_only <= 1'b1;
            if(opcode[5:3] == 3'b010) begin
              //lea (Am),An
              set_exec_move <= 1'b1;
              no_Flags <= 1'b1;
              dest_areg <= 1'b1;
              dest_hbits <= 1'b1;
              source_lowbits <= 1'b1;
              source_areg <= 1'b1;
              if(execOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
            else begin
              if(decodeOPC == 1'b1) begin
                ea_build <= 1'b1;
              end
            end
            if(get_ea_now == 1'b1) begin
              dest_areg <= 1'b1;
              dest_hbits <= 1'b1;
              regwrena <= 1'b1;
            end
          end
          else begin
            trap_illegal <= 1'b1;
            trapmake <= 1'b1;
          end
        end
        else begin
          //chk
          if(opcode[7] == 1'b1) begin
            set_exec_ADD <= 1'b1;
            if(decodeOPC == 1'b1) begin
              ea_build <= 1'b1;
            end
            datatype <= 2'b01;
            //Word
            if(execOPC == 1'b1) begin
              setaddsub <= 1'b0;
              //first alternative
              ea_data_OP1 <= 1'b1;
              if(c_out[1] == 1'b1 || OP1out[15] == 1'b1 || OP2out[15] == 1'b1) begin
                //				trap_chk <= '1';	--first I must change the Trap System
                //				trapmake <= '1';
              end
              //second alternative									
              //								IF (c_out(1)='0' AND flag_z(1)='0') OR OP1out(15)='1' OR OP2out(15)='1' THEN
              //					--				trap_chk <= '1';	--first I must change the Trap System
              //					--				trapmake <= '1';
              //								END IF;
              //								dest_hbits <= '1';
              //								source_lowbits <='1';
            end
          end
          else begin
            trap_illegal <= 1'b1;
            // chk long for 68020
            trapmake <= 1'b1;
          end
        end
      end
      else begin
        case(opcode[11:9])
        3'b000 : begin
          if(decodeOPC == 1'b1) begin
            ea_build <= 1'b1;
          end
          if(opcode[7:6] == 2'b11) begin
            //move from SR
            set_exec_MOVESR <= 1'b1;
            datatype <= 2'b01;
            write_back <= 1'b1;
            // im 68000 wird auch erst gelesen
            if(execOPC == 1'b1) begin
              if(endOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
          end
          else begin
            //negx
            use_XFlag <= 1'b1;
            write_back <= 1'b1;
            set_exec_ADD <= 1'b1;
            setaddsub <= 1'b0;
            if(execOPC == 1'b1) begin
              source_lowbits <= 1'b1;
              OP1out_zero <= 1'b1;
              if(endOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
          end
        end
        3'b001 : begin
          if(opcode[7:6] == 2'b11) begin
            //move from CCR 68010
            trap_illegal <= 1'b1;
            trapmake <= 1'b1;
          end
          else begin
            //clr
            if(decodeOPC == 1'b1) begin
              ea_build <= 1'b1;
            end
            write_back <= 1'b1;
            set_exec_AND <= 1'b1;
            if(execOPC == 1'b1) begin
              OP1out_zero <= 1'b1;
              if(endOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
          end
        end
        3'b010 : begin
          if(decodeOPC == 1'b1) begin
            ea_build <= 1'b1;
          end
          if(opcode[7:6] == 2'b11) begin
            //move to CCR
            set_exec_MOVE <= 1'b1;
            datatype <= 2'b01;
            if(execOPC == 1'b1) begin
              source_lowbits <= 1'b1;
              to_SR <= 1'b1;
            end
          end
          else begin
            //neg
            write_back <= 1'b1;
            set_exec_ADD <= 1'b1;
            setaddsub <= 1'b0;
            if(execOPC == 1'b1) begin
              source_lowbits <= 1'b1;
              OP1out_zero <= 1'b1;
              if(endOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
          end
        end
        3'b011 : begin
          //not, move toSR
          if(opcode[7:6] == 2'b11) begin
            //move to SR
            if(SVmode == 1'b1) begin
              if(decodeOPC == 1'b1) begin
                ea_build <= 1'b1;
              end
              set_exec_MOVE <= 1'b1;
              datatype <= 2'b01;
              if(execOPC == 1'b1) begin
                source_lowbits <= 1'b1;
                to_SR <= 1'b1;
              end
            end
            else begin
              trap_priv <= 1'b1;
              trapmake <= 1'b1;
            end
          end
          else begin
            //not
            if(decodeOPC == 1'b1) begin
              ea_build <= 1'b1;
            end
            write_back <= 1'b1;
            set_exec_EOR <= 1'b1;
            if(execOPC == 1'b1) begin
              OP2out_one <= 1'b1;
              ea_data_OP1 <= 1'b1;
              if(endOPC == 1'b1) begin
                Regwrena <= 1'b1;
              end
            end
          end
        end
        3'b100,3'b110 : begin
          if(opcode[7] == 1'b1) begin
            //movem, ext
            if(opcode[5:3] == 3'b000 && opcode[10] == 1'b0) begin
              //ext
              source_lowbits <= 1'b1;
              if(decodeOPC == 1'b1) begin
                set_exec_EXT <= 1'b1;
                set_exec_move <= 1'b1;
              end
              if(opcode[6] == 1'b0) begin
                datatype <= 2'b01;
                //WORD
              end
              if(execOPC == 1'b1) begin
                regwrena <= 1'b1;
              end
            end
            else begin
              //movem
              //								IF opcode(11 downto 7)="10001" OR opcode(11 downto 7)="11001" THEN	--MOVEM
              ea_only <= 1'b1;
              if(decodeOPC == 1'b1) begin
                datatype <= 2'b01;
                //Word
                set_get_movem_mask <= 1'b1;
                set_get_extendedOPC <= 1'b1;
                if(opcode[5:3] == 3'b010 || opcode[5:3] == 3'b011 || opcode[5:3] == 3'b100) begin
                  set_mem_rega <= 1'b1;
                  setstate <= 2'b01;
                  if(opcode[10] == 1'b0) begin
                    set_movem_busy <= 1'b1;
                  end
                  else begin
                    next_micro_state <= movem;
                  end
                end
                else begin
                  ea_build <= 1'b1;
                end
              end
              else begin
                if(opcode[6] == 1'b0) begin
                  datatype <= 2'b01;
                  //Word
                end
              end
              if(execOPC == 1'b1) begin
                if(opcode[5:3] == 3'b100 || opcode[5:3] == 3'b011) begin
                  regwrena <= 1'b1;
                  save_memaddr <= 1'b1;
                end
              end
              if(get_ea_now == 1'b1) begin
                set_movem_busy <= 1'b1;
                if(opcode[10] == 1'b0) begin
                  setstate <= 2'b01;
                end
                else begin
                  setstate <= 2'b10;
                end
              end
              if(opcode[5:3] == 3'b100) begin
                movem_presub <= 1'b1;
              end
              if(movem_addr == 1'b1) begin
                if(opcode[10] == 1'b1) begin
                  regwrena <= 1'b1;
                end
              end
              if(movem_busy == 1'b1) begin
                if(opcode[10] == 1'b0) begin
                  setstate <= 2'b11;
                end
                else begin
                  setstate <= 2'b10;
                end
              end
            end
          end
          else begin
            if(opcode[10] == 1'b1) begin
              //MUL, DIV 68020
              trap_illegal <= 1'b1;
              trapmake <= 1'b1;
            end
            else begin
              //pea, swap
              if(opcode[6] == 1'b1) begin
                datatype <= 2'b10;
                if(opcode[5:3] == 3'b000) begin
                  //swap
                  if(execOPC == 1'b1) begin
                    exec_swap <= 1'b1;
                    regwrena <= 1'b1;
                  end
                end
                else if(opcode[5:3] == 3'b001) begin
                  //bkpt
                end
                else begin
                  //pea
                  ea_only <= 1'b1;
                  if(decodeOPC == 1'b1) begin
                    ea_build <= 1'b1;
                  end
                  if(nextpass == 1'b1 && micro_state == idle) begin
                    presub <= 1'b1;
                    setstackaddr <= 1'b1;
                    set_mem_addsub <= 1'b1;
                    setstate <= 2'b11;
                    next_micro_state <= nop;
                  end
                  if(get_ea_now == 1'b1) begin
                    setstate <= 2'b01;
                  end
                end
              end
              else begin
                //nbcd	
                if(decodeOPC == 1'b1) begin
                  //nbcd
                  ea_build <= 1'b1;
                end
                use_XFlag <= 1'b1;
                write_back <= 1'b1;
                set_exec_ADD <= 1'b1;
                set_exec_SBCD <= 1'b1;
                if(execOPC == 1'b1) begin
                  source_lowbits <= 1'b1;
                  OP1out_zero <= 1'b1;
                  if(endOPC == 1'b1) begin
                    Regwrena <= 1'b1;
                  end
                end
              end
            end
          end
        end
        3'b101 : begin
          //tst, tas
          if(opcode[7:2] == 6'b111111) begin
            //4AFC illegal
            trap_illegal <= 1'b1;
            trapmake <= 1'b1;
          end
          else begin
            if(decodeOPC == 1'b1) begin
              ea_build <= 1'b1;
            end
            if(execOPC == 1'b1) begin
              dest_hbits <= 1'b1;
              //for Flags
              source_lowbits <= 1'b1;
              //						IF opcode(3)='1' THEN			--MC68020...
              //							source_areg <= '1';
              //						END IF;
            end
            set_exec_MOVE <= 1'b1;
            if(opcode[7:6] == 2'b11) begin
              //tas
              set_exec_tas <= 1'b1;
              write_back <= 1'b1;
              datatype <= 2'b00;
              //Byte
              if(execOPC == 1'b1 && endOPC == 1'b1) begin
                regwrena <= 1'b1;
              end
            end
          end
          //						WHEN "110"=>
        end
        3'b111 : begin
          //4EXX
          if(opcode[7] == 1'b1) begin
            //jsr, jmp
            datatype <= 2'b10;
            ea_only <= 1'b1;
            if(nextpass == 1'b1 && micro_state == idle) begin
              presub <= 1'b1;
              setstackaddr <= 1'b1;
              set_mem_addsub <= 1'b1;
              setstate <= 2'b11;
              next_micro_state <= nop;
            end
            if(decodeOPC == 1'b1) begin
              ea_build <= 1'b1;
            end
            if(get_ea_now == 1'b1) begin
              //jsr
              if(opcode[6] == 1'b0) begin
                setstate <= 2'b01;
              end
              ea_to_pc <= 1'b1;
              if(opcode[5:1] == 5'b11100) begin
                writePC_add <= 1'b1;
              end
              else begin
                writePC <= 1'b1;
              end
            end
          end
          else begin
            //
            case(opcode[6:0])
            7'b1000000,7'b1000001,7'b1000010,7'b1000011,7'b1000100,7'b1000101,7'b1000110,7'b1000111,7'b1001000,7'b1001001,7'b1001010,7'b1001011,7'b1001100,7'b1001101,7'b1001110,7'b1001111 : begin
              //trap
              trap_trap <= 1'b1;
              trapmake <= 1'b1;
            end
            7'b1010000,7'b1010001,7'b1010010,7'b1010011,7'b1010100,7'b1010101,7'b1010110,7'b1010111 : begin
              //link
              datatype <= 2'b10;
              if(decodeOPC == 1'b1) begin
                next_micro_state <= link;
                set_exec_MOVE <= 1'b1;
                //für displacement
                presub <= 1'b1;
                setstackaddr <= 1'b1;
                set_mem_addsub <= 1'b1;
                source_lowbits <= 1'b1;
                source_areg <= 1'b1;
              end
              if(execOPC == 1'b1) begin
                setstackaddr <= 1'b1;
                regwrena <= 1'b1;
              end
            end
            7'b1011000,7'b1011001,7'b1011010,7'b1011011,7'b1011100,7'b1011101,7'b1011110,7'b1011111 : begin
              //unlink
              datatype <= 2'b10;
              if(decodeOPC == 1'b1) begin
                setstate <= 2'b10;
                set_mem_rega <= 1'b1;
              end
              else if(execOPC == 1'b1) begin
                regwrena <= 1'b1;
                exec_exg <= 1'b1;
              end
              else begin
                setstackaddr <= 1'b1;
                regwrena <= 1'b1;
                get_ea_now <= 1'b1;
                ea_only <= 1'b1;
              end
            end
            7'b1100000,7'b1100001,7'b1100010,7'b1100011,7'b1100100,7'b1100101,7'b1100110,7'b1100111 : begin
              //move An,USP
              if(SVmode == 1'b1) begin
                no_Flags <= 1'b1;
                to_USP <= 1'b1;
                setstackaddr <= 1'b1;
                source_lowbits <= 1'b1;
                source_areg <= 1'b1;
                set_exec_MOVE <= 1'b1;
                datatype <= 2'b10;
                if(execOPC == 1'b1) begin
                  regwrena <= 1'b1;
                end
              end
              else begin
                trap_priv <= 1'b1;
                trapmake <= 1'b1;
              end
            end
            7'b1101000,7'b1101001,7'b1101010,7'b1101011,7'b1101100,7'b1101101,7'b1101110,7'b1101111 : begin
              //move USP,An
              if(SVmode == 1'b1) begin
                no_Flags <= 1'b1;
                from_USP <= 1'b1;
                set_exec_MOVE <= 1'b1;
                datatype <= 2'b10;
                if(execOPC == 1'b1) begin
                  regwrena <= 1'b1;
                end
              end
              else begin
                trap_priv <= 1'b1;
                trapmake <= 1'b1;
              end
            end
            7'b1110000 : begin
              //reset
              if(SVmode == 1'b0) begin
                trap_priv <= 1'b1;
                trapmake <= 1'b1;
              end
            end
            7'b1110001 : begin
              //nop
            end
            7'b1110010 : begin
              //stop
              if(SVmode == 1'b0) begin
                trap_priv <= 1'b1;
                trapmake <= 1'b1;
              end
              else begin
                if(decodeOPC == 1'b1) begin
                  setnextpass <= 1'b1;
                  set_directSR <= 1'b1;
                  set_stop <= 1'b1;
                end
              end
            end
            7'b1110011 : begin
              //rte
              if(SVmode == 1'b1) begin
                if(decodeOPC == 1'b1) begin
                  datatype <= 2'b01;
                  setstate <= 2'b10;
                  postadd <= 1'b1;
                  setstackaddr <= 1'b1;
                  set_mem_rega <= 1'b1;
                  set_directSR <= 1'b1;
                  next_micro_state <= rte;
                end
              end
              else begin
                trap_priv <= 1'b1;
                trapmake <= 1'b1;
              end
            end
            7'b1110101 : begin
              //rts
              if(decodeOPC == 1'b1) begin
                datatype <= 2'b10;
                setstate <= 2'b10;
                postadd <= 1'b1;
                setstackaddr <= 1'b1;
                set_mem_rega <= 1'b1;
                set_directPC <= 1'b1;
                next_micro_state <= nop;
              end
            end
            7'b1110110 : begin
              //trapv
              if(Flags[1] == 1'b1) begin
                trap_trapv <= 1'b1;
                trapmake <= 1'b1;
              end
            end
            7'b1110111 : begin
              //rtr
              if(decodeOPC == 1'b1) begin
                datatype <= 2'b01;
                setstate <= 2'b10;
                postadd <= 1'b1;
                setstackaddr <= 1'b1;
                set_mem_rega <= 1'b1;
                set_directCCR <= 1'b1;
                next_micro_state <= rte;
              end
            end
            default : begin
              trap_illegal <= 1'b1;
              trapmake <= 1'b1;
            end
            endcase
          end
        end
        default : begin
        end
        endcase
      end
      // 0101 ----------------------------------------------------------------------------		
    end
    4'b0101 : begin
      //subq, addq	
      if(opcode[7:6] == 2'b11) begin
        //dbcc
        if(opcode[5:3] == 3'b001) begin
          //dbcc
          datatype <= 2'b01;
          //Word
          if(decodeOPC == 1'b1) begin
            next_micro_state <= nop;
            OP2out_one <= 1'b1;
            if(condition == 1'b0) begin
              Regwrena <= 1'b1;
              if(c_in[2] == 1'b1) begin
                next_micro_state <= dbcc1;
              end
            end
            data_is_source <= 1'b1;
          end
        end
        else begin
          //Scc
          datatype <= 2'b00;
          //Byte
          write_back <= 1'b1;
          if(decodeOPC == 1'b1) begin
            ea_build <= 1'b1;
          end
          if(condition == 1'b0) begin
            set_exec_Scc <= 1'b1;
          end
          if(execOPC == 1'b1) begin
            if(condition == 1'b1) begin
              OP2out_one <= 1'b1;
              exec_EXG <= 1'b1;
            end
            else begin
              OP1out_zero <= 1'b1;
            end
            if(endOPC == 1'b1) begin
              Regwrena <= 1'b1;
            end
          end
        end
      end
      else begin
        //addq, subq
        if(decodeOPC == 1'b1) begin
          ea_build <= 1'b1;
        end
        if(opcode[5:3] == 3'b001) begin
          no_Flags <= 1'b1;
        end
        write_back <= 1'b1;
        set_exec_ADDQ <= 1'b1;
        set_exec_ADD <= 1'b1;
        if(execOPC == 1'b1) begin
          ea_data_OP1 <= 1'b1;
          if(endOPC == 1'b1) begin
            Regwrena <= 1'b1;
          end
          if(opcode[8] == 1'b1) begin
            setaddsub <= 1'b0;
          end
        end
      end
      // 0110 ----------------------------------------------------------------------------		
    end
    4'b0110 : begin
      //bra,bsr,bcc
      datatype <= 2'b10;
      if(micro_state == idle) begin
        if(opcode[11:8] == 4'b0001) begin
          //bsr
          if(opcode[7:0] == 8'b00000000) begin
            next_micro_state <= bsr1;
          end
          else begin
            next_micro_state <= bsr2;
            setstate <= 2'b01;
          end
          presub <= 1'b1;
          setstackaddr <= 1'b1;
          set_mem_addsub <= 1'b1;
        end
        else begin
          //bra
          if(opcode[7:0] == 8'b00000000) begin
            next_micro_state <= bra1;
          end
          if(condition == 1'b1) begin
            TG68_PC_br8 <= 1'b1;
          end
        end
      end
      // 0111 ----------------------------------------------------------------------------		
    end
    4'b0111 : begin
      //moveq
      if(opcode[8] == 1'b0) begin
        if(trap_interrupt == 1'b0) begin
          datatype <= 2'b10;
          //Long
          Regwrena <= 1'b1;
          set_exec_MOVEQ <= 1'b1;
          set_exec_MOVE <= 1'b1;
          dest_hbits <= 1'b1;
        end
      end
      else begin
        trap_illegal <= 1'b1;
        trapmake <= 1'b1;
      end
      // 1000 ----------------------------------------------------------------------------		
    end
    4'b1000 : begin
      //or	
      if(opcode[7:6] == 2'b11) begin
        //divu, divs
        if(opcode[5:4] == 2'b00) begin
          //Dn, An
          regdirectsource <= 1'b1;
        end
        if((micro_state == idle && nextpass == 1'b1) || (opcode[5:4] == 2'b00 && decodeOPC == 1'b1)) begin
          set_exec_DIVU <= 1'b1;
          setstate <= 2'b01;
          next_micro_state <= div1;
        end
        if(decodeOPC == 1'b1) begin
          ea_build <= 1'b1;
        end
        if(execOPC == 1'b1 && z_error == 1'b0 && set_V_Flag == 1'b0) begin
          regwrena <= 1'b1;
        end
        if((micro_state != idle && nextpass == 1'b1) || execOPC == 1'b1) begin
          dest_hbits <= 1'b1;
          source_lowbits <= 1'b1;
        end
        else begin
          datatype <= 2'b01;
        end
      end
      else if(opcode[8] == 1'b1 && opcode[5:4] == 2'b00) begin
        //sbcd, pack , unpack
        if(opcode[7:6] == 2'b00) begin
          //sbcd
          use_XZFlag <= 1'b1;
          set_exec_ADD <= 1'b1;
          set_exec_SBCD <= 1'b1;
          if(opcode[3] == 1'b1) begin
            write_back <= 1'b1;
            if(decodeOPC == 1'b1) begin
              set_direct_data <= 1'b1;
              setstate <= 2'b10;
              set_mem_addsub <= 1'b1;
              presub <= 1'b1;
              next_micro_state <= op_AxAy;
            end
          end
          if(execOPC == 1'b1) begin
            ea_data_OP1 <= 1'b1;
            dest_hbits <= 1'b1;
            source_lowbits <= 1'b1;
            if(endOPC == 1'b1) begin
              Regwrena <= 1'b1;
            end
          end
        end
        else begin
          //pack, unpack
          trap_illegal <= 1'b1;
          trapmake <= 1'b1;
        end
      end
      else begin
        //or
        set_exec_OR <= 1'b1;
        if(opcode[8] == 1'b1) begin
          write_back <= 1'b1;
        end
        if(decodeOPC == 1'b1) begin
          ea_build <= 1'b1;
        end
        if(execOPC == 1'b1) begin
          if(endOPC == 1'b1) begin
            Regwrena <= 1'b1;
          end
          if(opcode[8] == 1'b1) begin
            ea_data_OP1 <= 1'b1;
          end
          else begin
            dest_hbits <= 1'b1;
            source_lowbits <= 1'b1;
            if(opcode[3] == 1'b1) begin
              source_areg <= 1'b1;
            end
          end
        end
      end
      // 1001, 1101 -----------------------------------------------------------------------		
    end
    4'b1001,4'b1101 : begin
      //sub, add	
      set_exec_ADD <= 1'b1;
      if(decodeOPC == 1'b1) begin
        ea_build <= 1'b1;
      end
      if(opcode[8:6] == 3'b011) begin
        //adda.w, suba.w
        datatype <= 2'b01;
        //Word
      end
      if(execOPC == 1'b1) begin
        if(endOPC == 1'b1) begin
          Regwrena <= 1'b1;
        end
        if(opcode[14] == 1'b0) begin
          setaddsub <= 1'b0;
        end
      end
      if(opcode[8] == 1'b1 && opcode[5:4] == 2'b00 && opcode[7:6] != 2'b11) begin
        //addx, subx
        use_XZFlag <= 1'b1;
        if(opcode[3] == 1'b1) begin
          write_back <= 1'b1;
          if(decodeOPC == 1'b1) begin
            set_direct_data <= 1'b1;
            setstate <= 2'b10;
            set_mem_addsub <= 1'b1;
            presub <= 1'b1;
            next_micro_state <= op_AxAy;
          end
        end
        if(execOPC == 1'b1) begin
          ea_data_OP1 <= 1'b1;
          dest_hbits <= 1'b1;
          source_lowbits <= 1'b1;
        end
      end
      else begin
        //sub, add
        if(opcode[8] == 1'b1 && opcode[7:6] != 2'b11) begin
          write_back <= 1'b1;
        end
        if(execOPC == 1'b1) begin
          if(opcode[7:6] == 2'b11) begin
            //adda, suba
            no_Flags <= 1'b1;
            dest_areg <= 1'b1;
            dest_hbits <= 1'b1;
            source_lowbits <= 1'b1;
            if(opcode[3] == 1'b1) begin
              source_areg <= 1'b1;
            end
          end
          else begin
            if(opcode[8] == 1'b1) begin
              ea_data_OP1 <= 1'b1;
            end
            else begin
              dest_hbits <= 1'b1;
              source_lowbits <= 1'b1;
              if(opcode[3] == 1'b1) begin
                source_areg <= 1'b1;
              end
            end
          end
        end
      end
      // 1010 ----------------------------------------------------------------------------		
    end
    4'b1010 : begin
      //Trap 1010
      trap_1010 <= 1'b1;
      trapmake <= 1'b1;
      // 1011 ----------------------------------------------------------------------------		
    end
    4'b1011 : begin
      //eor, cmp
      if(decodeOPC == 1'b1) begin
        ea_build <= 1'b1;
      end
      if(opcode[8:6] == 3'b011) begin
        //cmpa.w
        datatype <= 2'b01;
        //Word
        set_exec_CPMAW <= 1'b1;
      end
      if(opcode[8] == 1'b1 && opcode[5:3] == 3'b001 && opcode[7:6] != 2'b11) begin
        //cmpm
        set_exec_CMP <= 1'b1;
        if(decodeOPC == 1'b1) begin
          set_direct_data <= 1'b1;
          setstate <= 2'b10;
          set_mem_rega <= 1'b1;
          postadd <= 1'b1;
          next_micro_state <= cmpm;
        end
        if(execOPC == 1'b1) begin
          ea_data_OP1 <= 1'b1;
          setaddsub <= 1'b0;
        end
      end
      else begin
        //sub, add
        if(opcode[8] == 1'b1 && opcode[7:6] != 2'b11) begin
          //eor
          set_exec_EOR <= 1'b1;
          write_back <= 1'b1;
        end
        else begin
          //cmp
          set_exec_CMP <= 1'b1;
        end
        if(execOPC == 1'b1) begin
          if(opcode[8] == 1'b1 && opcode[7:6] != 2'b11) begin
            //eor
            ea_data_OP1 <= 1'b1;
            if(endOPC == 1'b1) begin
              Regwrena <= 1'b1;
            end
          end
          else begin
            //cmp
            source_lowbits <= 1'b1;
            if(opcode[3] == 1'b1) begin
              source_areg <= 1'b1;
            end
            if(opcode[7:6] == 2'b11) begin
              //cmpa
              dest_areg <= 1'b1;
            end
            dest_hbits <= 1'b1;
            setaddsub <= 1'b0;
          end
        end
      end
      // 1100 ----------------------------------------------------------------------------		
    end
    4'b1100 : begin
      //and, exg
      if(opcode[7:6] == 2'b11) begin
        //mulu, muls
        if(opcode[5:4] == 2'b00) begin
          //Dn, An
          regdirectsource <= 1'b1;
        end
        if((micro_state == idle && nextpass == 1'b1) || (opcode[5:4] == 2'b00 && decodeOPC == 1'b1)) begin
          set_exec_MULU <= 1'b1;
          setstate <= 2'b01;
          next_micro_state <= mul1;
        end
        if(decodeOPC == 1'b1) begin
          ea_build <= 1'b1;
        end
        if(execOPC == 1'b1) begin
          regwrena <= 1'b1;
        end
        if((micro_state != idle && nextpass == 1'b1) || execOPC == 1'b1) begin
          dest_hbits <= 1'b1;
          source_lowbits <= 1'b1;
        end
        else begin
          datatype <= 2'b01;
        end
      end
      else if(opcode[8] == 1'b1 && opcode[5:4] == 2'b00) begin
        //exg, abcd
        if(opcode[7:6] == 2'b00) begin
          //abcd
          use_XZFlag <= 1'b1;
          //						datatype <= "00";		--ist schon default
          set_exec_ADD <= 1'b1;
          set_exec_ABCD <= 1'b1;
          if(opcode[3] == 1'b1) begin
            write_back <= 1'b1;
            if(decodeOPC == 1'b1) begin
              set_direct_data <= 1'b1;
              setstate <= 2'b10;
              set_mem_addsub <= 1'b1;
              presub <= 1'b1;
              next_micro_state <= op_AxAy;
            end
          end
          if(execOPC == 1'b1) begin
            ea_data_OP1 <= 1'b1;
            dest_hbits <= 1'b1;
            source_lowbits <= 1'b1;
            if(endOPC == 1'b1) begin
              Regwrena <= 1'b1;
            end
          end
        end
        else begin
          //exg
          datatype <= 2'b10;
          regwrena <= 1'b1;
          if(opcode[6] == 1'b1 && opcode[3] == 1'b1) begin
            dest_areg <= 1'b1;
            source_areg <= 1'b1;
          end
          if(decodeOPC == 1'b1) begin
            set_mem_rega <= 1'b1;
            exec_exg <= 1'b1;
          end
          else begin
            save_memaddr <= 1'b1;
            dest_hbits <= 1'b1;
          end
        end
      end
      else begin
        //and
        set_exec_AND <= 1'b1;
        if(opcode[8] == 1'b1) begin
          write_back <= 1'b1;
        end
        if(decodeOPC == 1'b1) begin
          ea_build <= 1'b1;
        end
        if(execOPC == 1'b1) begin
          if(endOPC == 1'b1) begin
            Regwrena <= 1'b1;
          end
          if(opcode[8] == 1'b1) begin
            ea_data_OP1 <= 1'b1;
          end
          else begin
            dest_hbits <= 1'b1;
            source_lowbits <= 1'b1;
            if(opcode[3] == 1'b1) begin
              source_areg <= 1'b1;
            end
          end
        end
      end
      // 1110 ----------------------------------------------------------------------------		
    end
    4'b1110 : begin
      //rotation	
      set_exec_ROT <= 1'b1;
      if(opcode[7:6] == 2'b11) begin
        datatype <= 2'b01;
        rot_bits <= opcode[10:9];
        ea_data_OP1 <= 1'b1;
        write_back <= 1'b1;
      end
      else begin
        rot_bits <= opcode[4:3];
        data_is_source <= 1'b1;
      end
      if(decodeOPC == 1'b1) begin
        if(opcode[7:6] == 2'b11) begin
          ea_build <= 1'b1;
        end
        else begin
          if(opcode[5] == 1'b1) begin
            if(OP2out[5:0] != 6'b000000) begin
              set_rot_cnt <= OP2out[5:0];
            end
            else begin
              set_rot_nop <= 1'b1;
            end
          end
          else begin
            set_rot_cnt[2:0] <= opcode[11:9];
            if(opcode[11:9] == 3'b000) begin
              set_rot_cnt[3] <= 1'b1;
            end
            else begin
              set_rot_cnt[3] <= 1'b0;
            end
          end
        end
      end
      if(opcode[7:6] != 2'b11) begin
        if(execOPC == 1'b1 && rot_nop == 1'b0) begin
          Regwrena <= 1'b1;
          set_rot_cnt <= rot_cnt - 1;
        end
      end
      //      ----------------------------------------------------------------------------		
    end
    default : begin
      trap_1111 <= 1'b1;
      trapmake <= 1'b1;
    end
    endcase
  end

  //---------------------------------------------------------------------------
  // execute microcode
  //---------------------------------------------------------------------------
  always @(clk, reset, OP2out, opcode, fetchOPC, decodeOPC, execOPC, endOPC, nextpass, condition, set_V_flag, trapmake, trapd, interrupt, trap_interrupt, rot_nop, Z_error, c_in, rot_cnt, one_bit_in, bit_number_reg, bit_number, ea_only, get_ea_now, ea_build, datatype, exec_write_back, get_extendedOPC, Flags, SVmode, movem_addr, movem_busy, getbrief, set_exec_AND, set_exec_OR, set_exec_EOR, TG68_PC_dec, c_out, OP1out, micro_state) begin
    if(Z_error == 1'b1) begin
      // divu by zero
      trapmake <= 1'b1;
      //wichtig für USP
      if(trapd == 1'b0) begin
        writePC <= 1'b1;
      end
    end
    if(trapmake == 1'b1 && trapd == 1'b0) begin
      next_micro_state <= trap1;
      presub <= 1'b1;
      setstackaddr <= 1'b1;
      set_mem_addsub <= 1'b1;
      setstate <= 2'b11;
      datatype <= 2'b10;
    end
    if(interrupt == 1'b1) begin
      next_micro_state <= int1;
      setstate <= 2'b10;
      //			datatype <= "01";		--wirkt sich auf Flags aus
    end
  end

  always @(posedge clk, posedge reset, posedge OP2out, posedge opcode, posedge fetchOPC, posedge decodeOPC, posedge execOPC, posedge endOPC, posedge nextpass, posedge condition, posedge set_V_flag, posedge trapmake, posedge trapd, posedge interrupt, posedge trap_interrupt, posedge rot_nop, posedge Z_error, posedge c_in, posedge rot_cnt, posedge one_bit_in, posedge bit_number_reg, posedge bit_number, posedge ea_only, posedge get_ea_now, posedge ea_build, posedge datatype, posedge exec_write_back, posedge get_extendedOPC, posedge Flags, posedge SVmode, posedge movem_addr, posedge movem_busy, posedge getbrief, posedge set_exec_AND, posedge set_exec_OR, posedge set_exec_EOR, posedge TG68_PC_dec, posedge c_out, posedge OP1out, posedge micro_state) begin
    if(reset == 1'b0) begin
      micro_state <= init1;
    end else begin
      if(clkena == 1'b1) begin
        trapd <= trapmake;
        if(fetchOPC == 1'b1) begin
          micro_state <= idle;
        end
        else begin
          micro_state <= next_micro_state;
        end
      end
    end
  end

  always @(clk, reset, OP2out, opcode, fetchOPC, decodeOPC, execOPC, endOPC, nextpass, condition, set_V_flag, trapmake, trapd, interrupt, trap_interrupt, rot_nop, Z_error, c_in, rot_cnt, one_bit_in, bit_number_reg, bit_number, ea_only, get_ea_now, ea_build, datatype, exec_write_back, get_extendedOPC, Flags, SVmode, movem_addr, movem_busy, getbrief, set_exec_AND, set_exec_OR, set_exec_EOR, TG68_PC_dec, c_out, OP1out, micro_state) begin
    case(micro_state)
    ld_nn : begin
      // (nnnn).w/l=>
      get_ea_now <= 1'b1;
      setnextpass <= 1'b1;
      setaddrlong <= 1'b1;
    end
    st_nn : begin
      // =>(nnnn).w/l
      setstate <= 2'b11;
      setaddrlong <= 1'b1;
      next_micro_state <= nop;
    end
    ld_dAn1 : begin
      // d(An)=>, --d(PC)=>
      setstate <= 2'b01;
      next_micro_state <= ld_dAn2;
    end
    ld_dAn2 : begin
      // d(An)=>, --d(PC)=>
      get_ea_now <= 1'b1;
      setdisp <= 1'b1;
      //word
      setnextpass <= 1'b1;
    end
    ld_AnXn1 : begin
      // d(An,Xn)=>, --d(PC,Xn)=>
      setstate <= 2'b01;
      next_micro_state <= ld_AnXn2;
    end
    ld_AnXn2 : begin
      // d(An,Xn)=>, --d(PC,Xn)=>
      setdisp <= 1'b1;
      //byte	
      setdispbyte <= 1'b1;
      setstate <= 2'b01;
      setbriefext <= 1'b1;
      next_micro_state <= ld_AnXn3;
    end
    ld_AnXn3 : begin
      get_ea_now <= 1'b1;
      setdisp <= 1'b1;
      //brief
      setdispbrief <= 1'b1;
      setnextpass <= 1'b1;
    end
    st_dAn1 : begin
      // =>d(An)
      setstate <= 2'b01;
      next_micro_state <= st_dAn2;
    end
    st_dAn2 : begin
      // =>d(An)
      setstate <= 2'b11;
      setdisp <= 1'b1;
      //word
      next_micro_state <= nop;
    end
    st_AnXn1 : begin
      // =>d(An,Xn)
      setstate <= 2'b01;
      next_micro_state <= st_AnXn2;
    end
    st_AnXn2 : begin
      // =>d(An,Xn)
      setdisp <= 1'b1;
      //byte
      setdispbyte <= 1'b1;
      setstate <= 2'b01;
      setbriefext <= 1'b1;
      next_micro_state <= st_AnXn3;
    end
    st_AnXn3 : begin
      setstate <= 2'b11;
      setdisp <= 1'b1;
      //brief	
      setdispbrief <= 1'b1;
      next_micro_state <= nop;
    end
    bra1 : begin
      //bra
      if(condition == 1'b1) begin
        TG68_PC_br8 <= 1'b1;
        //pc+0000
        setstate <= 2'b01;
        next_micro_state <= bra2;
      end
    end
    bra2 : begin
      //bra
      TG68_PC_brw <= 1'b1;
    end
    bsr1 : begin
      //bsr
      set_TG68_PC_dec <= 1'b1;
      //in 2 Takten -2
      setstate <= 2'b01;
      next_micro_state <= bsr2;
    end
    bsr2 : begin
      //bsr
      if(TG68_PC_dec[0] == 1'b1) begin
        TG68_PC_brw <= 1'b1;
      end
      else begin
        TG68_PC_br8 <= 1'b1;
      end
      writePC <= 1'b1;
      setstate <= 2'b11;
      next_micro_state <= nop;
    end
    dbcc1 : begin
      //dbcc
      TG68_PC_nop <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= dbcc2;
    end
    dbcc2 : begin
      //dbcc
      TG68_PC_brw <= 1'b1;
    end
    movem : begin
      //movem
      set_movem_busy <= 1'b1;
      setstate <= 2'b10;
    end
    andi : begin
      //andi
      if(opcode[5:4] != 2'b00) begin
        ea_build <= 1'b1;
        setnextpass <= 1'b1;
      end
    end
    op_AxAy : begin
      // op -(Ax),-(Ay)
      presub <= 1'b1;
      dest_hbits <= 1'b1;
      dest_areg <= 1'b1;
      set_mem_addsub <= 1'b1;
      setstate <= 2'b10;
    end
    cmpm : begin
      // cmpm (Ay)+,(Ax)+
      postadd <= 1'b1;
      dest_hbits <= 1'b1;
      dest_areg <= 1'b1;
      set_mem_rega <= 1'b1;
      setstate <= 2'b10;
    end
    link : begin
      // link
      setstate <= 2'b11;
      save_memaddr <= 1'b1;
      regwrena <= 1'b1;
    end
    int1 : begin
      // interrupt
      presub <= 1'b1;
      setstackaddr <= 1'b1;
      set_mem_addsub <= 1'b1;
      setstate <= 2'b11;
      datatype <= 2'b10;
      next_micro_state <= int2;
    end
    int2 : begin
      // interrupt
      presub <= 1'b1;
      setstackaddr <= 1'b1;
      set_mem_addsub <= 1'b1;
      setstate <= 2'b11;
      datatype <= 2'b01;
      writeSR <= 1'b1;
      next_micro_state <= int3;
    end
    int3 : begin
      // interrupt
      set_vectoraddr <= 1'b1;
      datatype <= 2'b10;
      set_directPC <= 1'b1;
      setstate <= 2'b10;
      next_micro_state <= int4;
    end
    int4 : begin
      // interrupt
      datatype <= 2'b10;
    end
    rte : begin
      // RTE
      datatype <= 2'b10;
      setstate <= 2'b10;
      postadd <= 1'b1;
      setstackaddr <= 1'b1;
      set_mem_rega <= 1'b1;
      set_directPC <= 1'b1;
      next_micro_state <= nop;
    end
    trap1 : begin
      // TRAP
      presub <= 1'b1;
      setstackaddr <= 1'b1;
      set_mem_addsub <= 1'b1;
      setstate <= 2'b11;
      datatype <= 2'b01;
      writeSR <= 1'b1;
      next_micro_state <= trap2;
    end
    trap2 : begin
      // TRAP
      set_vectoraddr <= 1'b1;
      datatype <= 2'b10;
      set_directPC <= 1'b1;
      //					longreaddirect <= '1';
      setstate <= 2'b10;
      next_micro_state <= trap3;
    end
    trap3 : begin
      // TRAP
      datatype <= 2'b10;
    end
    movep1 : begin
      // MOVEP d(An)
      setstate <= 2'b01;
      if(opcode[6] == 1'b1) begin
        set_movepl <= 1'b1;
      end
      next_micro_state <= movep2;
    end
    movep2 : begin
      setdisp <= 1'b1;
      if(opcode[7] == 1'b0) begin
        setstate <= 2'b10;
      end
      else begin
        setstate <= 2'b11;
        wait_mem_byte <= 1'b1;
      end
      next_micro_state <= movep3;
    end
    movep3 : begin
      if(opcode[6] == 1'b1) begin
        set_movepw <= 1'b1;
        next_micro_state <= movep4;
      end
      if(opcode[7] == 1'b0) begin
        setstate <= 2'b10;
      end
      else begin
        setstate <= 2'b11;
      end
    end
    movep4 : begin
      if(opcode[7] == 1'b0) begin
        setstate <= 2'b10;
      end
      else begin
        wait_mem_byte <= 1'b1;
        setstate <= 2'b11;
      end
      next_micro_state <= movep5;
    end
    movep5 : begin
      if(opcode[7] == 1'b0) begin
        setstate <= 2'b10;
      end
      else begin
        setstate <= 2'b11;
      end
    end
    init1 : begin
      // init SP
      longreaddirect <= 1'b1;
      next_micro_state <= init2;
    end
    init2 : begin
      // init PC
      get_ea_now <= 1'b1;
      //\
      ea_only <= 1'b1;
      //-  OP1in <= memaddr_in
      setaddrlong <= 1'b1;
      //   memaddr_in <= data_read
      regwrena <= 1'b1;
      setstackaddr <= 1'b1;
      //   dest_addr <= SP
      set_directPC <= 1'b1;
      longreaddirect <= 1'b1;
      next_micro_state <= nop;
    end
    mul1 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul2;
    end
    mul2 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul3;
    end
    mul3 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul4;
    end
    mul4 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul5;
    end
    mul5 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul6;
    end
    mul6 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul7;
    end
    mul7 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul8;
    end
    mul8 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul9;
    end
    mul9 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul10;
    end
    mul10 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul11;
    end
    mul11 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul12;
    end
    mul12 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul13;
    end
    mul13 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul14;
    end
    mul14 : begin
      // mulu
      set_exec_MULU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= mul15;
    end
    mul15 : begin
      // mulu
      set_exec_MULU <= 1'b1;
    end
    div1 : begin
      // divu
      if(OP2out[15:0] == 16'h0000) begin
        //div zero
        set_Z_error <= 1'b1;
      end
      else begin
        set_exec_DIVU <= 1'b1;
        next_micro_state <= div2;
      end
      setstate <= 2'b01;
    end
    div2 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div3;
    end
    div3 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div4;
    end
    div4 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div5;
    end
    div5 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div6;
    end
    div6 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div7;
    end
    div7 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div8;
    end
    div8 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div9;
    end
    div9 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div10;
    end
    div10 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div11;
    end
    div11 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div12;
    end
    div12 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div13;
    end
    div13 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div14;
    end
    div14 : begin
      // divu
      set_exec_DIVU <= 1'b1;
      setstate <= 2'b01;
      next_micro_state <= div15;
    end
    div15 : begin
      // divu
      set_exec_DIVU <= 1'b1;
    end
    default : begin
    end
    endcase
  end

  //---------------------------------------------------------------------------
  // Conditions
  //---------------------------------------------------------------------------
  always @(opcode, Flags) begin
    case(opcode[11:8])
    4'h0 : begin
      condition <= 1'b1;
    end
    4'h1 : begin
      condition <= 1'b0;
    end
    4'h2 : begin
      condition <=  ~Flags[0] &  ~Flags[2];
    end
    4'h3 : begin
      condition <= Flags[0] | Flags[2];
    end
    4'h4 : begin
      condition <=  ~Flags[0];
    end
    4'h5 : begin
      condition <= Flags[0];
    end
    4'h6 : begin
      condition <=  ~Flags[2];
    end
    4'h7 : begin
      condition <= Flags[2];
    end
    4'h8 : begin
      condition <=  ~Flags[1];
    end
    4'h9 : begin
      condition <= Flags[1];
    end
    4'ha : begin
      condition <=  ~Flags[3];
    end
    4'hb : begin
      condition <= Flags[3];
    end
    4'hc : begin
      condition <= (Flags[3] & Flags[1]) | ( ~Flags[3] &  ~Flags[1]);
    end
    4'hd : begin
      condition <= (Flags[3] &  ~Flags[1]) | ( ~Flags[3] & Flags[1]);
    end
    4'he : begin
      condition <= (Flags[3] & Flags[1] &  ~Flags[2]) | ( ~Flags[3] &  ~Flags[1] &  ~Flags[2]);
    end
    4'hf : begin
      condition <= (Flags[3] &  ~Flags[1]) | ( ~Flags[3] & Flags[1]) | Flags[2];
    end
    default : begin
    end
    endcase
  end

  //---------------------------------------------------------------------------
  // Bits
  //---------------------------------------------------------------------------
  always @(opcode, OP1out, OP2out, one_bit_in, one_bit_out, bit_Number, bit_number_reg) begin
    case(opcode[7:6])
    2'b00 : begin
      //btst
      one_bit_out <= one_bit_in;
    end
    2'b01 : begin
      //bchg
      one_bit_out <=  ~one_bit_in;
    end
    2'b10 : begin
      //bclr
      one_bit_out <= 1'b0;
    end
    2'b11 : begin
      //bset
      one_bit_out <= 1'b1;
    end
    default : begin
    end
    endcase
    if(opcode[8] == 1'b0) begin
      if(opcode[5:4] == 2'b00) begin
        bit_number <= bit_number_reg[4:0];
      end
      else begin
        bit_number <= {2'b00,bit_number_reg[2:0]};
      end
    end
    else begin
      if(opcode[5:4] == 2'b00) begin
        bit_number <= OP2out[4:0];
      end
      else begin
        bit_number <= {2'b00,OP2out[2:0]};
      end
    end
    bits_out <= OP1out;
    case(bit_Number)
    5'b00000 : begin
      one_bit_in <= OP1out[0];
      bits_out[0] <= one_bit_out;
    end
    5'b00001 : begin
      one_bit_in <= OP1out[1];
      bits_out[1] <= one_bit_out;
    end
    5'b00010 : begin
      one_bit_in <= OP1out[2];
      bits_out[2] <= one_bit_out;
    end
    5'b00011 : begin
      one_bit_in <= OP1out[3];
      bits_out[3] <= one_bit_out;
    end
    5'b00100 : begin
      one_bit_in <= OP1out[4];
      bits_out[4] <= one_bit_out;
    end
    5'b00101 : begin
      one_bit_in <= OP1out[5];
      bits_out[5] <= one_bit_out;
    end
    5'b00110 : begin
      one_bit_in <= OP1out[6];
      bits_out[6] <= one_bit_out;
    end
    5'b00111 : begin
      one_bit_in <= OP1out[7];
      bits_out[7] <= one_bit_out;
    end
    5'b01000 : begin
      one_bit_in <= OP1out[8];
      bits_out[8] <= one_bit_out;
    end
    5'b01001 : begin
      one_bit_in <= OP1out[9];
      bits_out[9] <= one_bit_out;
    end
    5'b01010 : begin
      one_bit_in <= OP1out[10];
      bits_out[10] <= one_bit_out;
    end
    5'b01011 : begin
      one_bit_in <= OP1out[11];
      bits_out[11] <= one_bit_out;
    end
    5'b01100 : begin
      one_bit_in <= OP1out[12];
      bits_out[12] <= one_bit_out;
    end
    5'b01101 : begin
      one_bit_in <= OP1out[13];
      bits_out[13] <= one_bit_out;
    end
    5'b01110 : begin
      one_bit_in <= OP1out[14];
      bits_out[14] <= one_bit_out;
    end
    5'b01111 : begin
      one_bit_in <= OP1out[15];
      bits_out[15] <= one_bit_out;
    end
    5'b10000 : begin
      one_bit_in <= OP1out[16];
      bits_out[16] <= one_bit_out;
    end
    5'b10001 : begin
      one_bit_in <= OP1out[17];
      bits_out[17] <= one_bit_out;
    end
    5'b10010 : begin
      one_bit_in <= OP1out[18];
      bits_out[18] <= one_bit_out;
    end
    5'b10011 : begin
      one_bit_in <= OP1out[19];
      bits_out[19] <= one_bit_out;
    end
    5'b10100 : begin
      one_bit_in <= OP1out[20];
      bits_out[20] <= one_bit_out;
    end
    5'b10101 : begin
      one_bit_in <= OP1out[21];
      bits_out[21] <= one_bit_out;
    end
    5'b10110 : begin
      one_bit_in <= OP1out[22];
      bits_out[22] <= one_bit_out;
    end
    5'b10111 : begin
      one_bit_in <= OP1out[23];
      bits_out[23] <= one_bit_out;
    end
    5'b11000 : begin
      one_bit_in <= OP1out[24];
      bits_out[24] <= one_bit_out;
    end
    5'b11001 : begin
      one_bit_in <= OP1out[25];
      bits_out[25] <= one_bit_out;
    end
    5'b11010 : begin
      one_bit_in <= OP1out[26];
      bits_out[26] <= one_bit_out;
    end
    5'b11011 : begin
      one_bit_in <= OP1out[27];
      bits_out[27] <= one_bit_out;
    end
    5'b11100 : begin
      one_bit_in <= OP1out[28];
      bits_out[28] <= one_bit_out;
    end
    5'b11101 : begin
      one_bit_in <= OP1out[29];
      bits_out[29] <= one_bit_out;
    end
    5'b11110 : begin
      one_bit_in <= OP1out[30];
      bits_out[30] <= one_bit_out;
    end
    5'b11111 : begin
      one_bit_in <= OP1out[31];
      bits_out[31] <= one_bit_out;
    end
    default : begin
    end
    endcase
  end

  //---------------------------------------------------------------------------
  // Rotation
  //---------------------------------------------------------------------------
  always @(opcode, OP1out, Flags, rot_bits, rot_msb, rot_lsb, rot_rot, rot_nop) begin
    case(opcode[7:6])
    2'b00 : begin
      //Byte
      rot_rot <= OP1out[7];
    end
    2'b01,2'b11 : begin
      //Word
      rot_rot <= OP1out[15];
    end
    2'b10 : begin
      //Long
      rot_rot <= OP1out[31];
    end
    default : begin
    end
    endcase
    case(rot_bits)
    2'b00 : begin
      //ASL, ASR
      rot_lsb <= 1'b0;
      rot_msb <= rot_rot;
    end
    2'b01 : begin
      //LSL, LSR
      rot_lsb <= 1'b0;
      rot_msb <= 1'b0;
    end
    2'b10 : begin
      //ROXL, ROXR
      rot_lsb <= Flags[4];
      rot_msb <= Flags[4];
    end
    2'b11 : begin
      //ROL, ROR
      rot_lsb <= rot_rot;
      rot_msb <= OP1out[0];
    end
    default : begin
    end
    endcase
    if(rot_nop == 1'b1) begin
      rot_out <= OP1out;
      rot_XC <= Flags[0];
    end
    else begin
      if(opcode[8] == 1'b1) begin
        //left
        rot_out <= {OP1out[30:0],rot_lsb};
        rot_XC <= rot_rot;
      end
      else begin
        //right
        rot_XC <= OP1out[0];
        rot_out <= {rot_msb,OP1out[31:1]};
        case(opcode[7:6])
        2'b00 : begin
          //Byte
          rot_out[7] <= rot_msb;
        end
        2'b01,2'b11 : begin
          //Word
          rot_out[15] <= rot_msb;
        end
        default : begin
        end
        endcase
      end
    end
  end

  //---------------------------------------------------------------------------
  // MULU/MULS
  //---------------------------------------------------------------------------
  always @(posedge clk, posedge opcode, posedge OP2out, posedge muls_msb, posedge mulu_reg, posedge OP1sign, posedge sign2) begin
    if(clkena == 1'b1) begin
      if(decodeOPC == 1'b1) begin
        if(opcode[8] == 1'b1 && reg_QB[15] == 1'b1) begin
          //MULS Neg faktor
          OP1sign <= 1'b1;
          mulu_reg <= {16'b0000000000000000,0 - reg_QB[15:0]};
        end
        else begin
          OP1sign <= 1'b0;
          mulu_reg <= {16'b0000000000000000,reg_QB[15:0]};
        end
      end
      else if(exec_MULU == 1'b1) begin
        mulu_reg <= dummy_mulu;
      end
    end
  end

  always @(clk, opcode, OP2out, muls_msb, mulu_reg, OP1sign, sign2) begin
    if((opcode[8] == 1'b1 && OP2out[15] == 1'b1) || OP1sign == 1'b1) begin
      muls_msb <= mulu_reg[31];
    end
    else begin
      muls_msb <= 1'b0;
    end
    if(opcode[8] == 1'b1 && OP2out[15] == 1'b1) begin
      sign2 <= 1'b1;
    end
    else begin
      sign2 <= 1'b0;
    end
    if(mulu_reg[0] == 1'b1) begin
      if(OP1sign == 1'b1) begin
        dummy_mulu <= {({muls_msb,mulu_reg[31:16]}) - ({sign2,OP2out[15:0]}),mulu_reg[15:1]};
      end
      else begin
        dummy_mulu <= {({muls_msb,mulu_reg[31:16]}) + ({sign2,OP2out[15:0]}),mulu_reg[15:1]};
      end
    end
    else begin
      dummy_mulu <= {muls_msb,mulu_reg[31:1]};
    end
  end

  //---------------------------------------------------------------------------
  // DIVU
  //---------------------------------------------------------------------------
  always @(posedge clk, posedge execOPC, posedge opcode, posedge OP1out, posedge OP2out, posedge div_reg, posedge dummy_div_sub, posedge div_quot, posedge div_sign, posedge dummy_div_over, posedge dummy_div) begin
    if(clkena == 1'b1) begin
      if(decodeOPC == 1'b1) begin
        if(opcode[8] == 1'b1 && reg_QB[31] == 1'b1) begin
          // Neg divisor
          div_sign <= 1'b1;
          div_reg <= 0 - reg_QB;
        end
        else begin
          div_sign <= 1'b0;
          div_reg <= reg_QB;
        end
      end
      else if(exec_DIVU == 1'b1) begin
        div_reg <= div_quot;
      end
    end
  end

  always @(clk, execOPC, opcode, OP1out, OP2out, div_reg, dummy_div_sub, div_quot, div_sign, dummy_div_over, dummy_div) begin
    set_V_Flag <= 1'b0;
    dummy_div_over <= ({1'b0,OP1out[31:16]}) - ({1'b0,OP2out[15:0]});
    if(opcode[8] == 1'b1 && OP2out[15] == 1'b1) begin
      dummy_div_sub <= (div_reg[31:15]) + ({1'b1,OP2out[15:0]});
    end
    else begin
      dummy_div_sub <= (div_reg[31:15]) - ({1'b0,OP2out[15:0]});
    end
    if((dummy_div_sub[16]) == 1'b1) begin
      div_quot[31:16] <= div_reg[30:15];
    end
    else begin
      div_quot[31:16] <= dummy_div_sub[15:0];
    end
    div_quot[15:0] <= {div_reg[14:0], ~dummy_div_sub[16]};
    if(execOPC == 1'b1 && opcode[8] == 1'b1 && (OP2out[15] ^ div_sign) == 1'b1) begin
      dummy_div[15:0] <= 0 - div_quot[15:0];
    end
    else begin
      dummy_div[15:0] <= div_quot[15:0];
    end
    if(div_sign == 1'b1) begin
      dummy_div[31:16] <= 0 - div_quot[31:16];
    end
    else begin
      dummy_div[31:16] <= div_quot[31:16];
    end
    if((opcode[8] == 1'b1 && (OP2out[15] ^ div_sign ^ dummy_div[15]) == 1'b1 && dummy_div[15:0] != 16'h0000) || (opcode[8] == 1'b0 && dummy_div_over[16] == 1'b0)) begin
      //Overflow DIVU
      set_V_Flag <= 1'b1;
    end
  end

  //---------------------------------------------------------------------------
  // Movem
  //---------------------------------------------------------------------------
  always @(reset, clk, movem_mask, movem_muxa, movem_muxb, movem_muxc) begin
    if(movem_mask[7:0] == 8'b00000000) begin
      movem_muxa <= movem_mask[15:8];
      movem_regaddr[3] <= 1'b1;
    end
    else begin
      movem_muxa <= movem_mask[7:0];
      movem_regaddr[3] <= 1'b0;
    end
    if(movem_muxa[3:0] == 4'b0000) begin
      movem_muxb <= movem_muxa[7:4];
      movem_regaddr[2] <= 1'b1;
    end
    else begin
      movem_muxb <= movem_muxa[3:0];
      movem_regaddr[2] <= 1'b0;
    end
    if(movem_muxb[1:0] == 2'b00) begin
      movem_muxc <= movem_muxb[3:2];
      movem_regaddr[1] <= 1'b1;
    end
    else begin
      movem_muxc <= movem_muxb[1:0];
      movem_regaddr[1] <= 1'b0;
    end
    if(movem_muxc[0] == 1'b0) begin
      movem_regaddr[0] <= 1'b1;
    end
    else begin
      movem_regaddr[0] <= 1'b0;
    end
    movem_bits <= ({4'b0000,movem_mask[0]}) + ({4'b0000,movem_mask[1]}) + ({4'b0000,movem_mask[2]}) + ({4'b0000,movem_mask[3]}) + ({4'b0000,movem_mask[4]}) + ({4'b0000,movem_mask[5]}) + ({4'b0000,movem_mask[6]}) + ({4'b0000,movem_mask[7]}) + ({4'b0000,movem_mask[8]}) + ({4'b0000,movem_mask[9]}) + ({4'b0000,movem_mask[10]}) + ({4'b0000,movem_mask[11]}) + ({4'b0000,movem_mask[12]}) + ({4'b0000,movem_mask[13]}) + ({4'b0000,movem_mask[14]}) + ({4'b0000,movem_mask[15]});
  end

  always @(posedge reset, posedge clk, posedge movem_mask, posedge movem_muxa, posedge movem_muxb, posedge movem_muxc) begin
    if(reset == 1'b0) begin
      movem_busy <= 1'b0;
      movem_addr <= 1'b0;
      maskzero <= 1'b0;
    end else begin
      if(clkena_in == 1'b1 && get_movem_mask == 1'b1) begin
        movem_mask <= data_read[15:0];
      end
      if(clkena_in == 1'b1 && test_maskzero == 1'b1) begin
        if(movem_mask == 16'h0000) begin
          maskzero <= 1'b1;
        end
      end
      if(clkena_in == 1'b1 && endOPC == 1'b1) begin
        maskzero <= 1'b0;
      end
      if(clkena == 1'b1) begin
        if(set_movem_busy == 1'b1) begin
          if(movem_bits[4:1] != 4'b0000 || opcode[10] == 1'b0) begin
            movem_busy <= 1'b1;
          end
          movem_addr <= 1'b1;
        end
        if(movem_addr == 1'b1) begin
          case(movem_regaddr)
          4'b0000 : begin
            movem_mask[0] <= 1'b0;
          end
          4'b0001 : begin
            movem_mask[1] <= 1'b0;
          end
          4'b0010 : begin
            movem_mask[2] <= 1'b0;
          end
          4'b0011 : begin
            movem_mask[3] <= 1'b0;
          end
          4'b0100 : begin
            movem_mask[4] <= 1'b0;
          end
          4'b0101 : begin
            movem_mask[5] <= 1'b0;
          end
          4'b0110 : begin
            movem_mask[6] <= 1'b0;
          end
          4'b0111 : begin
            movem_mask[7] <= 1'b0;
          end
          4'b1000 : begin
            movem_mask[8] <= 1'b0;
          end
          4'b1001 : begin
            movem_mask[9] <= 1'b0;
          end
          4'b1010 : begin
            movem_mask[10] <= 1'b0;
          end
          4'b1011 : begin
            movem_mask[11] <= 1'b0;
          end
          4'b1100 : begin
            movem_mask[12] <= 1'b0;
          end
          4'b1101 : begin
            movem_mask[13] <= 1'b0;
          end
          4'b1110 : begin
            movem_mask[14] <= 1'b0;
          end
          4'b1111 : begin
            movem_mask[15] <= 1'b0;
          end
          default : begin
          end
          endcase
          if(opcode[10] == 1'b1) begin
            if(movem_bits == 5'b00010 || movem_bits == 5'b00001 || movem_bits == 5'b00000) begin
              movem_busy <= 1'b0;
            end
          end
          if(movem_bits == 5'b00001 || movem_bits == 5'b00000) begin
            movem_busy <= 1'b0;
            movem_addr <= 1'b0;
          end
        end
      end
    end
  end


endmodule
