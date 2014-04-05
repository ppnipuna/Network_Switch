// we will be using 3 interfaces 
/* 1. For memory
   2. For input side
   3. For output side
*/

// #####################  Memory interface ########################
interface mem_if(input bit slow_clk);
// define signals
logic mem_en, mem_rd_wr, mem_rstn;
logic [1:0] mem_addr;
logic [7:0] mem_wdata, mem_rdata;

//define clocking block
clocking cb @(posedge slow_clk);
	output mem_en, mem_rd_wr, mem_wdata, mem_addr;
	input mem_rdata;
endclocking : cb

// define modport with respect to test bench
modport mem_tb(
clocking cb, output mem_rstn);

// define modport with repect to DUT
modport mem_dut(
input mem_rstn, mem_rd_wr, mem_en, mem_wdata, mem_addr, slow_clk,
output mem_rdata); 

endinterface : mem_if	

// #####################  Input interface ########################	
interface ip_if(input bit fast_clk);
// define signals
logic data_valid, data_stall,reset_b;
logic [7:0] data;

// clocking  block
clocking cb @(posedge fast_clk);
	output data_valid, data;
	input data_stall;
endclocking : cb

// define modport with respect to test bench
modport ip_tb(
clocking cb, output reset_b);

// define modport with repect to DUT
modport ip_dut(
input data_valid, reset_b, data, fast_clk,
output data_stall); 

endinterface : ip_if 

// #####################  Output interface ########################	
interface op_if(input bit slow_clk);
// define signals
logic read, ready;
logic [7:0] port;

// clocking  block
clocking cb @(posedge slow_clk);
	output read;
	input ready, port;
endclocking : cb

// define modport with respect to test bench
modport op_tb(
clocking cb);

// define modport with repect to DUT
modport ip_mp_dut(
input read, slow_clk, 
output ready, port); 

endinterface : op_if