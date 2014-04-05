`timescale 1ns / 1ns	   

//Predictor module: this module will decide whether to send a packet to DUT or not based on the address
module Predictor
	(
	input wire clk,						//clock from the fast domain
	input wire data_valid_this_clk,				//used to connect to DUT's data_valid_this_clk
	input wire  [07:00] data,			
	input wire  [07:00] addr_0,	// Address of the port 0
    input wire  [07:00] addr_1,	// Address of the port 1
    input wire  [07:00] addr_2,	// Address of the port 2
    input wire  [07:00] addr_3, // Address of the port 3
	output reg drop_pkt,   // will decide whether to drop_pkt or send the packet
	output reg [07:00] final_addr		//indicates which addr(sb) to route to
	);
	
	// intermediate signals
	reg [07:00] dest_addr; // will be used for comparing addresses with those which are valid
	reg data_valid_last_clk;
	
	initial data_valid_last_clk = 1'b0;
		
	always @(posedge clk)
		begin
			if(data_valid_this_clk == 1'b1 && data_valid_last_clk == 1'b0) begin
				data_valid_last_clk <= 1'b1;
				dest_addr <= data;
			end
			else if(data_valid_this_clk == 1'b0 && data_valid_last_clk == 1'b1) 
				data_valid_last_clk <= 1'b0;
		end
	
	always @(dest_addr) begin
		case(dest_addr)	  // to decide whether to route the packet or no
			addr_0: begin
				final_addr = addr_0;
				drop_pkt = 1'b0;
			end
			addr_1: begin
				final_addr = addr_1;
				drop_pkt = 1'b0;
			end
			addr_2: begin
				final_addr = addr_2;
				drop_pkt = 1'b0;
			end
			addr_3: begin
				final_addr = addr_3;
				drop_pkt = 1'b0;
			end
			default: begin
				drop_pkt = 1'b1;  // this packet will be dropped
				final_addr = -1;
			end																			
		endcase
	end
endmodule: Predictor   


//This module will be the entry point for the verification environment
module top;	
	
	//declare parameters
	parameter 			PER_FAST_CLK = 10,
						PER_SLOW_CLK = 333;
			
	//declare global signals
	logic fast_clk;
	logic slow_clk;
	
	
	// instantiate interfaces
	mem_if mem_if_top (slow_clk);
	ip_if ip_if_top (fast_clk);
	op_if op_if_top[4] (slow_clk);
	
	switch DUT(
	// Global inputs                   		
	.fast_clk		(fast_clk),    	  		
	.slow_clk		(slow_clk),      		
	.reset_b		(ip_if_top.reset_b), 
	
	// Memory inputs                  		
	.mem_en			(mem_if_top.mem_en),        // Memory enable
	.mem_rd_wr		(mem_if_top.mem_rd_wr),     // Memory read/write
	.mem_addr		(mem_if_top.mem_addr),      // Memory address
	.mem_wdata		(mem_if_top.mem_wdata),     // Data to be written to the memory interface
	.mem_rdata		(mem_if_top.mem_rdata),     // Data read from the memory interface
	
	// Input port               	        
	.data_stall		(ip_if_top.data_stall),     //
	.data_valid		(ip_if_top.data_valid),     // Data valid input
	.data			(ip_if_top.data),           // Data input
	
	// Output ports                    		
	.ready_0		(op_if_top[0].ready),       
	.read_0			(op_if_top[0].read),        
	.port0			(op_if_top[0].port),        
	.ready_1		(op_if_top[1].ready),       
	.read_1			(op_if_top[1].read),        
	.port1			(op_if_top[1].port),        
	.ready_2		(op_if_top[2].ready),       
	.read_2			(op_if_top[2].read),        
	.port2			(op_if_top[2].port),        
	.ready_3		(op_if_top[3].ready),       
	.read_3			(op_if_top[3].read),        
	.port3			(op_if_top[3].port)         
	);
	
	//instantiate DUT predictor
	Predictor pred_top(
	.clk			(fast_clk),											
	.data_valid_this_clk		(ip_if_top.data_valid),								
	.data			(ip_if_top.data),									
	.addr_0	($root.DUT.i_port_address_reg.address_port_0),	// Address for Port 0
	.addr_1	($root.DUT.i_port_address_reg.address_port_1),	// Address for Port 1
	.addr_2	($root.DUT.i_port_address_reg.address_port_2),	// Address for Port 2
	.addr_3	($root.DUT.i_port_address_reg.address_port_3),	// Address for Port 3
    .drop_pkt			(), 
	.final_addr		()
	);
	
	// test program instantiation	
	test test_inst();			
	
	// clock generation
	initial begin
			fork
				//***********Generate the Fast Clock**************//
				begin
					fast_clk = 1'b0;
					forever #(PER_FAST_CLK/2) fast_clk = ~fast_clk;
				end 
				//***********End Generate the Fast Clock**************//
				
				//***********Generate the Slow Clock**************//
				begin									  
					slow_clk = 1'b0;
					forever #(PER_SLOW_CLK/2) slow_clk = ~slow_clk;
				end												
				//***********End Generate the Slow Clock**************//
				
			join
			
		end	
	
endmodule: top