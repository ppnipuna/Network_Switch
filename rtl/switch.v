`timescale 1ns / 1ns

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : D flip-flop async reset
// Description :
//-----------------------------------------------------------------------------
module dff_async_reset
    (input  wire d,       // Data Input
    input  wire clk,     // Clock Input
    input  wire reset_b, // Reset input
    output reg  q);      // Q output
    
    always@(posedge clk or negedge reset_b)
        if (~reset_b) q <= 1'b0;
        else          q <= d;
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : Synchronization
//-----------------------------------------------------------------------------
module sync
    (input wire OutputClock,  //
    input  wire reset_b,      //
    input  wire InputData,    //
    output wire OutputData);  //

    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    wire synch_flop_1;
    
    
    /*************************************************************************
    * Module Instantations                                                  *
    *************************************************************************/
    
    dff_async_reset
        dff_1
        (// Global Signals                  // ----------------------------
        .clk     (OutputClock),
        .reset_b (reset_b),
        //                                  // ----------------------------
        .d       (InputData),
        .q       (synch_flop_1));
    
    dff_async_reset
        dff_2
        (// Global Signals                  // ----------------------------
        .clk     (OutputClock),
        .reset_b (reset_b),
        //                                 // ----------------------------
        .d       (synch_flop_1),
        .q       (OutputData));
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : Pointer Comparator Module
//-----------------------------------------------------------------------------
module COMPARE
    #(parameter DEPTH = 10, ALMOST_FULL = 4 /* 32 Elements */)
    (input  [DEPTH:0] wptr, rptr,
    output wire      empty, full);
    
    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    assign empty = (wptr[DEPTH:0] == rptr[DEPTH:0]);
    // assign full  = (wptr[DEPTH] != rptr[DEPTH]) & (wptr[DEPTH-1:0] == rptr[DEPTH-1:0]);
    assign full  = (wptr[DEPTH] != rptr[DEPTH]) & (wptr[DEPTH-1: ALMOST_FULL] == rptr[DEPTH-1: ALMOST_FULL]);
    
    // synthesis translate off
    always@(full or empty) begin
        $display("current depth is %0d, current empty is %0d, current full is %0d", DEPTH, empty, full);
    end 
    // synthesis translate on
endmodule        


//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : D flip-flop async reset
//-----------------------------------------------------------------------------
module POINTER
    #(parameter DEPTH = 5 /* 32 Elements */)
    (input                clk, reset_b, incr,
    output reg [DEPTH:0] ptr);
    
    // fifo control logic (register for read pointers)
    always@(posedge clk, negedge reset_b)
        if (~reset_b) ptr <= 0;
        else if (incr) ptr <= ptr + 1;
    
endmodule

module memory
    #(parameter WIDTH = 7, DEPTH = 32, ADDR = 5)
    (// Globals Signals
    input  wire             clk, reset_b, 
    input  wire             write,
    input  wire [ADDR -1:0] waddr, raddr,
    input  wire [WIDTH-1:0] wdata,
    output wire [WIDTH-1:0] rdata);
    
    integer i;
    reg [WIDTH-1:0] memory [DEPTH-1:0];
    
    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    assign rdata = memory [raddr]; // register file read operation
    
    always @(posedge clk, negedge reset_b) // register file write operation
        if (~reset_b) for (i = 0; i < DEPTH; i = i + 1) memory[i] <= 'd0;
        else if (write) memory [waddr] <= wdata;
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : Asynchronous FIFO
//-----------------------------------------------------------------------------
module async_fifo (reset_b, rclk, wclk, write, read, wdata, rdata, empty, full);
    parameter WIDTH =  8;
    parameter DEPTH = 32;
    parameter ADDR  =  5;
    
    input  wire 			reset_b;
    input  wire 			rclk;
    input  wire 			wclk;
    input  wire 			write;
    input  wire             read;
    input  wire [WIDTH-1:0] wdata;
    output wire [WIDTH-1:0] rdata;
    output wire             empty;
    output wire             full;
    
    wire             write_wclk;
    wire             read_rclk;
    wire [WIDTH-1:0] wdata_wclk;
    wire [WIDTH-1:0] rdata_wclk;
    wire [ADDR   :0] raddr_wclk;
    wire [ADDR   :0] waddr_wclk;
    wire [ADDR   :0] raddr_rclk;
    wire [ADDR   :0] waddr_rclk;
    
    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    assign wdata_wclk = wdata;
    assign write_wclk = write;
    assign read_rclk  = read;
    assign rdata      = rdata_wclk;
    
    /*************************************************************************
    * Module Instantiation                                                  *
    *************************************************************************/
    memory #(WIDTH, DEPTH, ADDR)
        i_memory
        (// Globals Signals
        .clk     (wclk),
        .reset_b (reset_b),
        .write   (write_wclk),
        .waddr   (waddr_wclk[ADDR-1:0]),
        .raddr   (raddr_wclk[ADDR-1:0]),
        .wdata   (wdata_wclk),
        .rdata   (rdata_wclk));
        
    genvar i;
    generate
        for(i=0; i <= ADDR; i = i + 1) begin: Write_Address
                sync
                i_write_ClockSync
                (// Global Signals
                .OutputClock(rclk),          // I
                .reset_b    (reset_b),       // I
                //
                .InputData  (waddr_wclk[i]), // I
                //
                .OutputData (waddr_rclk[i]));// O
            end
    endgenerate
    
    genvar j;
    generate
        for(j = 0; j <= ADDR; j = j + 1) begin: Read_Address
                sync
                i_read_ClockSync
                (// Glbal Signals
                .OutputClock(wclk),          // I
                .reset_b    (reset_b),        // I
                //
                .InputData  (raddr_rclk[j]), // I
                //
                .OutputData (raddr_wclk[j]));// O           
        end
    endgenerate
    
    POINTER #(ADDR)
        read_pointer_rclk
        (// Globals Signals
        .clk             (rclk),
        .reset_b         (reset_b),
        //
        .incr    (read_rclk),
        //
        .ptr     (raddr_rclk));
    
    POINTER #(ADDR)
        write_pointer_wclk
        (.clk     (wclk),
        .reset_b (reset_b),
        .incr    (write_wclk),
        .ptr     (waddr_wclk));
    
    COMPARE #(ADDR)
        comparator_rclk (
        .wptr    (waddr_rclk),
        .rptr    (raddr_rclk),
        .empty   (empty),
        .full    ());
    
    COMPARE #(ADDR)
        comparator_wclk
        (.wptr    (waddr_wclk),
        .rptr    (raddr_wclk),
        .empty   (),
        .full    (full));
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function : Output State Machine
//-----------------------------------------------------------------------------
module output_sm (
    // Global inputs
    input  wire clk,                   // System Clock
    input  wire reset_b,               // Active high, asyn reset
    // FIFO Interface
    input  wire        fifo_empty,     // FIFO is empty
    output wire        fifo_read,      // FIFO read enable
    input  wire [8:0]  fifo_data,      // FIFO data
    // Ouput Interface
    output wire         ready,         // Output data valid
    input  wire         read,          // Output read enable
    output reg  [7:0]   port           // Data input
    );
    
    parameter IDLE        = 3'b000,
        PROCESS_PACKET    = 3'b001,
        PROCESS_LAST      = 3'b010,
        END_OF_PACKET_1   = 3'b011,
        END_OF_PACKET_2   = 3'b100,
        END_OF_PACKET_3   = 3'b101;
    
    reg [02:00] state, next_state;
    reg [08:00] last_fifo_data;
    reg [07:00] next_port;
    reg         hold_off_ready;
    
    wire end_of_packet;
    
    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    
    // Pass FIFO empty signal directly out as Output data valid,
    // but also hold it until packet is processed
    assign ready = (~fifo_empty && ~hold_off_ready) || (state == PROCESS_LAST);
    
    // Pass read signal directly through as FIFO read, as long as we are in
    // states that need new data
    assign fifo_read = (~fifo_empty & read & ((next_state == PROCESS_PACKET) | (next_state == PROCESS_LAST)));
           
    assign end_of_packet = fifo_data[8];
           
    // Synchronous SM Logic
    always @(posedge clk or negedge reset_b)
        if (~reset_b) begin
            state           <= IDLE;
            port            <= 8'h0;
        end
        else begin
            state           <= next_state;
            port            <= next_port;
            last_fifo_data  <= fifo_data;
        end
    
    // Aynchronous SM Logic
    always @* 
        case(state)
            IDLE: begin
                hold_off_ready = 1'b0; // Enforce inter-packet gap
                if (read && ~fifo_empty) begin
                    next_state = PROCESS_PACKET;
                    next_port  = fifo_data[7:0];
                end
                else begin
                    next_state = IDLE;
                    next_port  = 8'h00;
                end
            end                
            PROCESS_PACKET: begin
                hold_off_ready = 1'b0; // Enforce inter-packet gap
                //  We decide that we are done processing the current packet,
                // either when the FIFO is empty or we see a different value
                // in the FIFO Index from our current value.
                if (end_of_packet | fifo_empty) begin
                    
                    next_state     = PROCESS_LAST;
                    next_port      = fifo_data[7:0];
                end
                else begin
                    next_state     = PROCESS_PACKET;
                    next_port      = fifo_data[7:0];
                end
            end                
            PROCESS_LAST: begin
                hold_off_ready = 1'b1; // Enforce inter-packet gap
                next_state     = END_OF_PACKET_1;
                next_port      = 8'h00;
            end
            END_OF_PACKET_1: begin 
                hold_off_ready = 1'b1; // Enforce inter-packet gap
                next_state     = END_OF_PACKET_2;
                next_port      = 8'h00;
            end
            END_OF_PACKET_2: begin
                hold_off_ready = 1'b1; // Enforce inter-packet gap
                next_state     = END_OF_PACKET_3;
                next_port      = 8'h00;
            end
            END_OF_PACKET_3: begin
                hold_off_ready = 1'b1; // Enforce inter-packet gap
                next_state = IDLE;
                next_port  = 8'h00;
            end
            // Illegal state
            default: begin
                hold_off_ready = 1'b0; // Enforce inter-packet gap
                next_state = IDLE;
                next_port  = 8'h00;
            end
        endcase
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function : Packet Router
//-----------------------------------------------------------------------------
module packet_router (
    // Global inputs                    // ------------------------------------
    input  wire         clk,            // System Clock
    input  wire         reset_b,        // Active Low, asyn reset
    // Port address registers           // ------------------------------------
    input wire  [07:00] address_port_0, // Address for each port
    input wire  [07:00] address_port_1, //
    input wire  [07:00] address_port_2, //
    input wire  [07:00] address_port_3, //
    // Input port                       // ------------------------------------
    input  wire         data_stall,     // Stall the input stream
    input  wire         data_valid,     // Data valid input
    input  wire [07:00] data,           // Data input
    // Output ports                     // ------------------------------------
    output reg          ready_0,        // Port 0 Output has data
    output reg  [07:00] port0,          // Port 0 Data Output
    output reg          ready_1,        // Output has data
    output reg  [07:00] port1,          // Data input
    output reg          ready_2,        // Output has data
    output reg  [07:00] port2,          // Data input
    output reg          ready_3,        // Output has data
    output reg  [07:00] port3);         // Data input
    
    parameter 
        IDLE              = 1'b0,
        PROCESS_PACKET    = 1'b1;
    
    reg       state;
    reg       next_state;
    reg [7:0] data_d;
    reg       data_ready;
    reg [7:0] packet_address;
    
    /*************************************************************************
    * Continuous Assignments                                                *
    *************************************************************************/
    
    // Synchronous SM Logic
    always @(posedge clk or negedge reset_b) begin
            if (~reset_b) begin
                    state               <= IDLE;
                    data_d              <= 9'h00;
                    data_ready          <= 1'b0;
                    packet_address      <= 8'h00;
                end
            else begin
                    state               <= next_state;
                    data_d              <= data;
                    data_ready          <= data_valid;
                    if (state == IDLE) 
                        packet_address  <= data;
                end
        end
    
    // Aynchronous SM Logic
    always @*
        case(state)
            IDLE: 
                if (data_valid & ~data_stall) next_state = PROCESS_PACKET;
                else                          next_state = IDLE;
            PROCESS_PACKET:
                if (data_valid & ~data_stall) next_state = PROCESS_PACKET;
                else if (~data_valid)         next_state = IDLE;
                else                          next_state = PROCESS_PACKET;
        endcase
    
    // Route data to correct output port or drop
    always @* begin
        port0   = 8'd0;
        ready_0 = 1'b0;
        port1   = 8'd0;
        ready_1 = 1'b0;
        port2   = 8'd0;
        ready_2 = 1'b0;
        port3   = 8'd0;
        ready_3 = 1'b0;
        case(packet_address)
            address_port_0: begin
                port0   = data_d;
                ready_0 = data_ready;
            end
            address_port_1: begin
                port1   = data_d;
                ready_1 = data_ready;
            end
            address_port_2: begin
                port2   = data_d;
                ready_2 = data_ready;
            end
            address_port_3: begin
                port3   = data_d;
                ready_3 = data_ready;
            end
        endcase
    end
    
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function    : Registers that hold port addresses
// Coder       : Kyle D. Gilsdorf
//-----------------------------------------------------------------------------
module port_address_reg
    (// Global inputs                  // -------------------------------------
    input  wire clk,                   // System Clock
    input  wire reset_b,               // Active high, asyn reset
    // Memory inputs                   // -------------------------------------
    input  wire         mem_en,        // Memory enable
    input  wire         mem_rd_wr,     // Memory read/write
    input  wire [01:00] mem_addr,      // Memory address
    input  wire [07:00] mem_wdata,     // Memory data
    output reg  [07:00] mem_rdata,     // Memory data
    // Register outputs                // -------------------------------------
    output reg  [07:00] address_port_0,// Port 0 address
    output reg  [07:00] address_port_1,// Port 1 address
    output reg  [07:00] address_port_2,// Port 2 address
    output reg  [07:00] address_port_3 // Port 3 address
    );
    
    always@*
        case(mem_addr)
            2'd0: mem_rdata <= address_port_0;
            2'd1: mem_rdata <= address_port_1;
            2'd2: mem_rdata <= address_port_2;
            2'd3: mem_rdata <= address_port_3;
        endcase
    
    always @(posedge clk or negedge reset_b) begin
            if (~reset_b) begin
                    address_port_0 <= 8'h00;
                    address_port_1 <= 8'h01;
                    address_port_2 <= 8'h02;
                    address_port_3 <= 8'h03;
                end
            else if (mem_en && mem_rd_wr) begin
                    case (mem_addr)
                        2'b00: address_port_0 <= mem_wdata;
                        2'b01: address_port_1 <= mem_wdata;
                        2'b10: address_port_2 <= mem_wdata;
                        2'b11: address_port_3 <= mem_wdata;
                    endcase
                end
        end
endmodule

//-----------------------------------------------------------------------------
// Design Name : CSE591 Lab #3
// Function : Toplevel Design
//-----------------------------------------------------------------------------
module switch (
    // Global inputs                   // -------------------------------------
    input  wire         fast_clk,      // 1GHz Write Clock
    input  wire         slow_clk,      // 111MHz Read Clock
    input  wire         reset_b,       // Active Low, Asynchronous Reset
    // Memory inputs                   // -------------------------------------
    input  wire         mem_en,        // Memory enable
    input  wire         mem_rd_wr,     // Memory read/write
    input  wire [01:00] mem_addr,      // Memory address
    input  wire [07:00] mem_wdata,     // Data to be written to the memory interface
    output wire [07:00] mem_rdata,     // Data read from the memory interface
    // Input port                      // -------------------------------------
    output wire         data_stall,    //
    input  wire         data_valid,    // Data valid input
    input  wire [07:00] data,          // Data input
    // Output ports                    // -------------------------------------
    output wire         ready_0,       // Port 0 has data
    input  wire         read_0,        // Data valid input
    output wire [07:00] port0,         // Data input
    output wire         ready_1,       // Output has data
    input  wire         read_1,        // Data valid input
    output wire [7:0]   port1,         // Data input
    output wire         ready_2,       // Output has data
    input  wire         read_2,        // Data valid input
    output wire [7:0]   port2,         // Data input
    output wire         ready_3,       // Output has data
    input  wire         read_3,        // Data valid input
    output wire [7:0]   port3);        // Data input
    
    // Internal signals
    wire [07:00] address_port [03:00]; // Address for each port
    wire [03:00] router_ready;         // Router has data
    wire [07:00] router_port [03:00];  // Router data
    wire [03:00] fifo_empty;
    wire [03:00] fifo_full;            // FIFO has data
    wire [03:00] fifo_read;            //
    wire [08:00] fifo_port [03:00];    // FIFO data
    
    
    /*************************************************************************
    * Continuous Assignments                                                 *
    **************************************************************************/
    assign data_stall = |fifo_full;
    
    /*************************************************************************
    * Module Instantations                                                   *
    **************************************************************************/
    port_address_reg // Instantiate Port address registers
        i_port_address_reg
        (// Global Signals                   // -------------------------------
        .clk             (fast_clk),         //
        .reset_b         (reset_b),          //
        // Memory Configuration Interface    // -------------------------------
        .mem_en          (mem_en),           //
        .mem_rd_wr       (mem_rd_wr),        //
        .mem_addr        (mem_addr),         //
        .mem_rdata       (mem_rdata),        //
        .mem_wdata       (mem_wdata),        //
        // Output Mux'ing Interface          // -------------------------------
        .address_port_0  (address_port[0]),  //
        .address_port_1  (address_port[1]),  //
        .address_port_2  (address_port[2]),  //
        .address_port_3  (address_port[3])); //
    
    // Instantiate Port address registers
    packet_router
        i_packet_router (
        // Global Signals                    // -------------------------------
        .clk             (fast_clk),         // I
        .reset_b         (reset_b),          // I
        //                                   // -------------------------------
        .address_port_0  (address_port[0]),  // O [07:00]
        .address_port_1  (address_port[1]),  // O [07:00]
        .address_port_2  (address_port[2]),  // O [07:00]
        .address_port_3  (address_port[3]),  //
        .data_stall      (data_stall),       // I
        .data_valid      (data_valid),       //
        .data            (data),             //
        .ready_0         (router_ready[0]),  //
        .port0           (router_port[0]),   //
        .ready_1         (router_ready[1]),  //
        .port1           (router_port[1]),   //
        .ready_2         (router_ready[2]),  //
        .port2           (router_port[2]),   //
        .ready_3         (router_ready[3]),  //
        .port3           (router_port[3]));  //
    
    genvar i;
    generate
        for(i=0; i < 4; i = i + 1) begin: Asynchronous_FIFOs
                async_fifo #(9,1024,10) // Instantiate Packet FIFO's (1024 deep by 8 bits)
                i_fifo (
                // Global Signals                  // ---------------------------------
                .reset_b         (reset_b),        // I         Active-Low Asynchronous
                // Read Interface                  // ---------------------------------
                .rclk            (slow_clk),       // I         111MHz Read Clock
                .read            (fifo_read[i]),   // I         Advance read pointer
                .empty           (fifo_empty[i]),  // O         FIFO is Empty
                .rdata           (fifo_port[i]),   // O [08:00] Data being read
                // Write Interface                 // ---------------------------------
                .wclk            (fast_clk),       // I         1GHz Write Clock
                .write           (router_ready[i]),// I         Push Data Signal
                .wdata           ({~data_valid, router_port[i][7:0]}), // I [08:00] Data to be written
                .full            (fifo_full[i]));  // O         FIFO is Full
            end
    endgenerate
    
    // Instantiate Output SM's
    output_sm
        i_output_sm_0 (
        // Global Signals                  // ---------------------------------
        .clk             (slow_clk),       // I         111MHz Read Clock
        .reset_b         (reset_b),        // I         Active-Low Asynchronous
        // FIFO Interface                  // ---------------------------------
        .fifo_empty      (fifo_empty[0]),  //
        .fifo_read       (fifo_read[0]),   //
        .fifo_data       (fifo_port[0]),   //
        //
        .ready           (ready_0),        //
        .read            (read_0),         //
        .port            (port0));         //
    
    output_sm
        i_output_sm_1 (
        // Global Signals                   // ----------------------------
        .clk             (slow_clk),        // I         111MHz Read Clock
        .reset_b         (reset_b),         // I         Active-Low Asynchronous
        // FIFO Interface                   // ----------------------------
        .fifo_empty      (fifo_empty[1]),
        .fifo_read       (fifo_read[1]),
        .fifo_data       (fifo_port[1]),    // I [08:00]
        // Output Interface
        .ready           (ready_1),
        .read            (read_1),
        .port            (port1));
    
    output_sm
        i_output_sm_2
        (// Global Signals                  // ----------------------------
        .clk             (slow_clk),        // I         111MHz Read Clock
        .reset_b         (reset_b),         // I         Active-Low Asynchronous
        // FIFO Interface                   // ----------------------------
        .fifo_empty      (fifo_empty[2]),   //
        .fifo_read       (fifo_read[2]),    //
        .fifo_data       (fifo_port[2]),    //
        // Output Interface                 // -----------------------------
        .ready           (ready_2),         //
        .read            (read_2),          //
        .port            (port2));          //
    
    output_sm
        i_output_sm_3
        (// Global Signals                  // ----------------------------
        .clk             (slow_clk),       // I         111MHz Read Clock
        .reset_b         (reset_b),        // I         Active-Low Asynchronous
        // FIFO Interface                  // ----------------------------
        .fifo_empty      (fifo_empty[3]),
        .fifo_read       (fifo_read[3]),
        .fifo_data       (fifo_port[3]),
        // Output Interface                // -----------------------------
        .ready           (ready_3),
        .read            (read_3),
        .port            (port3));
    
endmodule
