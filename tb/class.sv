/* Here we will define all the classes required:
The Packet class,
The Driver class,
The ScoreBoard class,
The Receiver class,
The Environment class
*/ 

// Following are the macros which will be required throughout many classes : Globally defining them
`define FCS_GOOD 1'b1
`define FCS_BAD 1'b0
`define LEN_GOOD 1'b1 
`define LEN_BAD 1'b0
`define TOTAL_OP_PORTS 4 

// Following global variable will be used to randomize output port addresses
randi port_address[`TOTAL_OP_PORTS];
//global error count
int err_cnt = 0;
//global received packet count
int rec_pkts = 0;
//global dropped packets count
int dropped_pkts = 0;

// ##################  Packet class	  #########################	
class Packet;	  

randi Length_type;	// random class members
randi FCS_type;
randi Dst_addr_rand_mode;	// will be used to randomize the destination address values

randi DA;
randi SA;
randi Length;
randi Data[]; // dynamic array to hold variable number of data bytes
randi FCS;	 // _frame check sequence which will decide whether packet contents are correct

//class constructor: will be used to create all objects of member variables
function new();
begin
	Length_type = new();
	FCS_type = new();
	DA = new();
	SA = new();
	Length = new();
	FCS = new();
end
endfunction: new

// ##############	Other functions of Packet class #################

// function used to display packet contents in formatted order
function void display();
begin
	$display("%0t: Inside Packet: ################# Type of Packet ################## ", $time);
	$display(" Packet FCS type : %h ", FCS_type.value);
	$display(" Packet Length type : %h ", Length_type.value);
	$display("################# Contents of Packet ##################");
	$display(" Source address of packet : %h ", SA.value);
	$display(" Destination address of packet : %h ", DA.value);	
	$display(" Length of packet : %d ", Length.value);
	foreach (Data[i])
		$write("%0d : %0h ",i, Data[i].value);
	$display(" \nPacket FCS value : %h ", FCS.value);
	$display("################# Packet Contents finished !! ##################");
end
endfunction : display

// function used to convert from Packet --> frame to be sent to the DUT
function void pack(ref logic [7:0] _frame[]);		// ref argument: changes can be seen globally 
begin														
	_frame = new[Data.size + 4];  // data + DA + SA + length + FCS
	_frame[0] = DA.value;
	_frame[1] = SA.value;
	_frame[2] = Length.value;
	foreach (Data[i])
		_frame[i+3] = Data[i].value;
	_frame[Data.size + 3] = FCS.value;
	$display ("%0t: Inside Packet : Frame constructed from packet", $time);
end
endfunction	: pack

// function will be used to convert from Frame --> Packet
function void unpack(ref logic [7:0] _frame[]);
begin
	DA.value = _frame[0];
	SA.value = _frame[1];
	Length.value = _frame[2];				   
	if(Length.value != _frame.size - 4)	 //based on the value of this field check the length type: good or bad
		Length_type.value = `LEN_BAD;
	// populate the packet data from the frame received	
	Data = new[_frame.size - 4];
	foreach (Data[i]) begin
		Data[i] = new();
		Data[i].value = _frame[i+3];	
	end
	
	FCS.value = _frame[_frame.size - 1];						
	$display("%0t: Inside Packet : Packet constructed from frame", $time);
end
endfunction	: unpack

// function used to compare received packet arguments with this packet arguments
function bit compare(Packet p);
begin 
	compare = 1;
		if(p.SA.value !== this.SA.value) begin
			$display("%0t: Inside Packet : Error !! Source addresses do not match !", $time);
			compare = 0; 
		end	
		
		if(p.DA.value !== this.DA.value) begin
			$display("%0t: Inside Packet : Error !! Destination addresses do not match !", $time);
			compare = 0; 
		end
		
		if(p.Length.value !== this.Length.value) begin
			$display("%0t: Inside Packet : Error !! Length fields do not match !", $time);
			compare = 0; 
		end
		
		foreach (Data[i]) begin
			if(p.Data[i].value !== this.Data[i].value) begin
			$display("%0t: Inside Packet : Error !! Data bytes do not match !", $time);
			compare = 0; 
			end				   
		end
		
		if(p.FCS.value !== this.FCS.value) begin
			$display("%0t: Inside Packet : Error !! FCS fields do not match !", $time);
			compare = 0; 
		end
	end	 

endfunction	: compare

// function will be used to randomize the packet contents
function bit randomize_pkt();
bit rand_status = `TRUE; // will determine if randomization was successful
begin 
	// use the randomize_i method of randi class to randomize the FCS_type
	FCS_type = new(`ENABLE_SET,,'{`FCS_GOOD, `FCS_BAD});
	rand_status = rand_status & FCS_type.randomize_i( );  
	
	// use the randomize_i method of randi class to randomize the Length_type
	Length_type = new(`ENABLE_SET,,'{`LEN_GOOD, `LEN_BAD});
	rand_status = rand_status & Length_type.randomize_i( );
	
	// use the randomize_i method of randi class to randomize the Length_type
	Dst_addr_rand_mode = new(`ENABLE_SET,,'{`ENABLE, `ENABLE_SET});
	rand_status = rand_status & Dst_addr_rand_mode.randomize_i( );
	
	//constrain the source addresses to a random address
	SA.cons.constraint_mode_i(`ENABLE);
	SA.cons.set_max_constraint(8'd255);	// address will be between 0 to 255
	SA.cons.set_min_constraint(8'd0);
	rand_status = rand_status & SA.randomize_i( );
	
	//constrain the destination addresses to generate a mixture of good and bad packets
	DA = new(Dst_addr_rand_mode.value,,'{port_address[0].value, port_address[1].value, port_address[2].value, port_address[3].value} );
	DA.cons.constraint_mode_i(`ENABLE);
	DA.cons.set_max_constraint(8'd255);
	DA.cons.set_min_constraint(8'd0);
	rand_status = rand_status & DA.randomize_i( );
	
	//constain the data length to be between 1 to 255 bytes
	Length = new();
	Length.cons.constraint_mode_i(`ENABLE);
	Length.cons.set_max_constraint(8'd255);
	Length.cons.set_min_constraint(8'd1);
	rand_status = rand_status & Length.randomize_i( );
	Data = new[Length.value];
	for(int i = 0; i < Length.value ; i++) 
	begin
		Data[i] = new( );
		Data[i].cons.constraint_mode_i(`ENABLE);
		Data[i].cons.set_max_constraint(8'd255);
		Data[i].cons.set_min_constraint(8'd0);
		rand_status = rand_status & Data[i].randomize_i( );
	end									
	
	//constrain the FCS field value based on whether fcs_type is FCS_GOOD / FCS_BAD
	if(FCS_type.value == `FCS_BAD)
		begin
			FCS.cons.constraint_mode_i(`DISABLE);
			rand_status = rand_status & FCS.randomize_i( );
		end
	else
		begin
			if(Length_type.value == `LEN_GOOD)
				FCS.value = fcs_value;
			else begin
				FCS.cons.constraint_mode_i(`DISABLE);
				rand_status = rand_status & FCS.randomize_i( );				
			end				
		end	
		
	// as we have already set length value as valid which will be chosen when length_type will be LEN_GOOD	
	//give incorrect length value when length_type will be LEN_BAD
	if(Length_type.value == `LEN_BAD)
		begin
			Length.cons.constraint_mode_i(`DISABLE); // this is achieved by setting the constraint OFF 
			rand_status = rand_status & Length.randomize_i( );		   // and again randomizing length
		end
		
	return rand_status;	// return the randomization status whether success or failure
end	
endfunction: randomize_pkt

// function calculates fcs: basically just does a XOR of all the packet fields
function byte unsigned fcs_value;
begin
	fcs_value = 0;
	fcs_value = fcs_value ^ DA.value;
	fcs_value = fcs_value ^ SA.value;
	fcs_value = fcs_value ^ Length.value;
	for (int i = 0; i < Length.value; i++)
		fcs_value = fcs_value ^ Data[i].value;
	fcs_value = fcs_value ^ FCS.value;
	end
endfunction : fcs_value
endclass : Packet	
// ##################  Packet class	Ends here  #########################

// ##################  Driver class  ######################### 

class Driver;

//instantiate the mailbox array: 4 mailboxes for 4 scoreborads
mailbox #(Packet) dv2sb[`TOTAL_OP_PORTS];

//constructor will be used to assign the member mailboxes
function new(ref mailbox #(Packet) dv2sb[`TOTAL_OP_PORTS]);
begin
	if(dv2sb[0] == null) begin
		$display ("%0t: Inside Driver : ERROR !! : Mailbox 0 is null",$time);
		$finish;
	end
	if(dv2sb[1] == null) begin
		$display ("%0t: Inside Driver : ERROR !! : Mailbox 1 is null",$time);
		$finish;
	end
	if(dv2sb[2] == null) begin
		$display ("%0t: Inside Driver : ERROR !! : Mailbox 2 is null",$time);
		$finish;
	end
	if(dv2sb[3] == null) begin
		$display ("%0t: Inside Driver : ERROR !! : Mailbox 3 is null",$time);
		$finish;
	end
	else  begin	  // if all mailboxes are valid
		this.dv2sb[0] = dv2sb[0];
		this.dv2sb[1] = dv2sb[1];
		this.dv2sb[2] = dv2sb[2];
		this.dv2sb[3] = dv2sb[3];
	end
end
endfunction : new

// task will be used to drive DUT with frames 
task automatic send(input int num);			// num = number of clock cyles to be run
	begin
		Packet p;													
		logic [7:0] dv_frame[];								
		int unsigned i;
		repeat(num) begin												
			i = 0;
			$display("%0t: Inside Driver : Send task started",$time);
			repeat(3) @$root.ip_if_top.cb;	// wait for at least 3 clock cycles before sending new packet as per given specifications							
			
			//Create Packet, randomize it, and pack it into 32 bit frame.
			p = new();												//create new object and randomize it
			if(p.randomize_pkt()) begin
				$display("%0t: Inside Driver : Packet randomized successfully!",$time);
				p.display(); //display packet contents
				
				p.pack(dv_frame);	//pack the packet into the frame
				
				//Drive the DUT with packed frame
				while(i < dv_frame.size) begin
					@$root.ip_if_top.cb; //wait till outputs are sampled									
					$root.ip_if_top.cb.data_valid <= 1'b1;	// assert the data_valid signal
					$root.ip_if_top.cb.data <= dv_frame[i];
					if($root.ip_if_top.cb.data_stall == 0)
						i++;
				end
				
				@$root.ip_if_top.cb; // important to wait for last TB output to be sampled											
				$root.ip_if_top.cb.data_valid <= 1'b0;	//deassert the data_valid signal
				$root.ip_if_top.cb.data <= 'd0;
								
				//Based on Predictor module output decide which sb to send packet to or to drop the packet !!
				if($root.pred_top.drop_pkt == 0)  begin
					case($root.pred_top.final_addr)
						port_address[0].value: begin
							dv2sb[0].put(p);	
							$display("%0t: Inside Driver: A packet is sent to Scoreboard %0h", $time, port_address[0].value);
						end
						port_address[1].value: begin
							dv2sb[1].put(p);	
							$display("%0t: Inside Driver: A packet is sent to Scoreboard %0h", $time, port_address[1].value);
						end
						port_address[2].value: begin
							dv2sb[2].put(p);	
							$display("%0t: Inside Driver: A packet is sent to Scoreboard %0h", $time, port_address[2].value);
						end
						port_address[3].value: begin
							dv2sb[3].put(p);	
							$display("%0t: Inside Driver: A packet is sent to Scoreboard %0h", $time, port_address[3].value);
						end
					endcase
				end
				else begin
					dropped_pkts++;
					$display("%0t: Inside Driver: Destination address INVALID. Packet will NOT be sent to Scoreboard", $time);		
					end
			end
			else begin
				$display("%0t: Inside Driver: ERROR !! Packet could not be randomized.", $time);
				$root.err_cnt++; //keep a track of errors for reporting purpose
			end
		end
	end
endtask: send
endclass : Driver
// ##################  Driver class ends here ######################### 

// ##################  Receiver class  ######################### 

class Receiver;
// receiver side mailbox
mailbox #(Packet) rc2sb;
int unsigned rc_port_address;
int rc_port_id;

//constructor will be used to assign to the mailbox and the port addresses
function new(input mailbox #(Packet) rc2sb, input int unsigned _port_addr);
begin
	if(rc2sb == null) begin
		$display ("%0t: Inside Receiver : ERROR !! : Mailbox is null",$time);
		$finish;
	end
	else  begin	 // assign the port addresses and mailbox
		this.rc2sb = rc2sb;
		case (_port_addr)
			'd0: this.rc_port_address = $root.port_address[0].value;
			'd1: this.rc_port_address = $root.port_address[1].value;
			'd2: this.rc_port_address = $root.port_address[2].value;
			'd3: this.rc_port_address = $root.port_address[3].value;
			default: $display("%0t: Inside Receiver : ERROR !! : Port address is invalid",$time);
		endcase													   
		this.rc_port_id = _port_addr;
	end
end
endfunction : new

//task to receive the frames from the DUT
task automatic rec();						
	begin
		Packet p;							
		logic[7:0] rc_frame[];		
		
		forever begin	// repeat for user defined fast clock cycles
			$display("%0t: Inside Receiver : Expecting a packet on %0h port.",$time, this.rc_port_address);
			
			// Read from this.rc_port_address and figure out 
			case(this.rc_port_id)
				'd0: begin
					repeat(2) @$root.op_if_top[0].cb;	// wait for 2 clock cycles 
					wait($root.op_if_top[0].cb.ready);
					$root.op_if_top[0].cb.read <= 1'b1;	//assert read signal to pick up data out of read ports
					repeat(2) @$root.op_if_top[0].cb;
					while($root.op_if_top[0].cb.ready) begin
						rc_frame = new[rc_frame.size + 1](rc_frame);  //correct way to build up dynamically changing array
						rc_frame[rc_frame.size - 1] = $root.op_if_top[0].cb.port;
						@$root.op_if_top[0].cb;
					end															  
					$root.op_if_top[0].cb.read <= 1'b0;	//de-assert read after completing the read operation
					@$root.op_if_top[0].cb;
				end
				// repeat the above chunk of code for each case expression value !
				'd1: begin
					repeat(2) @$root.op_if_top[1].cb;							
					wait($root.op_if_top[1].cb.ready);
					$root.op_if_top[1].cb.read <= 1'b1;						
					repeat(2) @$root.op_if_top[1].cb;
					while($root.op_if_top[1].cb.ready) begin
						rc_frame = new[rc_frame.size + 1](rc_frame);		
						rc_frame[rc_frame.size - 1] = $root.op_if_top[1].cb.port;
						@$root.op_if_top[1].cb;
					end															  
					$root.op_if_top[1].cb.read <= 1'b0;						
					@$root.op_if_top[1].cb;					
				end
				
				'd2: begin
					repeat(2) @$root.op_if_top[2].cb;							
					wait($root.op_if_top[2].cb.ready);
					$root.op_if_top[2].cb.read <= 1'b1;						
					repeat(2) @$root.op_if_top[2].cb;
					while($root.op_if_top[2].cb.ready) begin
						rc_frame = new[rc_frame.size + 1](rc_frame);		
						rc_frame[rc_frame.size - 1] = $root.op_if_top[2].cb.port;
						@$root.op_if_top[2].cb;
					end															  
					$root.op_if_top[2].cb.read <= 1'b0;						
					@$root.op_if_top[2].cb;					
				end
				
				'd3: begin								
					repeat(2) @$root.op_if_top[3].cb;							
					wait($root.op_if_top[3].cb.ready);
					$root.op_if_top[3].cb.read <= 1'b1;						
					repeat(2) @$root.op_if_top[3].cb;
					while($root.op_if_top[3].cb.ready) begin
						rc_frame = new[rc_frame.size + 1](rc_frame);		
						rc_frame[rc_frame.size - 1] = $root.op_if_top[3].cb.port;
						@$root.op_if_top[3].cb;
					end															  
					$root.op_if_top[3].cb.read <= 1'b0;						
					@$root.op_if_top[3].cb;										
				end
			endcase
			
			// after the complete frame is received
			//Create a new packet and unpack data
			p = new();
			p.unpack(rc_frame);								//unpack into the packet p
			p.display();
			
			// Send the Packet to the scoreboard.
			rc2sb.put(p);
			
			//display success
			$display("%0t: Inside Receiver : Received a packet successfully at port: %0h ",$time, this.rc_port_address); 
			rc_frame.delete();	// delete the array object 
		end
	end
endtask: rec
endclass: Receiver 
// ##################  Receiver class ends here ######################### 

// ##################  Scoreboard class  ######################### 

class Scoreboard;
//declaring mailbox handles
mailbox #(Packet) sb2dv, sb2rc;	
int unsigned sb_port_addr;

//class constructor to assign the mailboxes 
function new(input mailbox #(Packet) sb2dv, sb2rc, input int unsigned _port_addr);
begin
	if(sb2dv == null) begin
		$display ("%0t: Inside ScoreBoard : ERROR !! : Driver side mailbox is null",$time);
		$finish;
	end
	else if(sb2rc == null) begin
		$display ("%0t: Inside ScoreBoard : ERROR !! : Receiver side mailbox is null",$time);
		$finish;
	end
	else  begin
		this.sb2dv = sb2dv;
		this.sb2rc = sb2rc;
		case (_port_addr)
			'd0: this.sb_port_addr = $root.port_address[0].value;
			'd1: this.sb_port_addr = $root.port_address[1].value;
			'd2: this.sb_port_addr = $root.port_address[2].value;
			'd3: this.sb_port_addr = $root.port_address[3].value;
			default: $display("%0t: Inside ScoreBoard : ERROR !! : Port Address is invalid",$time);
		endcase
	end	   
end
endfunction: new	

//task which will wait for packets from driver and receiver and analyze them
task automatic start();
Packet rc_pkt, dv_pkt;
begin															  
	$display("%0t: Inside ScoreBoard : Start task initiated for Port with address: %0h",$time, this.sb_port_addr);
	forever	begin
			// blocking wait for a packet from the receiver which is slow domain	
			sb2rc.get(rc_pkt);
			$display("%0t: Inside ScoreBoard : Received a packet from receiver with port address: %0h",$time, this.sb_port_addr);
			
			//now take out pkt from drvr
			sb2dv.get(dv_pkt);
			$display("%0t: Inside ScoreBoard : Received a packet from driver for port address: %0h",$time, this.sb_port_addr);
			
			//compare both the packets
			if(rc_pkt.compare(dv_pkt))	begin
				$display("%0t: Inside ScoreBoard : Packet received at port address %0h matches with driver packet!!",$time, this.sb_port_addr);
				rec_pkts++; // keep count of packets received from receiver	
			end
			else begin
				$display("%0t: Inside ScoreBoard : ERROR : Packets received do NOT match !! : Port address: %0h",$time, this.sb_port_addr);
				$root.err_cnt++;	//increment error count
				rec_pkts++; // keep count of packets received from receiver
			end
		end
end
endtask: start
endclass: Scoreboard
// ##################  Scoreboard class ends here ######################### 

// ##################  Environment class  #########################  

class Environment; 
int num; // number of clocks for which driver has to run
//Declare member variables
Driver dv; 											
Receiver rc[`TOTAL_OP_PORTS];	// 4 receivers for 4 DUT ports	
	
// 4 scoreboards needed to check whether packet was received from the correct port -- IMPORTANT !!
Scoreboard sb[`TOTAL_OP_PORTS];

mailbox #(Packet) dv_sb[`TOTAL_OP_PORTS], rc_sb[`TOTAL_OP_PORTS]; // 4 mailboxes for the scoreboards

//class constructor to assign all the members variables
function new(int _num);
begin
	$display("@%0t : Environment : created env object", $time);
	this.num = _num;
	//We need to randomize the port address and also ensure that they are not equal !
	foreach(port_address[i]) begin
		port_address[i] = new();
		port_address[i].cons.constraint_mode_i(`ENABLE);
		port_address[i].cons.set_max_constraint(8'd255);
		port_address[i].cons.set_min_constraint(8'd0);
	end

	if(port_address[0].randomize_i()) begin
		$display("%0t: Inside Environment : Port 0 configued to: %0h address", $time, port_address[0].value);
		do
			void'(port_address[1].randomize_i());
		while(port_address[1].value == port_address[0].value);
		$display("%0t: Inside Environment : Port 1 configued to: %0h address", $time, port_address[1].value);
		// 2 distinct values obtained and assigned
		do
			void'(port_address[2].randomize_i());
		while(port_address[2].value == port_address[0].value || port_address[2].value == port_address[1].value);
		$display("%0t: Inside Environment : Port 2 configued to: %0h address", $time, port_address[2].value);
		// 3 distinct values obtained and assigned
		do
			void'(port_address[3].randomize_i());
		while(port_address[3].value == port_address[0].value || port_address[3].value == port_address[1].value
		|| port_address[3].value == port_address[2].value);
		$display("%0t: Inside Environment : Port 3 configued to: %0h address", $time, port_address[3].value);
	end
end
endfunction: new

// Other tasks
// Build task: used to instantiate the test bench components
task build();			
begin
	$display("%0t: Inside Environment : Build task started", $time);
	for (int i=0; i<`TOTAL_OP_PORTS; i++) begin
		dv_sb[i] = new;										//in this case will create an array of 4 mailboxes
		rc_sb[i] = new;
	end
	
	dv = new(dv_sb);										//instantiate driver
	for (int i=0; i<`TOTAL_OP_PORTS; i++)
	begin
		rc[i] = new(rc_sb[i], i);
		sb[i] = new(dv_sb[i], rc_sb[i], i);					//instantiate 4 receivers and 4 scoreboards
	end
	$display("%0t: Inside Environment : Success : Build task completed", $time);
end
endtask: build

//This task is used to reset the DUT 
task reset();			
begin		  											  
	$display("%0t: Inside Environment : Reset DUT task started", $time);
	
	//Reset all the DUT inputs
	$root.ip_if_top.cb.data				<= 'd0;
	$root.ip_if_top.cb.data_valid		<= 'd0;
	$root.op_if_top[0].cb.read			<= 'd0;
	$root.op_if_top[1].cb.read			<= 'd0;
	$root.op_if_top[2].cb.read 			<= 'd0;
	$root.op_if_top[3].cb.read 			<= 'd0;	
	$root.mem_if_top.cb.mem_wdata 		<= 'd0;
	$root.mem_if_top.cb.mem_addr 		<= 'd0;
	$root.mem_if_top.cb.mem_rd_wr		<= 'd0;
	$root.mem_if_top.cb.mem_en 			<= 'd0;
	
	//Assert the reset signal of the DUT and Memory
	$root.mem_if_top.mem_rstn 			<= 1'b0;
	$root.ip_if_top.reset_b 			<= 1'b0;
	repeat(5) @$root.ip_if_top.cb;		// stay in reset for 5 clock cycles of the fast clock
	
	//De-Assert the reset signal of the DUT and Memory
	$root.mem_if_top.mem_rstn 			<= 1'b1;
	$root.ip_if_top.reset_b 			<= 1'b1;
	
	$display("%0t: Inside Environment : Success : Reset task completed", $time);
end
endtask: reset	

// this task will be used to configure DUT signals
task cfg_dut();
begin
	$display("%0t: Inside Environment : Configure DUT task started", $time);
	
	//Configure DUT to have the above port addresses
	$root.mem_if_top.cb.mem_en 			<= 1'b1;		//enable switch memory
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_rd_wr 		<= 1'b1;		// mem write enable			****SPEC ERRROR****
	
	//store the port address earlier generated into the memory
	// to simplify: addr 0 will hold the random address generated for port_addr[0]
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_addr 		<= 2'h0;
	$root.mem_if_top.cb.mem_wdata 		<= port_address[0].value;
	$display("@%0t: Environment : Port 0 Address = %h", $time, port_address[0].value);
	// to simplify: addr 1 will hold the random address generated for port_addr[1]
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_addr 		<= 2'h1;
	$root.mem_if_top.cb.mem_wdata 		<= port_address[1].value;
	$display("@%0t: Environment : Port 1 Address = %h", $time, port_address[1].value);
	// to simplify: addr 2 will hold the random address generated for port_addr[2]
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_addr 		<= 2'h2;
	$root.mem_if_top.cb.mem_wdata 		<= port_address[2].value;
	$display("@%0t: Environment : Port 2 Address = %h", $time, port_address[2].value);
	// to simplify: addr 3 will hold the random address generated for port_addr[3]
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_addr 		<= 2'h3;
	$root.mem_if_top.cb.mem_wdata 		<= port_address[3].value;
	$display("@%0t: Environment : Port 3 Address = %h", $time, port_address[3].value);
	
	// De-assert the mem_en signal for this run of the tests as the addresses would be stable during each iteration
	@$root.mem_if_top.cb;
	$root.mem_if_top.cb.mem_en 			<= 1'b0;
	
	$display("%0t: Inside Environment : Success : Configure DUT task completed. Output ports have addresses assigned !", $time);
end
endtask: cfg_dut

//Start the Scoreboards , Driver and Receivers parallely
task start();					
begin														  
	$display("%0t: Inside Environment : start task started", $time); 
	fork			
		//Start all the Driver, Receiver and Scoreboard objects
		sb[0].start();
		sb[1].start();
		sb[2].start();
		sb[3].start();
		
		dv.send(this.num);			// number of cycles for which driver has to run
		
		rc[0].rec();
		rc[1].rec();
		rc[2].rec();
		rc[3].rec();					   
	join_any
	$display("%0t: Inside Environment : Success : start task completed", $time);
end
endtask: start											 

// task waits for a long time so that all the driven packets have been received
task wait_for_end ( );
begin 
	$display("%0t: Inside Environment : wait_for_end task started", $time);
	repeat(2000*this.num)	@$root.ip_if_top.cb;				//wait for some clock cyles
	$display("%0t: Inside Environment : Success: wait_for_end task completed", $time);
end
endtask : wait_for_end

//Call the build, reset and start tasks.
task run();						
begin 													
	$display("%0t: Inside Environment : Run task started", $time); 
	build();		
	reset();		
	cfg_dut();		
	start();		
	wait_for_end();	
	report();	
	
end
endtask: run											 

// Will report the status of the test based on number of errors encountered
task report ( );
begin
	int dr_mb_pkts = 0;
	//any packets which are remaining in the driver mailbox are counted as errors
	foreach(dv_sb[i]) begin
		dr_mb_pkts += dv_sb[i].num();
	end	
	if(dr_mb_pkts != 0) begin
		$display("%0t: Inside Environment: Packets inside driver mailbox : %0d",$time,dr_mb_pkts);
		$root.err_cnt  += dr_mb_pkts; // add to global error count
	end
		
	//Generate report
	$display("%0t: Inside Environment : Report task started", $time); 
	if($root.err_cnt == 0)	begin
		$display("%0t: Inside Environment : Success : All tests have passed", $time);
		$display("%0t: Inside Environment : Total packets received: %0d", $time,$root.rec_pkts); 
		$display("%0t: Inside Environment : Total dropped packets: %0d", $time,$root.dropped_pkts);
	end
	else   begin
		$display("%0t: Inside Environment : Failure : Errors encountered: %0d", $time,$root.err_cnt);
		$display("%0t: Inside Environment : Total packets received: %0d", $time,$root.rec_pkts); 
		$display("%0t: Inside Environment : Total dropped packets: %0d", $time,$root.dropped_pkts);
	end
	$display("%0t: Inside Environment : Report task completed", $time); 
end
endtask : report
endclass: Environment
// ##################  Environment class ends here ######################### 
