`timescale 1ns /1ns

program automatic test();		  
	// declare an environment handle
	Environment env;
	
	initial 
		begin					  
			$display("Inside Test: Program block started\n");
			env = new(20); // create environment object			
			env.run(); 
			
			#800;
			$finish;
		end
	
	final begin
		$display("%0d : Inside Test: Test of Super Switch completed!\n",$time); 
	end
	
endprogram : test