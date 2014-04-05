`define ENABLE      8'd1
`define DISABLE     8'd0
`define TRUE        8'd1
`define FALSE       8'd0

// Randomization Modes
`define ENABLE_SET      8'd2
`define ENABLE_SEQ      8'd1
`define DISABLE         8'd0

class int_constraint;
    // variables (properties)
    byte unsigned mode;
    byte unsigned max;
    byte unsigned min;

    // functions (methods)
    function new( );
        constraint_mode_i(`ENABLE);
        set_max_constraint(8'd255);
        set_min_constraint(8'd0);
    endfunction

    function void constraint_mode_i(input byte unsigned _mode);
        this.mode = _mode;
    endfunction

    function void set_max_constraint(input byte unsigned _max);
        this.max = _max;
    endfunction

    function void set_min_constraint(input byte unsigned _min);
        this.min = _min;
    endfunction
endclass : int_constraint

class randi;
    // properties
    byte unsigned mode;
    byte unsigned value;
    int_constraint cons;
    static byte unsigned empty_set [];
    byte unsigned set [];

    // methods
    function new (byte unsigned _mode = `ENABLE, byte unsigned _value = 8'h00, byte unsigned _set [] = empty_set);
        this.mode  = _mode;
        this.set   = _set;
        this.value = _value;
        cons = new ( );
    endfunction

    function void rand_mode_i(input byte unsigned _mode);
        this.mode = _mode;
    endfunction

    function byte unsigned randomize_i( );
        if (this.mode == `ENABLE) begin
            if (this.cons.mode == `DISABLE) begin
                // set random value
                this.value = {$random};
                return `TRUE;
            end
            else begin
                // set random value
                for(int i = 0; i < 20; i++) begin
                    this.value = this.cons.min + {$random} % (this.cons.max - this.cons.min + 1);
                    if((this.value >= this.cons.min) && (this.value <= cons.max)) begin
                        return `TRUE;
                        break;
                    end
                    else begin
                        $display("%m: could not generate constrained random value: %d >= %d <= %d", this.cons.min, this.value, this.cons.max);
                        return `FALSE;
                    end
                end
            end
        end
        else if (this.mode == `ENABLE_SET) begin
            this.value = set [{$random} % set.size];
            return `TRUE;
        end
        else begin
            $display("randi mode is disabled. enable to use randomize_i");
            return `FALSE;
        end
    endfunction : randomize_i
endclass : randi
