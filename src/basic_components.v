
/* 
 * description: an ALU that performs three operations: addition, ANDing and subtraction,
 * based on the control singal ALUOp	
 * inputs: a, b, alucont
 * output: result
 */
module alu(input	[15:0] a, b, 
		   input	[2:0]  alucont,
           output reg [15:0] result);


  always@(*)
    case(alucont[1:0])
      2'b000: result <= a & b;
      2'b001: result <= a + b;
      2'b010: result <= a - b;
	  2'b011: result <= a << b;
	  2'b100: result <= a >> b;
	  2'b101: result <= a - 1;
      default: result <= 16'bxxxxxxxxxxxxxxxx;
	
    endcase
endmodule

/* 
 * description: a register file consisting of 7 16-bit registers with two read ports and one write port.	
 * inputs: clk, RegWr, Rs1, Rs2, Rd, WBus
 * output: Bus1, Bus2
 */
 /*
module regfile(input         clk, 
               input         RegWr, 
               input  [2:0]  Rs1, Rs2, Rd, 
               input  [15:0] WBus, 
               output reg [15:0] Bus1, Bus2);

	reg [15:0] rf[7:0];
	

	always @(posedge clk)
		if (RegWr)
			rf[Rd] <= WBus ;
 
			
  	assign Bus1 =  rf[Rs1];
  	assign Bus2 =  rf[Rs2];
			  

  
endmodule				
*/

module regfile(
    input         clk, 
    input         RegWr, 
    input  [2:0]  Rs1, Rs2, Rd, 
    input  [15:0] WBus, 
    output reg [15:0] Bus1, Bus2
);

    reg [15:0] rf[7:0]; // Register file with 8 registers of 16-bit each

    // Initialize register file
    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1)
            rf[i] = 16'h0000;
    end

    // Write logic
    always @(posedge clk) begin
        if (RegWr)
            rf[Rd] <= WBus;
    end

    // Read logic
    always @(negedge clk) begin
        Bus1 = rf[Rs1];
        Bus2 = rf[Rs2];
    end

endmodule


/* 
 * description: a two-input adder that produces the addition result.	
 * inputs: a, b
 * output: y
 */
module adder(input  [15:0] a, b,
             output [15:0] y);

  assign y = a + b;
endmodule

/* 
 * description: a two-input subtractor that produces the subtraction result along with zero, negative and overflow flags.	
 * inputs: a, b
 * output: Z, N, V, y
 */
module subtractor(input  [15:0] a, b,
				output	Z, 
             	output [15:0] y);
				 
	assign y = a - b;
	assign Z = ~|y;

endmodule
				 
/* 
 * description: a sign extender that extends an n-bit input to produce a 16-bit output based on a control signal.	
 * inputs: a, extendcont
 * output: y
 */
module signext #(parameter WIDTH = 6) // sign extender with input of size WIDTH, extend control signal, and output of size 16.
				(input  [WIDTH-1:0] a,
				input	extendcont,
				output [15:0] y);
              
  assign y = extendcont ? {{(16-WIDTH){a[WIDTH-1]}}, a} : {{(16-WIDTH){1'b0}}, a};
endmodule

/* 
 * description: an n-bit register with reset.	
 * inputs: clk, reset, d
 * output: q
 */
module flopr #(parameter WIDTH = 16) // register with reset
              (input                  clk, reset,
               input      [WIDTH-1:0] d, 
               output reg [WIDTH-1:0] q);

  always @(negedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

/* 
 * description: an n-bit register with enable and reset.	
 * inputs: clk, reset, en, d
 * output: q
 */
module flopenr #(parameter WIDTH = 16)
                (input                  clk, reset,
                 input                  en,
                 input      [WIDTH-1:0] d, 
                 output reg [WIDTH-1:0] q);
 
  always @(negedge clk, posedge reset)
    if      (reset) q <= 0;
    else if (en)    q <= d;
endmodule

/* 
 * description: a 2*1 multiplexer that selects one of the four input data based on one selection bit. 	
 * inputs: d0, d1, s
 * output: y
 */
module mux2 #(parameter WIDTH = 16)
             (input  [WIDTH-1:0] d0, d1, 
              input              s, 
              output [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

/* 
 * description: a 4*1 multiplexer that selects one of the four input data based on two selection bits. 	
 * inputs: d0, d1, d2, d3, s
 * output: y
 */
module mux4 #(parameter WIDTH = 16)
             (input  [WIDTH-1:0] d0, d1, d2, d3,
              input  [1:0]       s, 
              output [WIDTH-1:0] y);

  assign y = s[1] ? (s[0] ? d3 : d2) : (s[0] ? d1 : d0); 
endmodule