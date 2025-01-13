/* 
 * description: this module acts as the data memory that stores the data loaded from CPU, specifically the register file.
 * inputs: clk, MemRd, MemWr, address, DataIn	
 * output: Dataout
 */
module dmem(
			input clk, MemRd, MemWr,
			input [15:0] address,
			input [15:0] DataIn,
            	output reg [15:0] Dataout );

	reg  [15:0] DRAM[31:0];

	// Initialize RAM
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            DRAM[i] = i;
    end
	
	
	always @(negedge clk) // may have to switch work done negedge and posedge 
    	
		if (MemWr)
			begin
      			DRAM[address[4:0]] <= DataIn[15:0] ;
			end
	
	always @(posedge MemRd)
		
		if (MemRd)
			Dataout <= DRAM[address[4:0]]; 
			
endmodule


/* 
 * description: this module acts as the instruction memory that stores the instruction sequence
 * to be executed on the processor	
 * inputs: clk, PC
 * output: instruction
 */
module imem(
			input  [15:0] PC,
            output reg [15:0] instruction );

	reg  [15:0] IRAM[63:0] ;

 	initial
    		begin
      		$readmemh("memfile.dat",IRAM) ;
    		end
	
	always@(*)
 	 	instruction <=  IRAM[PC];
  
endmodule