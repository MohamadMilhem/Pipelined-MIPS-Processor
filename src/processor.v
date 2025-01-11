/* 
 * description: The module cpu includes the connection of the datapath and controller.
 * inputs: clk, reset, dataout, instruction	
 * output: MemRd, MemWr, datain, instrAddr, dataAddr
 */
module cpu(input         clk, reset,
	input [15:0] dataout, instruction,
	output  MemRd, MemWr,
	output [15:0] datain, instrAddr, dataAddr);	  
	
//CONTROLLER
//output [2:0] EXALUOp,
//output  [1:0] RegSrc2, PCSrc,
//output RegSrc1, RegDest, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, EXALUSrc, EXMemDataIn, 
//output WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR

//ALU
//output	Z, stall,
//output [15:0] instrAddrF,
//output [15:0] DataInM, ALUOutM,
//output [3:0]  Opcode,
//output [2:0] Function


				

  wire [3:0] opcode;
  wire [2:0] funcode;
  wire Z, stall;
  wire [2:0] EXALUOp;
  wire [1:0] RegSrc2 , PCSrc;
  wire RegSrc1, RegDest, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, EXALUSrc, EXMemDataIn, 
		WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR;
		
controller c(clk, reset,
			opcode,
			funcode,
			Z,stall,
			EXALUOp,
			RegSrc2, PCSrc,
			RegSrc1,RegDest, MEMWBdata, sign_extend_imm, MemRd, MemWr,  EXALUSrc, EXMemDataIn,
		  	WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR);
			  
			  
datapath dp(clk, reset, 
			EXALUOp,
			RegSrc2, PCSrc,
			sign_extend_imm, RegSrc1, RegDest, MEMWBdata,EXALUSrc, EXMemDataIn,
			WBRegWr, EXRegWr, MEMRegWr, DMemWr, EXMemRd, EXMemWr, MEMMemRd, MEMMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR, 
			dataout,
			instruction,
			Z, stall,
			instrAddr, 
			datain, 
			dataAddr, 
			opcode,
			funcode);
			  
endmodule