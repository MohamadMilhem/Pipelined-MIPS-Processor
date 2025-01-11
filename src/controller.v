
/* 
 * description: The module controller includes the connection of three control units: main control, PC control and ALU control.
 * inputs: clk, reset, opcode, M, N, V, stall	
 * output: RegSrc1, RegSrc2, RegDest, MEMWBdata, EXALUOp, PCSrc,
 * sign_extend_imm, MEMMemRd, MEMMemWr, MEMSignExtendMemData, EXALUSrc, EXMemDataIn, 
 * WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF
 */	 
 
`include "opcodes.v"
`include "fun_codes.v"

module controller(input        clk, reset,
				input  [3:0] opcode,
				input [2:0] functionCode,
                input  Z, stall,
				output [2:0] EXALUOp,
				output  [1:0] RegSrc2, PCSrc,
				output RegSrc1, RegDest, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, EXALUSrc, EXMemDataIn, 
    			output WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR
				);
	
	wire  DRegWr, DALUSrc, DMemDataIn, DMemRd, DSignExtendMemData, selectedRegWr, selectedMemWr;
	wire  EXSignExtendMemData;
	wire [2:0] DALUOp;
	wire DWBdata, EXWBdata, DNOOP, DCntInst, DALUInst, DBRANCH_OR_FOR; 
	
	mainControlUnit mcu(opcode, functionCode,
	RegSrc1, RegSrc2, RegDest, DWBdata,
	sign_extend_imm, DRegWr, DALUSrc, DMemRd, DMemWr, NOOP, CntInst, ALUInst, BRANCH_OR_FOR);	
	
	
	// WE NEED TO CHECK THE BUFFERS . 
	alucontrol ac(opcode, functionCode, DALUOp);
	
	pccontrol pc(opcode, Z, functionCode, PCSrc, PCsrcJType, RRSrc, killF); 
	
	mux2 #(2) ctrlSignalSelector({DRegWr, DMemWr}, 2'b00, stall, {selectedRegWr, selectedMemWr});
	
	// ID/EX Buffers
	flopr #(4) EX_IDEXBuffer(clk, reset, {DALUSrc, DALUOp}, {EXALUSrc, EXALUOp});
	flopr #(3) MEM_IDEXBuffer(clk, reset, {DMemRd, selectedMemWr, DWBdata}, {EXMemRd, EXMemWr, EXWBdata});
	flopr #(1) WB_IDEXBuffer(clk, reset, {selectedRegWr}, {EXRegWr});  
	
	
	// EX/MEM Buffers
	flopr #(3) MEM_EXMEMBuffer(clk, reset, {EXMemRd, EXMemWr, EXWBdata}, {MEMMemRd, MEMMemWr, MEMWBdata});
	flopr #(1) WB_EXMEMBuffer(clk, reset, {EXRegWr}, {MEMRegWr});
	
	// MEM/WB Buffers
	flopr #(1) WB_MEMWBBuffer(clk, reset, {MEMRegWr}, {WBRegWr});
				 
endmodule

/* 
 * description: this module is responsible for generating the control units responsible for the control of MUXes and sign extenders
 * throughout the datapath and for controlling the data memory.
 * inputs: op, mode	
 * output: RegSrc1, RegSrc2, RegDest, WBdata, sign_extend_imm,
 * RegWr, ALUsrc, MemDataIn, MemRd, MemWr, Sign_Extend_Data
 */
module mainControlUnit(input  [3:0] op, input [2:0]functionCode,
			   output       RegSrc1, 
			   output		[1:0] RegSrc2,
               output       RegDest, WBdata,
               output       sign_extend_imm, RegWr,
               output       ALUsrc,
               output 		MemRd, MemWr,
			   output 		NOOP, CntInst, ALUInst, BRANCH_OR_FOR);

  reg [13:0] controls;

  assign {RegSrc1, RegSrc2, RegDest,
           RegWr, ALUsrc, WBdata, sign_extend_imm , MemRd,
		  MemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst } = controls;
 
	  
  always @(*)														 
	
    casez({op, functionCode})
      {`RTYPE, `FunAND}: controls <= 14'b0000100x00x010; // AND
	  {`RTYPE, `FunADD}: controls <= 14'b0000100x00x010; // ADD
	  {`RTYPE, `FunSUB}: controls <= 14'b0000100x00x010; // SUB 
	  {`RTYPE, `FunSLL}: controls <= 14'b0000100x00x010; // SLL 
	  {`RTYPE, `FunSRL}: controls <= 14'b0000100x00x010; // SRL
	  
	  
	  {`ANDI,  3'bzzz}: controls <=  14'b1xx1110000x010; // ANDI
	  {`ADDI,  3'bzzz}: controls <=  14'b1xx1110100x010; // ADDI
	  {`LW,    3'bzzz}: controls <=  14'b1xx1111110x000; // LW
	  {`SW,    3'bzzz}: controls <=  14'b1xxx01x101x000; // SW
	  {`BEQ,   3'bzzz}: controls <=  14'b101x00x1000001; // BEQ
	  {`BNE,   3'bzzz}: controls <=  14'b101x00x1000001; // BNE
	  {`FOR,   3'bzzz}: controls <=  14'b01011x0x001001; // FOR  
	  
	  
	  {`JTYPE, `FunJMP}: controls <=  14'bxxx0xxx00x001; // JMP
	  {`JTYPE, `FunCALL}: controls <= 14'bxxx0xxx00x001; // CALL
	  {`JTYPE, `FunRET}: controls <=  14'bxxx0xxx00x001; // RET
	  
	  {`NOOP, 3'bzzz}: controls <= 14'bxxx0xxx00x001;
	  
	  
	  default: controls <= 14'b00000000000000; // invalid opcode
	  
      
    endcase
endmodule

/* 
 * description: this module acts as the control unit for the ALU; it generates the control signals required to select
 * the appropriate operation based on the opcode of the instruction.	
 * inputs: opcode
 * output: ALUop
 */
module alucontrol(input	[3:0] opcode, 
				  input [2:0] functionCode,
				output reg	[3:0] ALUop);

  always @(*)
    case(opcode)
		`RTYPE: begin
			// Actions for RTYPE opcode = 0000
			case(functionCode)	   
				`FunAND: ALUop <= 3'b000;
				`FunADD: ALUop <= 3'b001;
				`FunSUB: ALUop <= 3'b010;
				`FunSLL: ALUop <= 3'b011;
				`FunSRL: ALUop <= 3'b100;
			endcase
		end
		`JTYPE: begin
			case(functionCode)
				`FunJMP: ALUop <= 3'bxxx;
				`FunCALL: ALUop <= 3'bxxx;
				`FunRET: ALUop <= 3'bxxx;
			endcase
		end	
		`ANDI: ALUop <= 3'b000;
		`ADDI: ALUop <= 3'b001;
		`LW: ALUop <= 3'b001;
		`SW: ALUop <= 3'b001; 
		`FOR: ALUop <= 3'b101;
		`BEQ: ALUop <= 3'bxxx;
		`BNE: ALUop <= 3'bxxx;
    endcase
endmodule



/* 
 * description: this module acts as the control unit for the ALU; it generates the control signals required to select
 * the appropriate operation based on the opcode of the instruction.	
 * inputs: opcode
 * output: ALUop
 */
module pccontrol(input	[3:0] IDOpcode,
	input Z,
	input [2:0] IDFunction,
	output [1:0] PCsrc,
	output PCsrcJType,
	output RRSrc,
	output killF);
	
	reg branchTaken, forTaken, jump, ret;
	
	assign branchTaken = ((IDOpcode == `BEQ) && (Z == 1)) || ((IDOpcode == `BNE) && (Z == 0));	   
	assign forTaken = (IDOpcode == `FOR) && (Z == 0);
	
	assign jump = (IDOpcode == `JTYPE) && ((IDFunction == `FunJMP) || (IDFunction == `FunCALL)); 
	
	assign RRSrc = (IDOpcode == `JTYPE) && (IDFunction == `FunCALL);	
	assign ret = (IDOpcode == `JTYPE) && (IDFunction == `FunRET);
	
	assign PCsrcJType = jump ? 1'b0 : 1'b1;
	
	assign PCsrc = branchTaken ? 2'b01 : 
	(forTaken ? 2'b10 : 
	(jump || ret  ?	2'b11 : 2'b00)); 
	
	// stall if there is a branch or jump
	assign killF = (branchTaken) || ((IDOpcode == `JTYPE) && ((IDFunction == `FunJMP) || (IDFunction == `FunCALL) || (IDFunction == `FunRET))) || (forTaken);
							
endmodule

/*
In case the code above didn't work 
module pccontrol(
    input  [3:0] IDOpcode,
    input        Z,
    input  [2:0] IDFunction,
    output [1:0] PCsrc,
    output       PCsrcJType,
    output       RRSrc,
    output       killF
);

    wire branchTaken, jump, forTaken, ret;

    assign branchTaken = ((IDOpcode == `BEQ) && (Z == 1)) || 
                         ((IDOpcode == `BNE) && (Z == 0));
    
    assign forTaken = (IDOpcode == `FOR) && (Z == 0);
    
    assign jump = (IDOpcode == `JMP) || (IDOpcode == `CALL); 
    
    assign RRSrc = (IDOpcode == `CALL);    
    
    assign ret = (IDOpcode == `RET);
    
    assign PCsrcJType = jump ? 1'b0 : 1'b1;
    
    assign PCsrc = branchTaken ? 2'b01 : 
                   (forTaken ? 2'b10 : 
                   (jump || ret ? 2'b11 : 2'b00));
    
    // Stall if there is a branch or jump
    assign killF = branchTaken || (IDOpcode == `JMP) || 
                   (IDOpcode == `CALL) || (IDOpcode == `RET) || forTaken;

endmodule
*/