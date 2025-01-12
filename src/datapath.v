/* 
 * description: datapath module is a connection of the functional units, buffers, register file and MUXes.
 * inputs: clk, reset, RegSrc1, RegSrc2, RegDest, MEMWBdata, EXALUOp, PCSrc,
 * sign_extend_imm, MEMSignExtendMemData, EXALUSrc, EXMemDataIn,
 * WBRegWr, EXRegWr, MEMRegWr, DMemWr, EXMemRd, EXMemWr, killF, instrF	
 * output: Z, stall, instrAddrF, DataInM, ALUOutM, Opcode
 */	 
 
 
module datapath(input  clk, reset,
				input  [2:0] EXALUOp,
				input  [1:0] RegSrc2, PCSrc,
				input sign_extend_imm, RegSrc1, RegDest,MEMWBdata, EXALUSrc, EXMemDataIn,
    			input WBRegWr, EXRegWr, MEMRegWr, DMemWr, EXMemRd, EXMemWr, MEMMemRd, MEMMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR, 
				input  [15:0] DataOut,
				input  [15:0] instrF, 
                output	Z, stall,
                output [15:0] instrAddrF,
                output [15:0] DataInM, ALUOutM,
                output [3:0]  Opcode,
				output [2:0] Function
				);
										
	// wires in the fetch stage				
	wire [15:0] PCPlus1F, branchAddr, selectedInstruction;
	wire [15:0] JtypeMuxOutput,NoOpMuxOutput;
	wire [15:0] PCNext;
	// wires in the decode stage
	wire [1:0] ForwardBus1, ForwardBus2;
	wire ForwardME, StoreALUOpFw, StoreDataInFw;
	wire [2:0] RegSrc1D, RegSrc2D, RegDestD;
	wire [15:0] Bus1D, Bus2D, SubOut, instrAddrD, PCPlus1D, RROutput,RRSelectOutput, BranchOrForMuxOutput;
	wire [15:0] ForwardedData1, ForwardedData2, extendedImmediateD, extendedImmediateS, instrD;	
	wire [15:0] CntrlInstCount, AluInstCount, NumOfInstExec, CyclesCount, StallCyclesCount, LoadInstCount, StoreInstCount;	 
	wire [15:0] CntrlInstCountInput, AluInstCountInput, NumOfInstExecInput, CyclesCountInput, StallCyclesCountInput, LoadInstCountInput, StoreInstCountInput;
	
	// wires in the execute stage
	wire EXForwardME,EXStoreALUOpFw, EXStoreDataInFw;
	wire [2:0] RegDestE, storeALUOp1RegNo, storeMemInRegNo;
	wire [15:0] ALUOutE, ALUIn1, ALUIn2, SelectedDataIn, DataInE, extendedImmediateE, instrAddrE,
	Bus1E, Bus2E, PCPlus1E;
	
	// wires in the memory stage
	
	wire [15:0] PCPlus2M, extendedDataOut, WrittenDataM;
	wire [2:0] RegDestM;
	
	// wires in the writeback stage
	wire [15:0] WrittenDataW;
	wire [2:0] RegDestW;
	

	// hazard detection
	hazard    h(RegDestE, RegDestM, RegDestW, RegSrc1D, RegSrc2D, storeALUOp1RegNo, storeMemInRegNo,
	EXRegWr,WBRegWr, MEMRegWr, EXMemRd, DMemWr,EXMemWr, MEMMemRd, MEMMemWr,
	ForwardBus1, ForwardBus2, 
	stall, StoreALUOpFw, StoreDataInFw);
	
	// Fetch stage logic
	mux2 #(16) NoOpMux(PCPlus1D, PCNext, killF, NoOpMuxOutput);	 
	mux2 #(16) JTypeMux({instrAddrF[15:9], instrD[11:3]}, RROutput, PCsrcJType, JtypeMuxOutput);
	mux4 PCinput(NoOpMuxOutput,branchAddr, Bus2D, JtypeMuxOutput , PCSrc, PCNext);	 
	flopenr #(16) PC(clk, reset, ~stall, PCNext, instrAddrF);	// PC register
	adder       pcadd1(PCNext, 16'b0000000000000001, PCPlus1F); // adder to increment PC address
	mux2 #(16) fetchedInstructionSelector(instrF, 16'b0000000000000000, killF, selectedInstruction); //	instrF is the output of the IMEM  
	
	
	// IF/ID Buffers
	flopenr #(16) PCBufferF(clk, reset, ~stall, instrAddrF, instrAddrD);	// PC Buffer
	flopenr #(16) PCPlusBufferF(clk, reset, ~stall, PCPlus1F, PCPlus1D);	// PC+2 Buffer
	flopenr #(16) FetchedInstrBufferF(clk, reset, ~stall, selectedInstruction, instrD);	// Fetched Instruction Buffer	
	
	
	
	// Decode stage logic
	assign Opcode = instrD[15:12];
	assign Function = instrD[2:0];
	mux2 #(3) regSrc1Selector(instrD[8:6], instrD[11:9], RegSrc1, RegSrc1D);			
	mux4 #(3) regSrc2Selector(instrD[5:3], instrD[8:6], instrD[11:9], 3'bxxx , RegSrc2, RegSrc2D);
	mux2 #(3) regDestSelector(instrD[11:9], instrD[8:6], RegDest, RegDestD);  
	
	regfile rf(clk, WBRegWr, RegSrc1D, RegSrc2D, RegDestW, WrittenDataW, Bus1D, Bus2D);	
	
	mux4 #(16) ForwardBus1Selector(Bus1D, ALUOutE, WrittenDataM, WrittenDataW, ForwardBus1, ForwardedData1);
	mux4 #(16) ForwardBus2Selector(Bus2D, ALUOutE, WrittenDataM, WrittenDataW, ForwardBus2, ForwardedData2);
	
	
	signext #(6) itype_extender(instrD[5:0], sign_extend_imm, extendedImmediateD);
	
	
	adder branchAdder(extendedImmediateD, instrAddrD, branchAddr);	
	
	mux2 #(16) BranchOrForMux(ForwardedData2, 16'b0000000000000001, BRANCH_OR_FOR, BranchOrForMuxOutput);
	
	subtractor sb(ForwardedData1, BranchOrForMuxOutput, Z, SubOut);	 
	
	//RR implementation
	mux2 #(16) RRSelect(RROutput, instrAddrD,RRSrc, RRSelectOutput);
	flopr #(16)  CntrlInstCountBuffer(clk, reset, RRSelectOutput, RROutput);	
	
	//Special Purpose Regirsters  
	adder       CntrlInstCountAddr(CntrlInstCount, CntInst, CntrlInstCountInput); 
	flopr #(16)  CntrlInstCountBuffer(clk, reset, CntrlInstCountInput, CntrlInstCount);	 
	
	adder       AluInstCountAddr(AluInstCount, ALUInst, AluInstCountInput); 
	flopr #(16)  AluInstCountBuffer(clk, reset, AluInstCountInput, AluInstCount);
	
	adder       CyclesCountAddr(CyclesCount, 16'b0000000000000001, CyclesCountInput); 
	flopr #(16)  CyclesCountBuffer(clk, reset, CyclesCountInput, CyclesCount);
	
	adder       NumOfInstExecAddr(NumOfInstExec, ~stall & ~killF , NumOfInstExecInput); 
	flopr #(16)  NumOfInstExecBuffer(clk, reset, NumOfInstExecInput, NumOfInstExec);
	
	adder       StallCyclesCountAddr(StallCyclesCount, NOOP | stall, StallCyclesCountInput); 
	flopr #(16)  StallCyclesCountBuffer(clk, reset, StallCyclesCountInput, StallCyclesCount);
	
	adder       LoadInstCountAddr(LoadInstCount, MEMMemRd, LoadInstCountInput); 
	flopr #(16)  LoadInstCountBuffer(clk, reset, LoadInstCountInput, LoadInstCount);	 
	
	adder       StoreInstCountAddr(StoreInstCount, MEMMemWr, StoreInstCountInput); 
	flopr #(16)  StoreInstCountBuffer(clk, reset, StoreInstCountInput, StoreInstCount);
	
	// ID/EX Buffers
	flopr #(16) ImmBufferE(clk, reset, extendedImmediateD, extendedImmediateE);
	flopr #(16) PCBufferE(clk, reset, instrAddrD, instrAddrE);
	flopr #(16) Bus1BufferE(clk, reset, ForwardedData1, Bus1E);
	flopr #(16) Bus2BufferE(clk, reset, ForwardedData2, Bus2E);
	flopr #(3) RdE(clk, reset, RegDestD, RegDestE);
	flopr #(16) PCPlusBufferE(clk, reset, PCPlus1D, PCPlus1E);
	flopr #(1)  StoreDataInFwBuffer(clk, reset, StoreDataInFw, EXStoreDataInFw);
	flopr #(1)  StoreALUOpFwBuffer(clk, reset, StoreALUOpFw, EXStoreALUOpFw);
	flopr #(3) StoreALUOP1RegNoBuffer(clk, reset, instrD[11:9], storeALUOp1RegNo);
	flopr #(3) StoreMemInRegNoBuffer(clk, reset, instrD[8:6], storeMemInRegNo);
	
	// Execute Stage Logic
	mux2 #(16) ALUInput1Selector(Bus1E, DataOut, EXStoreALUOpFw, ALUIn1); 
	mux2 #(16) ALUInput2Selector(Bus2E, extendedImmediateE, EXALUSrc, ALUIn2);
	alu ALU(ALUIn1, ALUIn2, EXALUOp, ALUOutE);
	mux2 #(16) DataInSelector(Bus2E, DataOut, EXStoreDataInFw, DataInE); 
	
	// EX/MEM Buffers  
	flopr #(16) ALUOutBuffer(clk, reset, ALUOutE, ALUOutM);
	flopr #(16)	DataInBuffer(clk, reset, DataInE, DataInM);
	flopr #(3)	RdMBuffer(clk, reset, RegDestE, RegDestM);
	
	
	// Memory Stage Logic
	
	mux2 #(16) WBDataSelector(ALUOutM, DataOut,  MEMWBdata, WrittenDataM);
	
	// MEM/WB Buffers
	
	flopr #(16) WBDataBuffer(clk, reset, WrittenDataM, WrittenDataW);
	flopr #(3) RdWBuffer(clk, reset, RegDestM, RegDestW);	  

endmodule

/* 
 * description: this module acts as the control unit for the ALU; it generates the control signals required to select
 * the appropriate operation based on the opcode of the instruction.	
 * inputs: opcode
 * output: ALUop
 */		
 // new definition of the hazard unit module should be also handled when instantaiting it again in the datapath module . 
 // we have added inputs that may have to be added later in the data path module which are the reg nos used in the lw sw hazard . 
module hazard(input [2:0] Rd2, Rd3, RD4, Rs1, Rs2,storeALUOp1RegNo, storeMemInRegNo,
			  input ExRegWr,WBRegWr, MemRegWr, ExMemRd, DMemWr,EXMemWr, MEMMemRd, MEMMemWr,
              output reg [1:0] ForwardBus1, ForwardBus2,
              output reg stall, StoreALUOpFw, StoreDataInFw);

 	always @(*)
		begin
    		
			ForwardBus1 = 2'b00; ForwardBus2 = 2'b00;
			StoreALUOpFw = 1'b0; 
			StoreDataInFw = 1'b0;
      		stall = 1'b0;
		  
        	if (Rs1 == Rd2 & ExRegWr)
				ForwardBus1 = 2'b01 ;
			
        	else if (Rs1 == Rd3 & MemRegWr)
				ForwardBus1 = 2'b10 ;
			else if (Rs1 == RD4 & WBRegWr)
				ForwardBus1 = 2'b11 ;
			
      		
		  
        	if (Rs2 == Rd2 & ExRegWr)
				ForwardBus2 = 2'b01 ;
        	else if (Rs2 == Rd3 & MemRegWr)
				ForwardBus2 = 2'b10 ;
			else if (Rs2 == RD4 & WBRegWr)
				ForwardBus2 = 2'b11 ;
			
			
			if (storeALUOp1RegNo == Rd3 & EXMemWr & MEMMemRd)
				StoreALUOpFw = 1'b1;
			
			if(storeMemInRegNo == Rd3 & EXMemWr & MEMMemRd)
				StoreDataInFw = 1'b1;
			
			if ((ExMemRd & ((Rs1 == Rd3 & MemRegWr)) | ((Rs2 == Rd3 & MemRegWr) ) & DMemWr))
				stall = 1'b1 ;
			
				
		end

endmodule