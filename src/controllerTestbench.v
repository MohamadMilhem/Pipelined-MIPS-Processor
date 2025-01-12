module controllerTestbench();

    // Testbench signals
    reg clk;
    reg reset;
    reg [3:0] opcode;
    reg [2:0] functionCode;
    reg Z;
    reg stall;
    
    wire [2:0] EXALUOp;
    wire [1:0] RegSrc2, PCSrc;
    wire RegSrc1, RegDest, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, EXALUSrc, EXMemDataIn, 
         WBRegWr, EXRegWr, MEMRegWr, EXMemRd, EXMemWr, DMemWr, killF, PCsrcJType, RRSrc, NOOP, CntInst, ALUInst, BRANCH_OR_FOR;

    // Instantiate the controller module
    controller uut (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .functionCode(functionCode),
        .Z(Z),
        .stall(stall),
        .EXALUOp(EXALUOp),
        .RegSrc2(RegSrc2),
        .PCSrc(PCSrc),
        .RegSrc1(RegSrc1),
        .RegDest(RegDest),
        .MEMWBdata(MEMWBdata),
        .sign_extend_imm(sign_extend_imm),
        .MEMMemRd(MEMMemRd),
        .MEMMemWr(MEMMemWr),
        .EXALUSrc(EXALUSrc),
        .EXMemDataIn(EXMemDataIn),
        .WBRegWr(WBRegWr),
        .EXRegWr(EXRegWr),
        .MEMRegWr(MEMRegWr),
        .EXMemRd(EXMemRd),
        .EXMemWr(EXMemWr),
        .DMemWr(DMemWr),
        .killF(killF),
        .PCsrcJType(PCsrcJType),
        .RRSrc(RRSrc),
        .NOOP(NOOP),
        .CntInst(CntInst),
        .ALUInst(ALUInst),
        .BRANCH_OR_FOR(BRANCH_OR_FOR)
    );

    // Clock generation
    always begin
        clk = 0; #5; clk = 1; #5;
    end

    // Apply test vectors
    initial begin
        // Initial reset
        reset = 1;
        opcode = 4'b0000;
        functionCode = 3'b000;
        Z = 0;
        stall = 0;
        #10;
        reset = 0;

        // Test case 1: AND (0000 000)
        #10;
        opcode = 4'b0000;
        functionCode = 3'b000;
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 1 - AND: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 2: ADD (0000 001)
        #10;
        opcode = 4'b0000;
        functionCode = 3'b001;
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 2 - ADD: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 3: SUB (0000 010)
        #10;
        opcode = 4'b0000;
        functionCode = 3'b010;
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 3 - SUB: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 4: SLL (0000 011)
        #10;
        opcode = 4'b0000;
        functionCode = 3'b011;
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 4 - SLL: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 5: SRL (0000 100)
        #10;
        opcode = 4'b0000;
        functionCode = 3'b100;
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 5 - SRL: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 6: ANDI (0010 NA)
        #10;
        opcode = 4'b0010;
        functionCode = 3'b000; // NA (no function code for ANDI)
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 6 - ANDI: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 7: ADDI (0011 NA)
        #10;
        opcode = 4'b0011;
        functionCode = 3'b000; // NA (no function code for ADDI)
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 7 - ADDI: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 8: LW (0100 NA)
        #10;
        opcode = 4'b0100;
        functionCode = 3'b000; // NA (no function code for LW)
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 8 - LW: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 9: SW (0101 NA)
        #10;
        opcode = 4'b0101;
        functionCode = 3'b000; // NA (no function code for SW)
        Z = 0;
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 9 - SW: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);

        // Test case 10: BEQ (0110 NA)
        #10;
        opcode = 4'b0110;
        functionCode = 3'b000; // NA (no function code for BEQ)
        Z = 1; // Assuming Z flag is active for BEQ
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 10 - BEQ: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
		
		// Test case 11: BNE (0111 NA)
        #10;
        opcode = 4'b0111;
        functionCode = 3'b000; // NA (no function code for BNE)
        Z = 0; // Assuming Z flag is inactive for BNE
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 11 - BNE: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
		
		// Test case 12: FOR (1000 NA)
        #10;
        opcode = 4'b1000;
        functionCode = 3'b000; // NA (no function code FOR)
        Z = 1; // Assuming Z flag is active for BEQ
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 12 - FOR: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
		
		// Test case 13: JMP (0001 000)
        #10;
        opcode = 4'b0001;
        functionCode = 3'b000; 
        Z = 0; // Assuming Z flag is inactive for JMP
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 13 - JMP: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
		
		// Test case 14: CALL (0001 001)
        #10;
        opcode = 4'b0001;
        functionCode = 3'b001;
        Z = 1; // Assuming Z flag is active for BEQ
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 14 - CALL: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
	   
		// Test case 15: RET (0001 010)
        #10;
        opcode = 4'b0001;
        functionCode = 3'b010; 
        Z = 1; // Assuming Z flag is active for BEQ
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 15 - RET: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
		
		// Test case 16: NOOP (1111 NA)
        #10;
        opcode = 4'b1111;
        functionCode = 3'b000; // NA (no function code for NOOP)
        Z = 1; // Assuming Z flag is active for NOOP
        stall = 0;
		#10 // wait for 1 cycle.
		#10 // wait for 2 cycles.
		#10	// wait for 3 cycles.
		#10 // wait for 4 cycles.
		#10 // wait for 5 cycles.
        $display("Test case 16 - NOOP: RegSrc1=%b, RegSrc2=%b, RegDest=%b, RegWr=%b, ALUSrc=%b, WBData=%b, SignExtend=%b, MemRd=%b, MemWr=%b, Branch_OR_For=%b, NoOp=%b, ALUInst=%b, CntInst=%b", 
            RegSrc1, RegSrc2, RegDest, WBRegWr, EXALUSrc, MEMWBdata, sign_extend_imm, MEMMemRd, MEMMemWr, BRANCH_OR_FOR, NOOP, ALUInst, CntInst);
        // End simulation
        #100;
        $finish;
    end

endmodule
