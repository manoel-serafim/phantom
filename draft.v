`timescale 1 ns / 1 ps
// `default_nettype none
// `define DEBUGNETS
// `define DEBUGREGS
// `define DEBUGASM
// `define DEBUG


module CPUIO(

	/* Clock Signal */
  input wire CLOCK_50,
  input [3:0] KEY,
  output [17:0]LEDR,
  output reg [8:0] LEDG
  
  

);

  wire CLOCK;
  wire [31:0] DATA_OUT_1;
  wire [31:0] DATA_OUT_2;
  
  /* Wires to connect instruction memory to CPU */
  wire [31:0] instructionPC;
  wire [31:0] instructionOut;

  /* wires to connect registers to CPU */
  wire [4:0] ADDR_REG_1;
  wire [4:0] ADDR_REG_2;
  wire [4:0] ADDR_WRITE_REG;
  wire [31:0] WRITE_DATA;
  

  /* Wires to connect Data Memory to CPU */
  wire [31:0] READ_DATA_MEM;
  wire [31:0] ALU_OUT;
  wire CONTROL_LEDWRITE;

   /* Wires to connect CPU Control Lines to Memories */
  wire CONTROL_REG2LOC;
  wire CONTROL_REGWRITE;
  wire CONTROL_MEMREAD;
  wire CONTROL_MEMWRITE;
  wire CONTROL_NBRANCH;
  wire CONTROL_ZBRANCH;
  wire RESET_REG;
  

	
  DeBounce DB(CLOCK_50, 1'b1 , KEY[0], CLOCK);

  /* Instruction Memory Module */
  INSTmem mem1
  (
    instructionPC,
    instructionOut
  );

  /* Registers Module */
  REG mem2
  (
    ADDR_REG_1,
    ADDR_REG_2,
    ADDR_WRITE_REG,
    WRITE_DATA,
	 RESET_REG,
    CONTROL_REGWRITE,
    DATA_OUT_1,
    DATA_OUT_2
  );

  /* Data Memory Module */
  MEM mem3
  (
    ALU_OUT,
    DATA_OUT_2,
    CONTROL_MEMREAD,
    CONTROL_MEMWRITE,
    READ_DATA_MEM
  );

  /* CPU Module */
  CPU core
  (
    .CLOCK(CLOCK),
    .INSTRUCTION(instructionOut),
    .PC(instructionPC),
    .CONTROL_REG2LOC(CONTROL_REG2LOC),
    .CONTROL_REGWRITE(CONTROL_REGWRITE),
    .CONTROL_MEMREAD(CONTROL_MEMREAD),
    .CONTROL_MEMWRITE(CONTROL_MEMWRITE),
    .CONTROL_NBRANCH(CONTROL_NBRANCH),
	 .CONTROL_ZBRANCH(CONTROL_ZBRANCH),
    .ADDR_REG_1(ADDR_REG_1),
    .ADDR_REG_2(ADDR_REG_2),
    .ADDR_WRITE_REG(ADDR_WRITE_REG),
    .REG_DATA_1(DATA_OUT_1),
    .REG_DATA_2(DATA_OUT_2),
    .ALU_OUT(ALU_OUT),
    .READ_DATA_MEM(READ_DATA_MEM),
    .WRITE_REG_DATA(WRITE_DATA),
	 .CONTROL_LEDWRITE(CONTROL_LEDWRITE)
  );
  
  LEDOUT leds
  (
	 DATA_OUT_1,
    DATA_OUT_2,
	 CLOCK,
	 CONTROL_LEDWRITE,
	 LEDR
  );
  
  always @(posedge CLOCK)begin
    LEDG[8:0]= instructionPC[8:0];
  end


endmodule



module CPU
(
  input clock_signal,
);

  wire [31:0] f_instruction;
  wire [31:0] f_phantom_instruction;
  wire [31:0] d_instruction;
  wire [31:0] d_phantom_instruction;

  FETCH_DECODE stage_fetch_decode(clock_signal, f_instruction, f_phantom_instruction, d_instruction, d_phantom_instruction);

  REG registers(clock_signal,)
 


  /* Multiplexer for the Program Counter */
  PCmux PCm(NEXT_PC, SHIFTED_PC, CONTROL_JUMP, BRANCH_PC);

  /* Multiplexer before the Register */
  REGmux mux2(INST_REG_ADDR1, INST_REG_ADDR2, CONTROL_REG2LOC, ADDR_REG_2);

  /* Multiplexer before the ALU */
  ALUmux mux3(REG_DATA_2, EXT_IMMED, CONTROL_ALUSRC, ALU_IN_B);

  /* Multiplexer after the Data memory */
  MEMmux mux4(READ_DATA_MEM, ALU_OUT, CONTROL_MEM2REG, WRITE_REG_DATA);

  /* Sign Extention Module */
  EXT mod1(INSTRUCTION,CLOCK, EXT_IMMED);
  
  /* ALU Result between the Registers and the Data Memory */
  ALU aluResult(REG_DATA_1, ALU_IN_B, ALU_CONTROL, ALU_OUT, ALU_ZFLAG, ALU_NFLAG);

  /* An ALU module to calulcate the next sequential PC */
  ALU adderNextPC(PC, 32'b00000000000000000000000000000001, 4'b0010, NEXT_PC, NEXT_PC_ZFLAG, NEXT_PC_NFLAG);

  /* An ALU module to calulcate a shifted PC */// IN FUTURE CAN CHANGE A=PC
  ALU adderShiftPC(PC, EXT_IMMED, 4'b0111, SHIFTED_PC, SHIFTED_PC_ZERO, SHIFTED_PC_NFLAG);

  


  /* Initialize when the CPU is first run */
  initial begin
    PC = 0;
    CONTROL_REG2LOC = 1'bz;
    CONTROL_MEM2REG = 1'bz;
    CONTROL_REGWRITE = 1'bz;
    CONTROL_MEMREAD = 1'bz;
    CONTROL_MEMWRITE = 1'bz;
    CONTROL_ALUSRC = 1'bz;
    CONTROL_ZBRANCH = 1'b0;
	 CONTROL_NBRANCH = 1'b0;
    CONTROL_UNCON_BRANCH = 1'b0;
	 CONTROL_LEDWRITE = 1'b0;
    BRANCH_ON_ZFLAG = ALU_ZFLAG & CONTROL_ZBRANCH;
	 BRANCH_ON_NFLAG = ALU_NFLAG & CONTROL_NBRANCH; 
    CONTROL_JUMP = CONTROL_UNCON_BRANCH | BRANCH_ON_ZFLAG | BRANCH_ON_NFLAG;
  end

  /* Parse and set the CPU's Control bits */
  always @(posedge CLOCK) begin //or INSTRUCTION

			 //Determine whether to branch
			 BRANCH_ON_ZFLAG = ALU_ZFLAG & CONTROL_ZBRANCH;
			 BRANCH_ON_NFLAG = ALU_NFLAG & CONTROL_NBRANCH;
			 CONTROL_JUMP = CONTROL_UNCON_BRANCH | BRANCH_ON_ZFLAG | BRANCH_ON_NFLAG;

			
			 PC = #1 BRANCH_PC;
			 

			 // Parse the incoming instruction for a given PC
			 INST_REG_ADDR1 = INSTRUCTION[20:16];
			 INST_REG_ADDR2 = INSTRUCTION[4:0];
			 ADDR_REG_1 = INSTRUCTION[9:5];
			 ADDR_WRITE_REG = INSTRUCTION[4:0];
			 
			 

			 //Control Unit:
			 case(INSTRUCTION[31:29])
				3'b000:begin // D-type Instruction
				  CONTROL_ZBRANCH = 1'b0;
				  CONTROL_NBRANCH = 1'b0;
				  CONTROL_UNCON_BRANCH = 1'b0;
				  CONTROL_LEDWRITE = 1'b0;
				  
				  //parse based on opcode:
				  case(INSTRUCTION[28:21])
					 8'b00000000:begin //LDR
						CONTROL_REG2LOC = 1'bx;  //
						CONTROL_MEM2REG = 1'b1;  // Content to load to Rd[0-4]
						CONTROL_REGWRITE = 1'b1;
						CONTROL_MEMREAD = 1'b1;
						CONTROL_MEMWRITE = 1'b0;
						CONTROL_ALUSRC = 1'b1; //addr comes from addr in immediate + data in reg
						ALU_CONTROL = 4'b0010; // sum data in reg to base addr that was given
					 end

					 8'b00000001:begin //STR
						CONTROL_REG2LOC = 1'b1; //Content to load from Rt[0-4]
						CONTROL_MEM2REG = 1'bx; // no
						CONTROL_REGWRITE = 1'b0; // no
						CONTROL_MEMREAD = 1'b0; //no
						CONTROL_MEMWRITE = 1'b1; //yes
						CONTROL_ALUSRC = 1'b1; // sum the address + content of r1
						ALU_CONTROL = 4'b0010; //sum
					 end

				  endcase
				end
				3'b001:begin //I-type Instruction
				  CONTROL_ZBRANCH = 1'b0;
				  CONTROL_NBRANCH = 1'b0;
				  CONTROL_UNCON_BRANCH = 1'b0;
				  CONTROL_LEDWRITE = 1'b0;
				  
				  case(INSTRUCTION[28:22])
					 7'b0000000:begin //MOVI
						CONTROL_REG2LOC = 1'bx; //do not matter
						CONTROL_MEM2REG = 1'b0; // alu output
						CONTROL_REGWRITE = 1'b1; //yes
						CONTROL_MEMREAD = 1'b0; // no
						CONTROL_MEMWRITE = 1'b0; //no
						CONTROL_ALUSRC = 1'b1; // get immediate
						ALU_CONTROL = 4'b0111; //want to just pass the value of the immediate
						end

					 7'b0000001:begin //ADDI
						CONTROL_REG2LOC = 1'bx; // do not matter
						CONTROL_MEM2REG = 1'b0; // alu output para Rd[0-4]
						CONTROL_REGWRITE = 1'b1; //yes Rd[0-4]
						CONTROL_MEMREAD = 1'b0; // no
						CONTROL_MEMWRITE = 1'b0; //no
						CONTROL_ALUSRC = 1'b1; // get immediate
						ALU_CONTROL = 4'b0010; //sum Rn[5-9] to immediate and store
						end
				  endcase
				end

				3'b010:begin //R-type Instruction
				  CONTROL_ZBRANCH = 1'b0;
				  CONTROL_NBRANCH = 1'b0;
				  CONTROL_UNCON_BRANCH = 1'b0;
				  CONTROL_LEDWRITE = 1'b0;
				  
				  case(INSTRUCTION[28:21])
				  
					 8'b00000000:begin //ADD
						CONTROL_REG2LOC = 1'b0; // get from Rm[20-16]
						CONTROL_MEM2REG = 1'b0; // alu to Rd[0-3]
						CONTROL_REGWRITE = 1'b1; //yes Rd[0-3]
						CONTROL_MEMREAD = 1'b0; //no
						CONTROL_MEMWRITE = 1'b0; //no
						CONTROL_ALUSRC = 1'b0; //get from r2 data Rm[16-20] 
						ALU_CONTROL = 4'b0010; //sum Rn[5-9] + Rm[20-16] them
					 end
					 8'b00000001:begin //SUB
						CONTROL_REG2LOC = 1'b0; // get from Rm[20-16]
						CONTROL_MEM2REG = 1'b0; // alu to Rd[0-4]
						CONTROL_REGWRITE = 1'b1; //yes Rd[0-4]
						CONTROL_MEMREAD = 1'b0; //no
						CONTROL_MEMWRITE = 1'b0; //no
						CONTROL_ALUSRC = 1'b0; //get from r2 data Rm[16-20] 
						ALU_CONTROL = 4'b0110; //sub Rn[5-9] - Rm[20-16] them
					 end
					 
					 8'b00000010:begin //MOV
						CONTROL_REG2LOC = 1'b0; // get from Rm[20-16]
						CONTROL_MEM2REG = 1'b0; // alu to Rd[0-4]
						CONTROL_REGWRITE = 1'b1; //yes Rd[0-4]
						CONTROL_MEMREAD = 1'b0; //no
						CONTROL_MEMWRITE = 1'b0; //no
						CONTROL_ALUSRC = 1'b0; //get from r2 data Rm[16-20] 
						ALU_CONTROL = 4'b0111; //buff
					 end

				  endcase
				
				end
				3'b011:begin // CB-type Instruction
				  CONTROL_UNCON_BRANCH = 1'b0;
				  CONTROL_LEDWRITE = 1'b0;
				  
				  //parse based on opcode:
				  case(INSTRUCTION[28:24])
					 5'b00000:begin //BEQ
						CONTROL_REG2LOC = 1'b1;//Rt[0-4]
						CONTROL_MEM2REG = 1'b0;// no
						CONTROL_REGWRITE = 1'b0; // no
						CONTROL_MEMREAD = 1'b0; // no
						CONTROL_MEMWRITE = 1'b0; // no
						CONTROL_ALUSRC = 1'b0; //GET REGVAL
						ALU_CONTROL = 4'b0110; //sub para gerar flag
						CONTROL_ZBRANCH = 1'b1; //
						CONTROL_NBRANCH = 1'b0;
						
					 end

					 5'b00001:begin //BLT
						CONTROL_REG2LOC = 1'b1;  // 
						CONTROL_MEM2REG = 1'b0;  // 
						CONTROL_REGWRITE = 1'b0; //
						CONTROL_MEMREAD = 1'b0;  //
						CONTROL_MEMWRITE = 1'b0; //
						CONTROL_ALUSRC = 1'b0;   //
						ALU_CONTROL = 4'b0110;   //
						CONTROL_ZBRANCH = 1'b0; //
						CONTROL_NBRANCH = 1'b1;
						
					 end

				  endcase
				end

				3'b100:begin // B-type Instruction
				  //parse based on opcode:
				  case(INSTRUCTION[28:26])
					 3'b000:begin //BI
						CONTROL_REG2LOC = 1'b0;//do not really matter
						CONTROL_MEM2REG = 1'b0;// not matter
						CONTROL_REGWRITE = 1'b0;// no
						CONTROL_MEMREAD = 1'b0; // no
						CONTROL_MEMWRITE = 1'b0;//no
						CONTROL_ALUSRC = 1'b0;//no
						ALU_CONTROL = 4'b0111; // buffer
						CONTROL_ZBRANCH = 1'b0; //
						CONTROL_NBRANCH = 1'b0;
						CONTROL_UNCON_BRANCH = 1'b1;
						CONTROL_LEDWRITE = 1'b0;// PC = SHIFTED_PC
						//halt == movi 0 to rzero [1111]
						
					 end
				  endcase
				end
				
				3'b111:begin // LED-type Instruction
				  //parse based on opcode:
				  case(INSTRUCTION[28:26])
					 3'b111:begin //BI
						CONTROL_REG2LOC = 1'b1;//IN[0-4
						CONTROL_MEM2REG = 1'b0;// no
						CONTROL_REGWRITE = 1'b0;// no
						CONTROL_MEMREAD = 1'b0; // no
						CONTROL_MEMWRITE = 1'b0;//no
						CONTROL_ALUSRC = 1'b0;//no
						ALU_CONTROL = 4'b0111; // buffer
						CONTROL_ZBRANCH = 1'b0; //
						CONTROL_NBRANCH = 1'b0;
						CONTROL_UNCON_BRANCH = 1'b0; //
						CONTROL_LEDWRITE = 1'b1;
						
					 end
				  endcase
				 end

			 endcase
			 
			 //Determine whether to branch
			 BRANCH_ON_ZFLAG = ALU_ZFLAG & CONTROL_ZBRANCH;
			 BRANCH_ON_NFLAG = ALU_NFLAG & CONTROL_NBRANCH;
			 CONTROL_JUMP = CONTROL_UNCON_BRANCH | BRANCH_ON_ZFLAG | BRANCH_ON_NFLAG; 

			
			 PC <= #1 BRANCH_PC;

		
	
		
		
    
  end
  
  
endmodule












//muxes
module PCmux
(
  input [31:0] pcInput,
  input [31:0] shiftInput,
  input CONTROL_JUMP,
  output reg [31:0] pcOut
);

  always @(pcInput, shiftInput, CONTROL_JUMP, pcOut) begin
    if (CONTROL_JUMP == 0) begin
      pcOut = pcInput;
    end

    else begin
      pcOut = shiftInput;
    end
  end
endmodule

module REGmux//No more need for this guy in a 16 bit cpu
(
  input [4:0] addrRm,
  input [4:0] addrRn,
  input CONTROL_REG2LOC,
  output reg [4:0] muxOutput
);

  always @(addrRm, addrRn, CONTROL_REG2LOC) begin

    if (CONTROL_REG2LOC == 0) begin
      muxOutput = addrRm;
    end

    else begin
      muxOutput = addrRn;
    end
  end
endmodule

module MUX_alu_source
(
  input CLOCK,
  input [31:0] register,
  input [31:0] immediate,
  input CONTROL_ALUSRC,
  output reg [31:0] out
);

  always @(posedge CLOCK) begin

    if (CONTROL_ALUSRC == 0) begin
      out = register;
    end

    else begin
      out = immediate;
    end
  end
endmodule


module MUX_write_back
(
  input [31:0] data,
  input [31:0] alu_out,
  input CONTROL_MEM2REG,
  output reg [31:0] out
);

  always @(data, alu_out, CONTROL_MEM2REG, out) begin
    if (CONTROL_MEM2REG == 0) begin
      out = alu_out;
    end

    else begin
      out = data;
    end
  end
endmodule


//modules
module EXT
(
  input [31:0] inputInstruction,
  input CLOCK,
  output reg [31:0] outImmediate
);

    always @(posedge CLOCK) begin

      if (inputInstruction[31:29]== 3'b100) begin // B
        outImmediate[25:0] = inputInstruction[25:0];
        outImmediate[31:26] = {32{outImmediate[25]}};

      end else if (inputInstruction[31:29] == 3'b011) begin // CB
        outImmediate[13:0] = inputInstruction[23:10];
        outImmediate[31:14] = {32{outImmediate[13]}};

      end else if (inputInstruction[31:29] == 3'b000) begin // D Type
        outImmediate[10:0] = inputInstruction[20:10]; //address
        outImmediate[31:11] = {32{outImmediate[10]}};

      end else if (inputInstruction[31:29] == 3'b001) begin // I Type
        outImmediate[11:0] = inputInstruction[21:10]; //immediate
        outImmediate[31:12] = {32{outImmediate[11]}};
      end

    end
endmodule


module REG
(
  input [4:0] read_addr1,
  input [4:0] read_addr2,
  input [4:0] write_addr,
  input [31:0] write_data,
  input reset,
  input CONTROL_REGWRITE,
  output reg [31:0] data1,
  output reg [31:0] data2
);

  reg [31:0] Data[31:0];
  integer counter;

  always @(read_addr1, read_addr2, write_addr, write_data, CONTROL_REGWRITE, reset) begin
	  if (reset == 1) begin
		  for (counter = 0; counter < 31; counter = counter + 1) begin
			  Data[counter] = counter;
		  end
		  Data[counter] = 32'h0000;
	  end
	  
	 Data[31] = 32'h0000;
	  
    data1 = Data[read_addr1];
    data2 = Data[read_addr2];
	 

    if (CONTROL_REGWRITE == 1) begin
      Data[write_addr] = write_data;
    end
  end
endmodule



module MEM
(
  input [31:0] instruction_addr,
  input [10:0] addr, //possible to reference 2^11 addresses
  input [31:0] in_data,
  input CONTROL_MemRead,
  input CONTROL_MemWrite,
  output reg [31:0] cpu_instruction,
  output reg [31:0] out_data
);

  reg [31:0] Data[31:0]; // Just 32 for faster compilation
  initial begin
  //FIBONACCI CHECKER:
	 
	 //LABEL START
	 //LDR R7, INPUT
	 
	 //MOVI R0,0
	 Data[0] = 32'b00100000000000000000000000000000;
	 //MOVI R1,1
	 Data[1] = 32'b00100000000000000000010000000001;
	 
	 
	 //LABEL LOOP
	 //OUT(R0, R1)
	 //Data[3]
	 //"DISPLAY"
	 Data[2] = 32'b11111100000000000000000000100000;
	 Data[3] = 32'b11111100000000000000000000100000;
	 
	 //ADD R2, R1,R0
	 Data[4] = 32'b01000000000000010000000000000010;
	 //MOV R0, R1
	 Data[5] = 32'b01000000010000010000001111100000;
	 //MOV R1, R2
	 Data[6] = 32'b01000000010000100000001111100001;
	 
	 //B LOOP
	 Data[7] = 32'b10000000000000000000000000000010;
	 Data[8] = 32'b00100000000000000000000000011111;//HALT
	 
	 
  end
  
    always @(addr, in_data, CONTROL_MemRead, CONTROL_MemWrite, instruction_addr) begin
      if (CONTROL_MemWrite == 1) begin
        Data[addr] = in_data;
      end

      if (CONTROL_MemRead == 1) begin
        out_data = Data[addr];
      end
      cpu_instruction = Data[instruction_addr];
    end
endmodule


module LEDOUT
(
	input [31:0] data_g,
	input [31:0] data_r,
	input CLOCK,
	input CONTROL_LEDWRITE,
	output reg [0:17] LED
);

	always @(posedge CLOCK) begin
		
		if(CONTROL_LEDWRITE == 1) begin
		  LED[0:8] = data_g[8:0];
        LED[9:17] = data_r[8:0];
		end
	
	end

endmodule





`timescale 1 ns / 100 ps
module  DeBounce 
	(
	input 			clk, n_reset, button_in,				// inputs
	output reg 	DB_out													// output
	);
//// ---------------- internal constants --------------
	parameter N = 11 ;		// (2^ (21-1) )/ 38 MHz = 32 ms debounce time
////---------------- internal variables ---------------
	reg  [N-1 : 0]	q_reg;							// timing regs
	reg  [N-1 : 0]	q_next;
	reg DFF1, DFF2;									// input flip-flops
	wire q_add;											// control flags
	wire q_reset;
//// ------------------------------------------------------

////contenious assignment for counter control
	assign q_reset = (DFF1  ^ DFF2);		// xor input flip flops to look for level chage to reset counter
	assign  q_add = ~(q_reg[N-1]);			// add to counter when q_reg msb is equal to 0
	
//// combo counter to manage q_next	
	always @ ( q_reset, q_add, q_reg)
		begin
			case( {q_reset , q_add})
				2'b00 :
						q_next <= q_reg;
				2'b01 :
						q_next <= q_reg + 1;
				default :
						q_next <= { N {1'b0} };
			endcase 	
		end
	
//// Flip flop inputs and q_reg update
	always @ ( posedge clk )
		begin
			if(n_reset ==  1'b0)
				begin
					DFF1 <= 1'b0;
					DFF2 <= 1'b0;
					q_reg <= { N {1'b0} };
				end
			else
				begin
					DFF1 <= button_in;
					DFF2 <= DFF1;
					q_reg <= q_next;
				end
		end
	
//// counter control
	always @ ( posedge clk )
		begin
			if(q_reg[N-1] == 1'b1)
					DB_out <= DFF2;
			else
					DB_out <= DB_out;
		end

	endmodule

