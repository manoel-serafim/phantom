
/*--[Instruction Execution Stage Modules]--*/
/*
  [Arithmethic Logic Unit]
  @param input:: clock_signal = clock signal
  @param input:: a = data from the a register or the output of the data memory stage or the write_back stage
  @param input:: b = data from the b register or the immediate or the output of the data memory stage or the write_back stage
  @param input:: alu_ctrl = control signal to define the operation to be done
  @param input:: rotate_shift_ctrl = 2 bit value control if will do a rotation or a shift in the output and if it will be done to the right or left
  @param input:: amount = five bit value (up to 32) amount to shit or rotate

  @param output:: alu_out = result from the operation between a and b specified by alu_ctrl
  @param output:: cpsr = current program status register

*/
module ALU
(
  input clock_signal,
  input [31:0] a,
  input [31:0] b,
  input [3:0] alu_ctrl,
  input [1:0] rotate_shift_ctrl,
  input [4:0] amount,
  output reg [31:0] alu_out,
  output reg cpsr[31:0]
);

  always @(posedge CLK) begin
    case (alu_ctrl)
      4'b0000 : alu_out <= a & b;     //and
      4'b0001 : alu_out <= a | b;     //or
      4'b0010 : alu_out <= a + b;     //add
      4'b0011 : alu_out <= a - b;     //sub
      4'b0100 : alu_out <= ~a;        //not
      4'b0101 : alu_out <= a;         //buf
      4'b0110 : alu_out <= ~(a | b);  //nor
      4'b0111 : alu_out <= ~(a & b)   //nand
      4'b1000 : alu_out <= ~(a ^ b);  //xnor
      4'b1001 : alu_out <= a ^ b;     //xor
      4'b1010 : alu_out <= a * b;     //mult
      4'b1011 : alu_out <= a / b;     //div
      //4 ops to define
      4'b1100 : alu_out <= ~a;        //not
      4'b1101 : alu_out <= a;         //buf
      4'b1110 : alu_out <= ~(a | b);  //nor
      4'b1111 : alu_out <= ~(a & b)   //nand
    endcase

    case (rotate_shift_ctrl)
      2'b00 : alu_out <= alu_out << amount;  //shift left
      2'b01 : alu_out <= alu_out >> amount;  //shift right
      2'b10 : alu_out <= alu_out <<< amount; //rotate left
      2'b11 : alu_out <= alu_out >>> amount; //rotate right
    endcase

    //cpsr set based on the data provided
    cpsr[30] <= (alu_out == 0) ? 1'b1 : 1'b0;
    cpsr[31] <= (alu_out < 0) ? 1'b1 : 1'b0;

  end
endmodule


module FRWRD
(
  input clock_signal,
  input [4:0] a_address,
  input [4:0] b_address,
  input [4:0] data_dest_address,
  input data_register_write_ctrl,
  input [4:0] back_dest_address,
  input back_register_write_ctrl,

  output [1:0] a_data_ctrl,
  output [1:0] b_data_ctrl,

);

  
endmodule




/*--[Instruction Decoding Stage Modules]--*/

/*
  [Register Bank]
  @param input:: clock_signal = clock signal
  @param input:: a_address = address of the first register  
  @param input:: b_address = address of the second register that is also the destination reg address
  @param input:: phantom_a_address = the phantom branch address for the first register
  @param input:: phantom_b_address = the phantom address for the second register
  @param input:: write_register_ctrl = write back register write ctrl
  @param input:: write address = the write back address for the register that will have its contents written
  @param input:: write_data = what will be written at the above addr
  @param input:: cpsr_write_ctrl = ctrl to write into the cpsr
  @param input:: cpsr_data = data to write when the ctrl is enabled
  @param input:: pc_write_ctrl = pc write enabler
  @param input:: pc_data = data to write in pc

  @param output:: a_data = content of register in addr a
  @param output:: b_data = content of register in addr b
  @param output:: phantom_a_data = data for phantom branch
  @param output:: phantom_b_data = data for phantom branch
  @param output:: cpsr = current program status register
  @param output:: pc = program counter
*/

module REG
(
  input clock_signal,
  input [4:0] a_address,
  input [4:0] b_address,

  input [4:0] phantom_a_address,
  input [4:0] phantom_b_address,

  input [4:0] write_address,
  input [31:0] write_data,
  input write_register_ctrl,

  input cpsr_write_ctrl,
  input [31:0] cpsr_data,
  input pc_write_ctrl,
  input [31:0] pc_data,

  output reg [31:0] a_data,
  output reg [31:0] b_data,
  output reg [31:0] phantom_a_data,
  output reg [31:0] phantom_b_data,

  
  output reg [31:0] cpsr,
  output reg [31:0] pc
);

  reg [31:0] register[31:0];

  initial begin
    for (int i = 0; i <= 32; i = i + 1) begin
      register[i] <= 32'h0000;
    end
  end

  always @(posedge clock_signal) begin
    register[0] <= 32'h0000;

	  
    a_data <= register[a_address];
    b_data <= register[b_address];
    phantom_a_data <= register[phantom_a_address];
    phantom_b_data <= register[phantom_b_address];
	 

    register[write_address] <= (write_register_ctrl == 1) ? write_data : register[write_address];
    register[31] <= (cpsr_write_ctrl == 1) ? cpsr_data : register[31];
    register[30] <= (pc_write_ctrl == 1) ? pc_data : register[31];

    cpsr <= register[31];
    pc <= register[30];

  end

endmodule


/*
  [Control Unit]
  
*/

module CTRL
(
  input clock_signal,
  input [31:0] instruction,
  input [31:0] phantom_instruction,

  output mem_write_ctrl,
  output mem_read_ctrl,
  output register_write_ctrl,
  output alu_ctrl,
  output src_ctrl,
  output phantom_mem_write_ctrl,
  output phantom_mem_read_ctrl,
  output phantom_register_write_ctrl,
  output phantom_alu_ctrl,
  output phantom_src_ctrl,
);

  

endmodule


module HDU
(
  input clock_signal,
  input [4:0] execute_write_register_address,
  input execute_mem_read_ctrl,
  input [4:0] register_a_address,
  input [4:0] register_b_address,
  input [4:0] phantom_register_a_address,
  input [4:0] phantom_register_b_address,

  output IF_ID_stalling_ctrl,
  output phantom_IF_ID_stalling_ctrl
);

endmodule


module EXT
(
  input clock_signal,
  input [31:0] instruction,
  input [31:0] phantom_instruction,

  input [31:0] immediate,
  input [31:0] phantom_immediate,

);

endmodule

module MUX_EXSTAGE
(
  input branch_ctrl,

);

  always @(*) begin
    //send the correct signals to the execution stage registers, they will only be set at negedge

  
  end
endmodule





/*--[Instruction Fetch Stage Modules]--*/

/*
  [Instruction Mapper]
  @param input:: clock_signal = clock signal
  @param input:: instruction_address = the last instruction address that is in the pc register
  @param input:: phantom_instruction_address = the phantom address for the branched and received via the stage register

  @param output:: pc = program counter
  @param output:: phantom_pc = program counter for the next branched pc it will be defined from a mux that will be se in the execution stage
*/
module MAPPER
(
  input clock_signal,
  input [31:0] instruction_address,
  input [31:0] phantom_instruction_address,
  output reg [31:0] pc,
  output reg [31:0] phantom_pc
);

  always @(posedge clock_signal) begin 
    pc <= instruction_address+1;
    phantom_pc <= phantom_instruction_address;
  end
endmodule


/*
  [Branch Fetch MUX]
  @param input:: clock_signal = clock signal
  @param input:: instruction_address = the last instruction address that is in the pc register
  @param input:: phantom_instruction_address = the phantom address for the branched and received via the stage register
  @param output:: pc = program counter
  @param output:: phantom_pc = program counter for the next branched pc it will be defined from a mux that will be se in the execution stage
*/
module MUX_PHNT
(
  input clock_signal,
  input [28:0] instruction_address,
  input [28:0] phantom_branched_address,
  input branch_ctrl, 
  output reg [31:0] ,
  output reg [31:0] 
);

  always @(posedge clock_signal) begin 
    pc <= instruction_address+1;
    phantom_pc <= phantom_instruction_address;
  end
endmodule





/*--[Staging Modules]--*/

/*
  [IF/ID Stage Registers]
  @param input:: clock_signal = clock signal
  @param input:: fetch_instruction = instruction that was fetch from the instruction memory
  @param input:: fetch_phantom_instruction = instruction that was fatched based on the address of the branch or the instruction
  #@param input:: fetch_phantom_branch = saved branch pc used in order to get the next from branch

  #@param output:: saved_phantom_branch = the phantom path of the instruction to be written into the pc and also feed into the mux that defines the mapping
  @param output:: decode_instruction = normal branch instruction saved to be used next
  @param output:: phantom_decode_instruction = normal branch instruction saved to be used next in the phantom brancher
*/
module FETCH_DECODE
(
  input clock_signal,
  input reg [31:0] fetch_instruction,
  input reg [31:0] fetch_phantom_instruction,
  output reg [31:0] decode_instruction,
  output reg [31:0] decode_phantom_instruction,
); 

  always @(negedge clock_signal) begin 
    decode_instruction <= fetch_instruction;
    decode_phantom_instruction <= fetch_phantom_instruction;
  end

endmodule


/*
  [ID/EX Stage Registers]
  @param input:: clock_signal = clock signal
  @param input:: 
  @param input:: 
  @param input:: 

  #@param output:: 
  @param output:: 
  @param output:: 
*/
module DECODE_EXECUTE
(
  input clock_signal,
  input [31:0] decode_register_a_data,
  input [31:0] decode_register_b_data,
  input [31:0] decode_immediate,
  input decode_mem_write_ctrl,
  input decode_mem_read_ctrl,
  input decode_register_write_ctrl,
  input decode_alu_ctrl,
  input decode_src_ctrl,
  input [4:0] decode_a_register_address,
  input [4:0] decode_dest_register_address,

  output reg [31:0] execute_register_a_data,
  output reg [31:0] execute_register_b_data,
  output reg [31:0] execute_immediate,
  output execute_mem_write_ctrl,
  output execute_mem_read_ctrl,
  output execute_register_write_ctrl,
  output execute_alu_ctrl,
  output execute_src_ctrl,
  input [4:0] execute_a_register_address,
  input [4:0] execute_dest_register_address,

); 

  always @(negedge clock_signal) begin 
    execute_register_a_data <= decode_register_a_data;
    execute_register_b_data <= decode_register_b_data;
    execute_immediate <= decode_immediate;
    execute_mem_write_ctrl <= decode_mem_write_ctrl;
    execute_mem_read_ctrl <= decode_mem_read_ctrl;
    execute_register_write_ctrl <= decode_register_write_ctrl;
    execute_alu_ctrl <= decode_alu_ctrl;
    execute_src_ctrl <= decode_src_ctrl;
    execute_a_register_address <= decode_a_register_address;
    execute_dest_register_address <= decode_dest_register_address
  end

endmodule



























