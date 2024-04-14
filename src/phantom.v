

module phantom #()(); 



DECODE_EXECUTE de_stage_registers(
  clock, 
  decode_a_data, 
  decode_b_data, 
  decode_immd, 
  decode_mem_read_ctrl, 
  decode_register_write_ctrl, 
  decode_alu_ctrl, 
  decode_src_ctrl, 
  decode_a_register_address, 
  decode_dest_register_address, 
  execute_register_a_data, 
  execute_register_b_data, 
  execute_immediate, 
  execute_mem_write_ctrl, 
  execute_mem_read_ctrl, 
  execute_register_write_ctrl, 
  execute_alu_ctrl, 
  execute_src_ctrl, 
  execute_a_register_address, 
  execute_b_register_address
);

//execute stage

MUX_SRC b_src(
  .(),
  .(),

);

FRWRD forwarding_unit(
  clock, 
  .a_address(execute_a_register_address), 
  .b_address(execute_b_register_address), 
  .data_dest_address(data_dest_address), 
  .data_register_write_ctrl(data_register_write_ctrl), 
  .back_dest_address(back_dest_address), 
  .back_register_write_ctrl(back_register_write_ctrl), 
  .a_data_ctrl(forwarding_a_data_ctrl), 
  .b_data_ctrl(forwarding_b_data_ctrl)
);

MUX_FRWRD a_frwrd(
  .ctrl(forwarding_a_ctrl),
  .ex_data(a_data), 
  .dt_data(data_immd),
  .wb_data(back_immd), 
  .output(a)
);

MUX_FRWRD b_frwrd(
  .ctrl(forwarding_b_ctrl),
  .ex_data(b_data), 
  .dt_data(data_immd),
  .wb_data(back_immd), 
  .output(b)
);

ALU alu(
  .clock_signal(clock),
  .a(a), 
  .b(b), 
  .alu_ctrl(alu_ctrl), 
  .rotate_shift_ctrl(rotate_shift_ctrl),
  .amount(amount), 
  .alu_out(alu_out), 
  .cpsr(cpsr)
);

endmodule




























