`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:08:50 12/15/2013 
// Design Name: 
// Module Name:    GPS_Controller 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module GPS_Controller
  (
   output [31:0] long,
   output [31:0] lat,
   output [31:0] alt,
   output [31:0] time_,
   output [31:0] ground_speed,

   output [7:0]  tx_data,
   output        tx_send,
   input         tx_busy,
	
   input [7:0]   rx_data,
   input         rx_new,
	
   input         busy,
   input         new,
   input         rst,
   input         clk
   );

   reg [15:0]    msg_type;
   reg [1:0]     state;
	
	reg tx_send;
   

	reg[1:0] rom_msg;
	reg[5:0] rom_index;
	wire[5:0] rom_length;
	gps_rom messages (
		.message(rom_msg),
		.index(rom_index),
		.data(tx_data),
		.length(rom_length) );
	
	localparam state_wait_for_tx_free = 0;
	localparam state_set_up_tx = 1;
	localparam state_check_finished_send = 2;
	localparam state_butts = 3;

   always @(posedge clk or posedge rst)
     begin
        if (rst)
          begin
          end else begin
			   case (state)
					state_wait_for_tx_free: if (!tx_busy) state <= state_set_up_tx;
					state_set_up_tx: begin
						tx_send <= 1;
						state <= state_check_finished_send;
					end
					state_check_finished_send: begin
						tx_send <= 0;
						if (rom_index == rom_length - 1) begin
							if (rom_msg == 2) state <= state_butts;
							else begin
								rom_msg <= rom_msg + 1;
								rom_index <= 0;
							end
						end else rom_index <= rom_index + 1;
					end
					state_butts: ;
				endcase
          end
     end

endmodule
