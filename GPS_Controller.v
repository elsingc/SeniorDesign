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
   output reg        data_valid,
   output reg [7:0]  lon_deg,
	output reg [23:0] lon_submins,
	output reg        lon_east,
   output reg [6:0]  lat_deg,
	output reg [23:0] lat_submins,
	output reg        lat_north,
   output reg [31:0] time_,
   output reg [31:0] ground_speed,

   output [7:0]      tx_data,
   output            tx_send,
   input             tx_busy,
   output            tx_req_speed,
   input             tx_cur_speed,

   input             rx_new,
   input [7:0]       rx_data,

   input             rst,
   input             clk
   );

   wire [1:0]        rom_message;
   wire [5:0]        rom_index;
   wire [5:0]        rom_length;
   wire [7:0]        rom_data;

   gps_rom messages (.message(rom_message),
                     .index(rom_index),
                     .data(tx_data),
                     .length(rom_length) );

   localparam s_cfg_send = 0;
   localparam s_cfg_check_finished = 1;
   localparam s_cfg_wait = 2;
   localparam s_idle = 3;
   localparam s_rx_save = 4;
   localparam s_rx_process = 5;

   reg [2:0]         state_d, state_q;
   reg               tx_send_d, tx_send_q;
   reg [1:0]         rom_message_d, rom_message_q;
   reg [5:0]         rom_index_d, rom_index_q;
   reg [23:0]        send_wait_ctr_d, send_wait_ctr_q;

   reg [7:0]         rx_buffer [0:127];
   reg [6:0]         rx_size;


   assign tx_req_speed = (rom_message >= 1);
   assign rom_message = rom_message_q;
   assign rom_index = rom_index_q;
   assign tx_send = tx_send_q;


   always @(posedge clk or posedge rst) begin

      if (rst) begin
			rom_message_q <= 0;
			rom_index_q <= 0;
			tx_send_q <= 0;
			state_q <= s_cfg_send;
			send_wait_ctr_q <= 0;
			//data_valid <= 0; 
		end else begin
			tx_send_q <= tx_send_d;
			state_q <= state_d;
			rom_message_q <= rom_message_d;
			rom_index_q <= rom_index_d;
			send_wait_ctr_q <= send_wait_ctr_d;
      end // else: !if(rst)
   end // always @ (posedge clk or posedge rst)


   always @(*) begin
      state_d <= state_q;
      tx_send_d <= tx_send_q;
      rom_message_d <= rom_message_q;
      rom_index_d <= rom_index_q;
      send_wait_ctr_d <= send_wait_ctr_q;

      case (state_q)
			s_cfg_send:begin
				if (!tx_busy && tx_req_speed == tx_cur_speed) begin
					tx_send_d <= 1;
					state_d <= s_cfg_check_finished;
				end
			end

			s_cfg_check_finished: begin
				tx_send_d <= 0;

				rom_index_d <= rom_index_q + 1;
				if (rom_index_d == rom_length) begin
					rom_index_d <= 0;
					rom_message_d <= rom_message_q + 1;
					if (rom_message_d == 3) 
						state_d <= s_idle;
					else begin
						state_d <= s_cfg_wait;
						send_wait_ctr_d <= 0;
					end
				end else 
					state_d <= s_cfg_send;
			end // case: s_cfg_check_finished

			s_cfg_wait: if (!tx_busy) begin
				if (send_wait_ctr_d == 24'hffffff) 
					state_d <= s_cfg_send;
				else 
					send_wait_ctr_d <= send_wait_ctr_q + 1;
			end

	// ------

			s_idle: if (rx_new && rx_data == "$") begin
				state_d <= s_rx_save;
				rx_size <= 0;
			end

			s_rx_save: if (rx_new) begin
				if (rx_data == "*") 
					state_d <= s_rx_process;
				else begin
					rx_buffer[rx_size] <= rx_data;
					rx_size <= rx_size + 1;
				end
			end

			s_rx_process: begin
				case ({rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3], rx_buffer[4]})
					"GPGGA": begin
						if (rx_size != 68) 
							data_valid <= 0;
						else begin
							data_valid <= 1;
							lat_deg <= (rx_buffer[17] - "0") * 10 + (rx_buffer[18] - "0");
							lat_submins <= (rx_buffer[19] - "0") * 100000 +
												(rx_buffer[20] - "0") * 10000 +
												(rx_buffer[22] - "0") * 1000 +
												(rx_buffer[23] - "0") * 100 +
												(rx_buffer[24] - "0") * 10 +
												(rx_buffer[25] - "0");
							lat_north <= (rx_buffer[27] == "N");

							lon_deg <= (rx_buffer[29] - "0") * 100 + (rx_buffer[30] - "0") * 10 + (rx_buffer[31] - "0");
							lon_submins <= (rx_buffer[32] - "0") * 100000 +
												(rx_buffer[33] - "0") * 10000 +
												(rx_buffer[35] - "0") * 1000 +
												(rx_buffer[36] - "0") * 100 +
												(rx_buffer[37] - "0") * 10 +
												(rx_buffer[38] - "0");
							lon_east <= (rx_buffer[40] == "E");
						end
					end
					"GPVTG": ;
		//$GPVTG,59.11,T,,M,0.09,N,0.17,K,A
					default: ;
				endcase
				state_d <= s_idle;
			end
      endcase // case (state_q)
   end // always @ (*)

endmodule
