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
   
   output [0:80*8-1] gps_ram,

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

   reg [7:0]         rx_buffer [0:79];
   reg [6:0]         rx_size;


   assign tx_req_speed = (rom_message >= 1);
   assign rom_message = rom_message_q;
   assign rom_index = rom_index_q;
   assign tx_send = tx_send_q;
   
   // ffr the first line would have worked
   assign gps_ram[00*8+:8] = rx_buffer[00];
   assign gps_ram[008:015] = rx_buffer[01];
   assign gps_ram[016:023] = rx_buffer[02];
   assign gps_ram[024:031] = rx_buffer[03];
   assign gps_ram[032:039] = rx_buffer[04];
   assign gps_ram[040:047] = rx_buffer[05];
   assign gps_ram[048:055] = rx_buffer[06];
   assign gps_ram[056:063] = rx_buffer[07];
   assign gps_ram[064:071] = rx_buffer[08];
   assign gps_ram[072:079] = rx_buffer[09];
   
   assign gps_ram[080:087] = rx_buffer[10];
   assign gps_ram[088:095] = rx_buffer[11];
   assign gps_ram[096:103] = rx_buffer[12];
   assign gps_ram[104:111] = rx_buffer[13];
   assign gps_ram[112:119] = rx_buffer[14];
   assign gps_ram[120:127] = rx_buffer[15];
   assign gps_ram[128:135] = rx_buffer[16];
   assign gps_ram[136:143] = rx_buffer[17];
   assign gps_ram[144:151] = rx_buffer[18];
   assign gps_ram[152:159] = rx_buffer[19];
   
   assign gps_ram[160:167] = rx_buffer[20];
   assign gps_ram[168:175] = rx_buffer[21];
   assign gps_ram[176:183] = rx_buffer[22];
   assign gps_ram[184:191] = rx_buffer[23];
   assign gps_ram[192:199] = rx_buffer[24];
   assign gps_ram[200:207] = rx_buffer[25];
   assign gps_ram[208:215] = rx_buffer[26];
   assign gps_ram[216:223] = rx_buffer[27];
   assign gps_ram[224:231] = rx_buffer[28];
   assign gps_ram[232:239] = rx_buffer[29];
   
   assign gps_ram[240:247] = rx_buffer[30];
   assign gps_ram[248:255] = rx_buffer[31];
   assign gps_ram[256:263] = rx_buffer[32];
   assign gps_ram[264:271] = rx_buffer[33];
   assign gps_ram[272:279] = rx_buffer[34];
   assign gps_ram[280:287] = rx_buffer[35];
   assign gps_ram[288:295] = rx_buffer[36];
   assign gps_ram[296:303] = rx_buffer[37];
   assign gps_ram[304:311] = rx_buffer[38];
   assign gps_ram[312:319] = rx_buffer[39];
   
   assign gps_ram[320:327] = rx_buffer[40];
   assign gps_ram[328:335] = rx_buffer[41];
   assign gps_ram[336:343] = rx_buffer[42];
   assign gps_ram[344:351] = rx_buffer[43];
   assign gps_ram[352:359] = rx_buffer[44];
   assign gps_ram[360:367] = rx_buffer[45];
   assign gps_ram[368:375] = rx_buffer[46];
   assign gps_ram[376:383] = rx_buffer[47];
   assign gps_ram[384:391] = rx_buffer[48];
   assign gps_ram[392:399] = rx_buffer[49];
   
   assign gps_ram[400:407] = rx_buffer[50];
   assign gps_ram[408:415] = rx_buffer[51];
   assign gps_ram[416:423] = rx_buffer[52];
   assign gps_ram[424:431] = rx_buffer[53];
   assign gps_ram[432:439] = rx_buffer[54];
   assign gps_ram[440:447] = rx_buffer[55];
   assign gps_ram[448:455] = rx_buffer[56];
   assign gps_ram[456:463] = rx_buffer[57];
   assign gps_ram[464:471] = rx_buffer[58];
   assign gps_ram[472:479] = rx_buffer[59];
   
   assign gps_ram[480:487] = rx_buffer[60];
   assign gps_ram[488:495] = rx_buffer[61];
   assign gps_ram[496:503] = rx_buffer[62];
   assign gps_ram[504:511] = rx_buffer[63];
   assign gps_ram[512:519] = rx_buffer[64];
   assign gps_ram[520:527] = rx_buffer[65];
   assign gps_ram[528:535] = rx_buffer[66];
   assign gps_ram[536:543] = rx_buffer[67];
   assign gps_ram[544:551] = rx_buffer[68];
   assign gps_ram[552:559] = rx_buffer[69];
   
   assign gps_ram[560:567] = rx_buffer[70];
   assign gps_ram[568:575] = rx_buffer[71];
   assign gps_ram[576:583] = rx_buffer[72];
   assign gps_ram[584:591] = rx_buffer[73];
   assign gps_ram[592:599] = rx_buffer[74];
   assign gps_ram[600:607] = rx_buffer[75];
   assign gps_ram[608:615] = rx_buffer[76];
   assign gps_ram[616:623] = rx_buffer[77];
   assign gps_ram[624:631] = rx_buffer[78];
   assign gps_ram[632:639] = rx_buffer[79];

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
