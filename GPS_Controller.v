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
   output reg [31:0] lon,
   output reg [31:0] lat,
   output reg [31:0] time_,
   output reg [31:0] ground_speed,

   output [7:0]      tx_data,
   output reg        tx_send,
   input             tx_busy,
  
   input [7:0]       rx_data,
   input             rx_new,
  
   input             busy,
   input             new,
   input             rst,
   input             clk
   );
   
   reg [1:0]         rom_msg;
   reg [5:0]         rom_index;
   wire [5:0]        rom_length;
   
   gps_rom messages (
                     .message(rom_msg),
                     .index(rom_index),
                     .data(tx_data),
                     .length(rom_length) );

   wire [7:0]        ack_header [5:0];
   assign ack_header[0] = 8'hB5;
   assign ack_header[1] = 8'h62;
   assign ack_header[2] = 8'h05;
   assign ack_header[3] = 8'h01;
   assign ack_header[4] = 8'h02;
   assign ack_header[5] = 8'h00;

   reg [4:0]         ack_index;

   reg [15:0]        msg_type;
   reg [15:0]        msg_length;
   reg [4:0]         msg_bytes_to_dump;
   reg [4:0]         state;
   
   localparam s_cfg_send = 0;
   localparam s_cfg_check_finished = 1;
   localparam s_cfg_check_ack_byte = 2;
   
   localparam s_idle = 3;
   localparam s_rx_sync_2 = 4;
   localparam s_rx_msg_class = 5;  
   localparam s_rx_msg_id = 6;   
   localparam s_rx_length_lsb = 7;
   localparam s_rx_length_msb = 8;
   
   localparam s_nav_posllh_time_0 = 9;
   localparam s_nav_posllh_time_1 = 10;
   localparam s_nav_posllh_time_2 = 11;
   localparam s_nav_posllh_time_3 = 12;
   localparam s_nav_posllh_lon_0 = 13;
   localparam s_nav_posllh_lon_1 = 14;
   localparam s_nav_posllh_lon_2 = 15;
   localparam s_nav_posllh_lon_3 = 16;
   localparam s_nav_posllh_lat_0 = 17;
   localparam s_nav_posllh_lat_1 = 18;
   localparam s_nav_posllh_lat_2 = 19;
   localparam s_nav_posllh_lat_3 = 20;
   localparam s_nav_posllh_dump = 21;
   
   localparam s_nav_velned_dump_pre = 22;
   localparam s_nav_velned_speed_0 = 23;
   localparam s_nav_velned_speed_1 = 24;
   localparam s_nav_velned_speed_2 = 25;
   localparam s_nav_velned_speed_3 = 26;
   localparam s_nav_velned_dump_post = 27;
   
   localparam s_dump_message = 28;
   
   localparam s_error = 31;
   

   always @(posedge clk or posedge rst) begin
      
      if (rst) begin
         rom_msg <= 0;
         rom_index <= 0;
         state <= s_cfg_send;
      end
        
      else begin
         case (state)
           s_cfg_send:
             if (!tx_busy) begin
                tx_send <= 1;
                state <= s_cfg_check_finished;
             end

           s_cfg_check_finished: begin
              tx_send <= 0;
              
              rom_index <= rom_index + 1;
              if (rom_index == rom_length) begin
                 ack_index <= 0;
                 state <= s_cfg_check_ack_byte;
              end
           end

           s_cfg_check_ack_byte:
             if (rx_new) begin
                if (ack_index < 6 && ack_header[ack_index] != rx_data)
                  state <= s_error;
                else begin
                   ack_index <= ack_index + 1;
                   if (ack_index == 10)
                     if (rom_msg != 2) begin
                        rom_msg <= rom_msg + 1;
                        rom_index <= 0;                          
                        state <= s_cfg_send;
                     end                         
                     else
                       state <= s_idle;
                end // else: !if(ack_index < 6 && ack_header[ack_index] != rx_data)
             end
               
           s_idle:
             if (rx_new && rx_data == 8'hB5) state <= s_rx_sync_2;
               
           s_rx_sync_2:
             if (rx_new) state <= (rx_data == 8'h62) ? s_rx_msg_class : s_idle;
               
           s_rx_msg_class:
             if (rx_new) begin
                msg_type[15:8] <= rx_data;
                state <= s_rx_msg_id;
             end
               
           s_rx_msg_id: if (rx_new) begin
              msg_type[7:0] <= rx_data;
              state <= s_rx_length_lsb;
           end
               
           s_rx_length_lsb: if (rx_new) begin
              msg_length[7:0] <= rx_data;
              state <= s_rx_length_msb;
           end
               
           s_rx_length_msb: if (rx_new) begin
              msg_length[15:8] <= rx_data;
                  
              case (msg_type)
                16'h0112: // NAV-VELNED
                  if (msg_length != 36)
                    state <= s_error;
                  else begin
                     msg_bytes_to_dump <= 20;
                     state <= s_nav_velned_dump_pre;
                  end
                
                16'h0102: // NAV-POSLLH
                  if (msg_length != 28)
                    state <= s_error;
                  else
                    state <= s_nav_posllh_time_0;
                    
                default: begin
                   msg_bytes_to_dump <= msg_length + 2;
                   state <= s_dump_message;
                end
              endcase // case (msg_type)
           end // if (rx_new)
           
    
           // NAV-POSLLH

           // GPS time
           s_nav_posllh_time_0:
             if (rx_new) begin
                time_[7:0] <= rx_data;
                state <= s_nav_posllh_time_1;
             end
               
           s_nav_posllh_time_1:
             if (rx_new) begin
                time_[15:8] <= rx_data;
                state <= s_nav_posllh_time_2;
             end
               
           s_nav_posllh_time_2:
             if (rx_new) begin
                time_[23:16] <= rx_data;
                state <= s_nav_posllh_time_3;
             end
               
           s_nav_posllh_time_3:
             if (rx_new) begin
                time_[31:24] <= rx_data;
                state <= s_nav_posllh_lon_0;
             end
               
           // longitude    
           s_nav_posllh_lon_0:
             if (rx_new) begin
                lon[7:0] <= rx_data;
                state <= s_nav_posllh_lon_1;
             end
               
           s_nav_posllh_lon_1:
             if (rx_new) begin
                lon[15:8] <= rx_data;
                state <= s_nav_posllh_lon_2;
             end
               
           s_nav_posllh_lon_2:
             if (rx_new) begin
                lon[23:16] <= rx_data;
                state <= s_nav_posllh_lon_3;
             end
               
           s_nav_posllh_lon_3:
             if (rx_new) begin
                lon[31:24] <= rx_data;
                state <= s_nav_posllh_lat_0;
             end
               
           // latitude         
           s_nav_posllh_lat_0:
             if (rx_new) begin
                lat[7:0] <= rx_data;
                state <= s_nav_posllh_lat_1;
             end
               
           s_nav_posllh_lat_1:
             if (rx_new) begin
                lat[15:8] <= rx_data;
                state <= s_nav_posllh_lat_2;
             end
               
           s_nav_posllh_lat_2:
             if (rx_new) begin
                lat[23:16] <= rx_data;
                state <= s_nav_posllh_lat_3;
             end
               
           s_nav_posllh_lat_3:
             if (rx_new) begin
                lat[31:24] <= rx_data;
                msg_bytes_to_dump <= 18; // 16 payload bytes + checksum
                state <= s_nav_posllh_dump;
             end
               
           s_nav_posllh_dump:
             if (rx_new) begin
                msg_bytes_to_dump <= msg_bytes_to_dump - 1;
                if (msg_bytes_to_dump == 0)
                  state <= s_idle;
             end
               
           // NAV-VELNED
               
           s_nav_velned_dump_pre:
             if (rx_new) begin
                msg_bytes_to_dump <= msg_bytes_to_dump - 1;
                if (msg_bytes_to_dump == 0)
                  state <= s_nav_velned_speed_0;
             end
               
           s_nav_velned_speed_0:
             if (rx_new) begin
                ground_speed[7:0] <= rx_data;
                state <= s_nav_velned_speed_1;
             end
               
           s_nav_velned_speed_1:
             if (rx_new) begin
                ground_speed[15:8] <= rx_data;
                state <= s_nav_velned_speed_2;
             end
               
           s_nav_velned_speed_2:
             if (rx_new) begin
                ground_speed[23:16] <= rx_data;
                state <= s_nav_velned_speed_3;
             end
               
           s_nav_velned_speed_3:
             if (rx_new) begin
                ground_speed[31:24] <= rx_data;
                msg_bytes_to_dump <= 12;
                state <= s_nav_velned_dump_post;
             end
               
           s_nav_velned_dump_post:
             if (rx_new) begin
                msg_bytes_to_dump <= msg_bytes_to_dump - 1;
                if (msg_bytes_to_dump == 0)
                  state <= s_idle;
             end
               
           s_dump_message:
             if (rx_new) begin
                msg_bytes_to_dump <= msg_bytes_to_dump - 1;
                if (msg_bytes_to_dump == 0)
                  state <= s_idle;
             end
               
           s_error: ;
               
         endcase // case (state)
      end // else: !if(rst)
   end // always @ (posedge clk or posedge rst)

endmodule
