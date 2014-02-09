`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:48:05 02/04/2014 
// Design Name: 
// Module Name:    gps 
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
module gps_rom
  (
   input [1:0]  message,
   input [5:0]  index,
   output [7:0] data,
   output [5:0] length
   );

   wire [7:0] cfg_nav5_data [43:0];
   
   assign cfg_nav5_data[0] = 8'hB5;
   assign cfg_nav5_data[1] = 8'h62;
   assign cfg_nav5_data[2] = 8'h06;
   assign cfg_nav5_data[3] = 8'h24;
   assign cfg_nav5_data[4] = 8'h24;
   assign cfg_nav5_data[5] = 8'h00;
   assign cfg_nav5_data[6] = 8'h01;
   assign cfg_nav5_data[7] = 8'h00;
   assign cfg_nav5_data[8] = 8'h07;
   assign cfg_nav5_data[9] = 8'h00;
   assign cfg_nav5_data[10] = 8'h00;
   assign cfg_nav5_data[11] = 8'h00;
   assign cfg_nav5_data[12] = 8'h00;
   assign cfg_nav5_data[13] = 8'h00;
   assign cfg_nav5_data[14] = 8'h00;
   assign cfg_nav5_data[15] = 8'h00;
   assign cfg_nav5_data[16] = 8'h00;
   assign cfg_nav5_data[17] = 8'h00;
   assign cfg_nav5_data[18] = 8'h00;
   assign cfg_nav5_data[19] = 8'h00;
   assign cfg_nav5_data[20] = 8'h00;
   assign cfg_nav5_data[21] = 8'h00;
   assign cfg_nav5_data[22] = 8'h00;
   assign cfg_nav5_data[23] = 8'h00;
   assign cfg_nav5_data[24] = 8'h00;
   assign cfg_nav5_data[25] = 8'h00;
   assign cfg_nav5_data[26] = 8'h00;
   assign cfg_nav5_data[27] = 8'h00;
   assign cfg_nav5_data[28] = 8'h00;
   assign cfg_nav5_data[29] = 8'h00;
   assign cfg_nav5_data[30] = 8'h00;
   assign cfg_nav5_data[31] = 8'h00;
   assign cfg_nav5_data[32] = 8'h00;
   assign cfg_nav5_data[33] = 8'h00;
   assign cfg_nav5_data[34] = 8'h00;
   assign cfg_nav5_data[35] = 8'h00;
   assign cfg_nav5_data[36] = 8'h00;
   assign cfg_nav5_data[37] = 8'h00;
   assign cfg_nav5_data[38] = 8'h00;
   assign cfg_nav5_data[39] = 8'h00;
   assign cfg_nav5_data[40] = 8'h00;
   assign cfg_nav5_data[41] = 8'h00;
   assign cfg_nav5_data[42] = 8'h56;
   assign cfg_nav5_data[43] = 8'hD6;

   wire [7:0] cfg_msg_posllh_data [10:0];
   
   assign cfg_msg_posllh_data[0] = 8'hB5;
   assign cfg_msg_posllh_data[1] = 8'h62;
   assign cfg_msg_posllh_data[2] = 8'h06;
   assign cfg_msg_posllh_data[3] = 8'h01;
   assign cfg_msg_posllh_data[4] = 8'h03;
   assign cfg_msg_posllh_data[5] = 8'h00;
   assign cfg_msg_posllh_data[6] = 8'h01;
   assign cfg_msg_posllh_data[7] = 8'h02;
   assign cfg_msg_posllh_data[8] = 8'h01;
   assign cfg_msg_posllh_data[9] = 8'h0e;
   assign cfg_msg_posllh_data[10] = 8'h47;

   wire [7:0] cfg_msg_velned_data [10:0];
   
   assign cfg_msg_velned_data[0] = 8'hB5;
   assign cfg_msg_velned_data[1] = 8'h62;
   assign cfg_msg_velned_data[2] = 8'h06;
   assign cfg_msg_velned_data[3] = 8'h01;
   assign cfg_msg_velned_data[4] = 8'h03;
   assign cfg_msg_velned_data[5] = 8'h00;
   assign cfg_msg_velned_data[6] = 8'h01;
   assign cfg_msg_velned_data[7] = 8'h12;
   assign cfg_msg_velned_data[8] = 8'h01;
   assign cfg_msg_velned_data[9] = 8'h1e;
   assign cfg_msg_velned_data[10] = 8'h67;
	
	assign data = (message == 0) ? cfg_nav5_data [index]
	            : (message == 1) ? cfg_msg_posllh_data [index]
					: (message == 2) ? cfg_msg_velned_data [index]
					: 8'd0;
	
	assign length = (message == 0) ? 6'd44
	              : (message == 1) ? 6'd11
					  : (message == 2) ? 6'd11
					  : 6'd0;
        
endmodule
