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

   wire [7:0]   cfg_messages  [0:50] = "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
   wire [7:0]   cfg_baud_rate [0:19] = "$PMTK251,115200*1F\r\n";
   wire [7:0]   cfg_fix_rate  [0:16] = "$PMTK220,100*2F\r\n";

   assign data = (message == 0) ? cfg_baud_rate [index]
     : (message == 1) ? cfg_messages  [index]
     : (message == 2) ? cfg_fix_rate  [index]
     : 8'h00;

   assign length = (message == 0) ? 6'd20
	           : (message == 1) ? 6'd51
		   : (message == 2) ? 6'd17
		   : 0;

endmodule
