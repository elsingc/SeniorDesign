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
module GPS_Controller(
	output [19:0] x_gps,
	output [19:0] y_gps,
	output [19:0] z_gps,
	output [19:0] time_gps,
	output [19:0] ground_speed,
	output ena,
	output [7:0]data_wr,
	input [7:0] data_rd,
	input busy,
	input new,
	input rst,
	input clk
    );


endmodule
