`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:06:23 12/15/2013 
// Design Name: 
// Module Name:    Gyro_Controller 
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
module Gyro_Controller(
	output [19:0] roll,
	output [19:0] pitch,
	output [19:0] yaw,
	output ena,
	output [7:0] data_wr,
	input [7:0] data_rd,
	input busy,
	input new,
	input rst,
	input clk
    );


endmodule
