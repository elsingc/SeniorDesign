`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:13:56 12/15/2013 
// Design Name: 
// Module Name:    Camera_Initializer 
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
module Camera_Initializer(
	input start,
	input new_data,
	input [7:0] data,
	output ena,
	output [7:0] addr,
	output [7:0] sub_addr,
	output [7:0] data_wr,
	input [7:0] data_rd,
	input busy,
	input ack_err,
	input rst,
	input clk
    );


endmodule
