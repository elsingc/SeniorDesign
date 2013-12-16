`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:01:29 12/15/2013 
// Design Name: 
// Module Name:    Data_Controller 
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
module Data_Controller(
	input busy,
	input block, //set to 0
	output new_data_tx,
	output [7:0] data_tx,
	input new_data_rx,
	input [7:0] data_rx,
	input [7:0] data,
	output [7:0] addr,
	input rst,
	input clk
    );


endmodule
