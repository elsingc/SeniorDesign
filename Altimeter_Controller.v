`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:04:45 12/15/2013 
// Design Name: 
// Module Name:    Altimeter_Controller 
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
module Altimeter_Controller(
	output [19:0] pressure,
	output [19:0] temp,
	output [19:0] delta_pressure,
	output [19:0] delta_temp,
	output [19:0] min_pressure,
	output [19:0] max_pressure,
	output [19:0] min_temp,
	output [19:0] max_temp,
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
