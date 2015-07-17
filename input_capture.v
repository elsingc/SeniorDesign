`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:24:06 01/30/2014 
// Design Name: 
// Module Name:    analog_inputs 
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
module input_capture(
    input clk,
	 input rst,
	 output [3:0] channel,
	 input new_sample,
	 input [9:0] sample,
	 input [3:0] sample_channel,
	 output [7:0] led,
	 output reg [9:0] analog_channel_0,
	 output reg [9:0] analog_channel_1,
	 output reg [9:0] analog_channel_2,
	 output reg [9:0] analog_channel_3,
	 output reg [9:0] analog_channel_4,
	 output reg [9:0] analog_channel_5,
	 output reg [9:0] analog_channel_6,
	 output reg [9:0] analog_channel_7,
	 output reg [9:0] analog_channel_8,
	 output reg [9:0] analog_channel_9
	 );
	 

always @(*) begin
	
	if(new_sample && sample_channel == 4'd0) begin
		analog_channel_0 <= sample;
	end else if(new_sample && sample_channel == 4'd1) begin
		analog_channel_1 <= sample;
	end else if(new_sample && sample_channel == 4'd2) begin
		analog_channel_2 <= sample;
	end else if(new_sample && sample_channel == 4'd3) begin
		analog_channel_3 <= sample;
	end else if(new_sample && sample_channel == 4'd4) begin
		analog_channel_4 <= sample;
	end else if(new_sample && sample_channel == 4'd5) begin
		analog_channel_5 <= sample;
	end else if(new_sample && sample_channel == 4'd6) begin
		analog_channel_6 <= sample;
	end else if(new_sample && sample_channel == 4'd7) begin
		analog_channel_7 <= sample;
	end else if(new_sample && sample_channel == 4'd8) begin
		analog_channel_8 <= sample;
	end else if(new_sample && sample_channel == 4'd9) begin
		analog_channel_9 <= sample;
	end
end

endmodule
