`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:02:28 12/15/2013 
// Design Name: 
// Module Name:    Sensor_Reg 
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
module Sensor_Reg(
output [7:0] data,
input [7:0] addr,
input [19:0] pressure,
input [19:0] temp,
input [19:0] delta_pressure,
input [19:0] delta_temp,
input [19:0] min_pressure,
input [19:0] max_pressure,
input [19:0] min_temp,
input [19:0] max_temp,
input [19:0] roll,
input [19:0] pitch,
input [19:0] yaw,
input [19:0] x_accl,
input [19:0] y_accl,
input [19:0] z_accl,
input [19:0] x_gps,
input [19:0] y_gps,
input [19:0] z_gps,
input [19:0] time_gps,
input [19:0] ground_speed,
input [19:0] air_speed_p,
input [19:0] air_speed_n,
input rst,
input clk
    );


endmodule
