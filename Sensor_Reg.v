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
output reg [7:0] data,
input [7:0] addr,
input [19:0] pressure,
input [15:0] alt_temp,
input [15:0] gyro_temp,
input [15:0] gyro_x,
input [15:0] gyro_y,
input [15:0] gyro_z,
input [15:0] x_accl,
input [15:0] y_accl,
input [15:0] z_accl,
input [15:0] magm_x,
input [15:0] magm_y,
input [15:0] magm_z,
input [31:0] gps_long,
input [31:0] gps_lat,
input [31:0] gps_alt,
input [31:0] gps_time,
input [31:0] ground_speed,
input [15:0] air_speed_p,
input [15:0] air_speed_n,
input rst,
input clk
    );


always@(*) begin
	if(rst)begin
	
	end else begin
		case(addr)
			//pressure msb
			1:begin
				data <= pressure[19:16];
			end
			
			//pressure csb
			2:begin
				data <= pressure[15:8];
			end
			
			//pressure lsb
			3:begin
				data <= pressure[7:0];
			end
			
			//temp msb
			4:begin
				data <= alt_temp[15:8];
			end
			
			//temp lsb
			5:begin
				data <= gyro_temp[7:0];
			end
			
			//x_accl msb
			6:begin
				data <= x_accl[15:8];
			end
			
			//x_accl lsb
			7:begin
				data <= x_accl[7:0];
			end
			
			//y_accl msb
			8:begin
				data <= y_accl[15:8];
			end
			
			//y_accl lsb
			9:begin
				data <= y_accl[7:0];
			end
			
			//z_accl msb
			10:begin
				data <= z_accl[15:8];
			end
			
			//z_accl lsb
			11:begin
				data <= z_accl[7:0];
			end
			
			//roll msb
			12:begin
				data <= gyro_x[15:8];
			end
			
			//roll lsb
			13:begin
				data <= gyro_x[7:0];
			end
			
			//pitch msb
			14:begin
				data <= gyro_y[15:8];
			end
			
			//pitch lsb
			15:begin
				data <= gyro_y[7:0];
			end
			
			//yaw msb
			16:begin
				data <= gyro_z[15:8];
			end
			
			//yaw lsb
			17:begin
				data <= gyro_z[7:0];
			end
			
			//roll msb
			18:begin
				data <= magm_x[15:8];
			end
			
			//roll lsb
			19:begin
				data <= magm_x[7:0];
			end
			
			//pitch msb
			20:begin
				data <= magm_y[15:8];
			end
			
			//pitch lsb
			21:begin
				data <= magm_y[7:0];
			end
			
			//yaw msb
			22:begin
				data <= magm_z[15:8];
			end
			
			//yaw lsb
			23:begin
				data <= magm_z[7:0];
			end
			
		endcase
	end
end

endmodule
