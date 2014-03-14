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
input [23:0] pressure,
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
input [7:0]  gps_lon_deg,
input [23:0] gps_lon_submins,
input [7:0]  gps_lat_deg,
input [23:0] gps_lat_submins,
input [7:0]  gps_status,
input [31:0] gps_time,
   input [0:80*8-1]      gps_ram,
input [31:0] ground_speed,
input [15:0] air_speed_p,
input [15:0] air_speed_n,
input rst,
input clk
    );

reg [23:0] int_pressure;
reg [15:0] 	int_alt_temp = 16'd0, 
				int_gyro_temp = 16'd0, 
				int_gyro_x = 16'd0, 
				int_gyro_y = 16'd0, 
				int_gyro_z = 16'd0, 
				int_x_accl = 16'd0, 
				int_y_accl = 16'd0,
				int_z_accl = 16'd0, 
				int_magm_x = 16'd0, 
				int_magm_y = 16'd0, 
				int_magm_z = 16'd0;
always@(*) begin
	if(rst)begin
	
	end else begin
		case(addr)
			//pressure msb
			8'd1:begin
				//data[7:4]<= 4'h0;
				data <= int_pressure[23:16];
			end
			
			//pressure csb
			8'd2:begin
				data <= int_pressure[15:8];
			end
			
			//pressure lsb
			8'd3:begin
				data <= int_pressure[7:0];
			end
			 
			//temp msb
			8'd4:begin
				data <= int_alt_temp[15:8];
			end
			
			//temp lsb
			8'd5:begin
				data <= int_alt_temp[7:0];
			end
			
			//temp msb
			8'd6:begin
				data <= int_gyro_temp[15:8];
			end
			
			//temp lsb
			8'd7:begin
				data <= int_gyro_temp[7:0];
			end
			
			//x_accl msb
			8'd8:begin
				data <= int_x_accl[15:8];
			end
			
			//x_accl lsb
			8'd9:begin
				data <= int_x_accl[7:0];
			end
			
			//y_accl msb
			8'd10:begin
				data <= int_y_accl[15:8];
			end
			
			//y_accl lsb
			8'd11:begin
				data <= int_y_accl[7:0];
			end
			
			//z_accl msb
			8'd12:begin
				data <= int_z_accl[15:8];
			end
			
			//z_accl lsb
			8'd13:begin
				data <= int_z_accl[7:0];
			end
			
			//roll msb
			8'd14:begin
				data <= int_gyro_x[15:8];
			end
			
			//roll lsb
			8'd15:begin
				data <= int_gyro_x[7:0];
			end
			
			//pitch msb
			8'd16:begin
				data <= int_gyro_y[15:8];
			end
			
			//pitch lsb
			8'd17:begin
				data <= int_gyro_y[7:0];
			end
			
			//yaw msb
			8'd18:begin
				data <= int_gyro_z[15:8];
			end
			
			//yaw lsb
			8'd19:begin
				data <= int_gyro_z[7:0];
			end
			
			//roll msb
			8'd20:begin
				data <= int_magm_x[15:8];
			end
			
			//roll lsb
			8'd21:begin
				data <= int_magm_x[7:0];
			end
			
			//pitch msb
			8'd22:begin
				data <= int_magm_y[15:8];
			end
			
			//pitch lsb
			8'd23:begin
				data <= int_magm_y[7:0];
			end
			
			//yaw msb
			8'd24:begin
				data <= int_magm_z[15:8];
			end
			
			//yaw lsb
			8'd25:begin 
				data <= int_magm_z[7:0];
			end
			
			8'd26:begin
				data <= gps_lon_deg;
			end
			
			8'd27:begin
				data <= gps_lon_submins [23:16];
			end
			
			8'd28:begin
				data <= gps_lon_submins [15:8];
			end
			
			8'd29:begin
				data <= gps_lon_submins [7:0];
			end
			
			8'd30:begin
				data <= gps_lon_deg;
			end
			
			8'd31:begin
				data <= gps_lon_submins [23:16];
			end
			
			8'd32:begin
				data <= gps_lon_submins [15:8];
			end
			
			8'd33:begin
				data <= gps_lon_submins [7:0];
			end
			
			8'd34:begin
				data <= gps_status;
			end

//input [31:0] gps_time,
//input [31:0] ground_speed,
			
			default:begin
                          if (addr < (80 + 35))
                             data <= gps_ram[(addr-35)*8+:8];
                          else
				data <= data;
			end
		endcase
	end
end

always@(negedge clk or posedge rst)begin
	if(rst)begin
	
	end else begin
		int_pressure <= pressure;
		int_alt_temp <= alt_temp;
		int_gyro_temp <= gyro_temp;
		int_gyro_x <= gyro_x;
		int_gyro_y <= gyro_y;
		int_gyro_z <= gyro_z;
		int_x_accl <= x_accl;
		int_y_accl <= y_accl;
		int_z_accl <= z_accl;
		int_magm_x <= magm_x;
		int_magm_y <= magm_y;
		int_magm_z <= magm_z;
	end
end
endmodule
