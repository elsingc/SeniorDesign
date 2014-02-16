`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:26:35 01/30/2014 
// Design Name: 
// Module Name:    I2C_Driver 
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
module I2C_Driver(
	//output reg [7:0] debug, 
   input clk,
	input rst,
	input rw,	
	input [7:0]data_wr,
	output reg [7:0]data_rd,
	output reg busy,
	output reg ready,
	output reg ack_err,
	input ena,
	input start_transfer,
	input stop_transfer,
	input r_start,
	inout SDA,
	inout SCL
   );	

//assign data_rd = data_rx;

localparam clock_speed = 50000000;
localparam com_speed = 400000;//0;
localparam divider =(clock_speed/com_speed)/4;


localparam STATE_SIZE = 9;
localparam 	system_ready = 0,
				start = 1,
				wait_transfer = 2,
				transfer = 3,
				ack = 4,
				wait_release_ack = 5,
				repeated_start_a = 6,
				repeated_start_b = 7,
				repeated_start_c = 8,
				stop = 9;
				
reg [STATE_SIZE:0] state = system_ready;

reg [3:0] bitcount  = 4'd0;
reg [7:0] count  = 8'd0;

reg data_clk = 1'b1;
reg scl_clk = 1'b1;
reg scl_ena = 1'b0;
reg scl_wait = 1'b0;
reg scl_repeated_start = 1'b0;
reg sda_int = 1'b1;
reg sda_int_negedge = 1'b1;
reg sda_ena_n = 1'b1;
reg rstart_clk_high = 1'b0;
reg [7:0] data_tx = 8'h00;
reg [7:0] data_rx = 8'h00;
reg stretch = 1'b0;
assign SDA = sda_ena_n == 1'b0 ? 1'b0 : 1'bz;
assign SCL = (scl_ena == 1'b1) ? scl_clk : 1'bz;

always @(*) begin
	if(state == start) begin
		sda_ena_n <= data_clk;
	end else if(state == stop) begin
		sda_ena_n <= ~data_clk;
	end else if(state == ack || state == wait_release_ack) begin
		if(rw == 1'b1)begin
			sda_ena_n <= 1'b0;
		end else begin
			sda_ena_n <= sda_int;
		end
	end else begin
		sda_ena_n <= (sda_int);
	end
end

always @(posedge clk or posedge rst)begin
	if(rst) begin
		stretch <= 1'b0;
		count <= 8'd0;
	end else begin
		if(count ==((divider*4)-1)) begin
			count <= 8'd0;
		end else if(stretch == 0) begin
			count <= count + 1'b1;
		end
		if((count >= 0) && (count < (divider))) begin
			if(rstart_clk_high)begin
				scl_clk <= 1'b1;
			end else begin
				scl_clk <= 1'b0;
			end
			data_clk <= 1'b0;
		end else if((count >= divider) && (count < (divider*2))) begin
			if(rstart_clk_high)begin
				scl_clk <= 1'b1;
			end else begin
				scl_clk <= 1'b0;
			end
			data_clk <= 1'b1;
		end else if((count >= (divider*2)) && (count <=divider*3)) begin
			if(rstart_clk_high)begin
				scl_clk <= 1'b1;
			end else begin
				if(scl_wait)begin
					scl_clk <= 1'b0;
				end else begin
					scl_clk <= 1'bZ;
				end
				if(SCL == 0 && !scl_wait)begin
					stretch <= 1'b1;
				end else begin
					stretch <= 1'b0;
				end
			end
			data_clk <= 1'b1;
		end else begin
			if(rstart_clk_high)begin
				scl_clk <= 1'b1;
			end else begin
				if(scl_wait)begin
					scl_clk <= 1'b0;
				end else begin
					scl_clk <= 1'bZ;
				end
			end
			data_clk <= 1'b0;
		end
	end
end

always @(posedge data_clk or posedge rst)begin
	if(rst) begin
		state <= system_ready;
		busy <= 1'b1;
		scl_ena <= 1'b0;
		sda_int <= 1'b1;
		scl_repeated_start <= 1'b1;
		bitcount <= 4'd7;
	end else begin// if(data_clk == 1'b1) begin
		if(rw == 1'b1)begin
			//debug <= 8'hff;
		end else begin
			//debug <= 8'h00;
		end
		case (state)
			system_ready: begin
				scl_ena <= 1'b0;
				if(ena==1'b1)begin
					state <= start;
				end else begin
					busy <= 1'b0;
					ready <= 1'b1;
					state <= system_ready;
				end
			end
			
			//sends the start bit
			start: begin
				busy <= 1'b1;
				ready <= 1'b0;
				scl_ena <= 1'b1;
				scl_wait <= 1'b1;
				sda_int <= 1'b0;
				state <= wait_transfer;
			end
			
			//this state determines whether another read or write is performed before stoping
			wait_transfer: begin
				ready <= 1'b1;
				busy <= 1'b0;
				bitcount <= 4'd8; 
				scl_wait <= 1'b1;					
				scl_ena <= 1'b1;
				sda_int <= 1'b1;
				rstart_clk_high <= 1'b0;
				if(start_transfer/* && !stop_transfer*/) begin
					data_tx <= data_wr;
					state <= transfer;
				end else if(stop_transfer) begin
					state <= stop;
				end else if(r_start) begin
					state <= repeated_start_a;
				end else begin
					state <= wait_transfer;
				end
			end			
			
			
			//this side handles the setting up of data when in write mode
			//if in read, the data gets clocked in at the negedge which is handdled
			//by the negedge statemachine
			transfer: begin
				busy <= 1'b1;
				ready <= 1'b0;
				scl_ena <= 1'b1;
				scl_wait <= 1'b0;
				rstart_clk_high <= 1'b0;
				if(bitcount == 0) begin
					if(rw == 1'b1)begin
						sda_int <= 1'b0;
					//	debug <= data_rx;
						data_rd = data_rx;
					end else begin
						sda_int <= 1'b1;
					end
					scl_wait <= 1'b1;
					state <= ack;
				end else begin
					if(rw == 1'b0)begin
						sda_int <= data_tx[bitcount - 1]; //<---------might be wrong, bitcound -1 might be right
					end else begin
						sda_int <= 1'b1;
					end
					scl_wait <= 1'b0;
					bitcount <= bitcount - 1'b1;
					state <= transfer;
				end
				
			end
				
			ack:begin
				if(rw == 1'b1)begin 
					sda_int <= 1'b1;
					
				end else begin
					sda_int <= 1'b1;
				end
				rstart_clk_high <= 1'b0;
				scl_ena <= 1'b1;
				scl_wait <= 1'b0;
				busy <= 1'b1;
				ready <= 1'b0;
				state <= wait_transfer;
			end
			
			wait_release_ack: begin
				scl_ena <= 1'b1;
				scl_wait <= 1'b1;
				busy <= 1'b1;
				ready <= 1'b0;
				rstart_clk_high <= 1'b0;
				if(rw == 1'b1)begin
					sda_int <= 1'b1;
					
				end else begin
					sda_int <= 1'b1;
					if(SDA == 1'b1) begin
						state <= wait_transfer;
					end else begin
						state <= wait_release_ack;
					end
				end
			end
			//send clock high
			repeated_start_a: begin
				ready <= 1'b0;
				busy <= 1'b1;
				sda_int <= 1'b1;
				scl_ena <= 1'b1;
				rstart_clk_high <= 1'b1;
				state <= repeated_start_b;
			end
			
			//send sda low
			repeated_start_b: begin
				busy <= 1'b1;
				ready <= 1'b0;
				scl_ena <= 1'b0;
				scl_wait <= 1'b1;
				sda_int <= 1'b0;
				rstart_clk_high <= 1'b1;
				state <= repeated_start_c;
			end
			
			repeated_start_c: begin
				busy <= 1'b1;
				ready <= 1'b0;
				scl_ena <= 1'b1;
				scl_wait <= 1'b1;
				sda_int <= 1'b0;
				rstart_clk_high <= 1'b0;
				state <= wait_transfer;
			end
			
			stop:begin
				rstart_clk_high <= 1'b0;
				ready <= 1'b0;
				busy <= 1'b1;
				sda_int <= 1'b1;
				scl_ena <= 1'b0;
				state <= system_ready;
			end
			
			default:begin
				state <= system_ready;
			end
		endcase
	end
end


//not sure if this is needed yet
always @(negedge data_clk or posedge rst) begin
	if(rst) begin
		ack_err <= 1'b0;
	end else if(data_clk == 0) begin
		case(state)
			start:begin
				ack_err <= 1'b0;
			end
			
			transfer: begin
				if(rw == 1'b1)begin
					data_rx[bitcount] <= SDA;
				end
			end
			
			ack: begin
				if(rw == 1'b1)begin
					sda_int_negedge <= 1'b1;
				end else begin
					ack_err <= SDA | ack_err;
				end
			end
			
			default:begin
			end
		endcase
	end
end

endmodule
