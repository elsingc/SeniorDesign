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
	output reg [7:0] debug,
	input busy,
	input block, //set to 0
	output reg new_data_tx,
	output reg [7:0] data_tx,
	input new_data_rx,
	input [7:0] data_rx,
	input [7:0] data,
	output reg [7:0] addr,
	output reg drop,
	input rst,
	input clk
    );

localparam DATA_LENGTH = 25;//35;
localparam STATE_SIZE = 5;
localparam	IDLE = 0,
				BURST_DATA_ADDR = 1,
				BURST_DATA_SEND = 2,
				GET_ADDR = 3,
				SEND_DATA = 4;
//				PRINT_BYTE = 2;
 
reg [STATE_SIZE-1:0] state;
 
 
always @(posedge clk or posedge rst) begin 
	if(rst)begin
		state <= IDLE;
	end else begin
		case (state)
			IDLE: begin
				new_data_tx <= 1'b0;
				data_tx <= 8'h00;
				//debug <= 8'h01;
				if(new_data_rx && data_rx == 8'h04) begin
				//	debug <= 8'h03;
					state <= GET_ADDR;
				end else if(new_data_rx && data_rx == 8'h05) begin
				//	debug <= 8'h02;
					addr <= 8'h00;
					state <= BURST_DATA_ADDR;
				end else if(new_data_rx && data_rx == 8'h42) begin
					addr<= 8'h00;
					state <= IDLE;
					drop <= ~drop;
				end else begin
					debug <= data_rx;
					state <= IDLE;
				end
			end
			
			BURST_DATA_ADDR: begin
               if (addr >= DATA_LENGTH) begin
						addr <= 8'd0; 
						state <= IDLE;
					end else begin
						//addr <= addr + 1'b1;
						state <= BURST_DATA_SEND;
					end
			end
			
			BURST_DATA_SEND:begin
				if (!busy) begin
					new_data_tx <= 1'b1;
					data_tx <= data;
					addr <= addr + 1'b1;

					state <= BURST_DATA_ADDR;
				end else begin
					new_data_tx <= 1'b0;
					state <= BURST_DATA_SEND;
				end
			end
			
			GET_ADDR: begin
				new_data_tx <= 1'b0;
				data_tx <= 8'h00;
				if(new_data_rx) begin
					addr <= data_rx;
					state <= SEND_DATA;
				end else begin
					state <= GET_ADDR;
				end
			end
			
			SEND_DATA: begin
				new_data_tx <= 1'b0;
				data_tx <= 8'h00;
				if(!busy) begin
					new_data_tx <= 1'b1;
					data_tx <= data;
					state <= IDLE;
				end else begin
					state <= SEND_DATA;
				end
			end
			
		 endcase
	end
end
 
endmodule
