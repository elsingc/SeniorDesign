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
	output reg debug,
	input busy,
	input block, //set to 0
	output reg new_data_tx,
	output reg [7:0] data_tx,
	input new_data_rx,
	input [7:0] data_rx,
	input [7:0] data,
	output reg [7:0] addr,
	input rst,
	input clk
    );

localparam STATE_SIZE = 5;
localparam	IDLE = 0,
				GET_ADDR = 1,
				WAIT_NOT_RX = 2,
				WAIT_ADDR = 3;
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
				if(new_data_rx && data_rx == 8'h04) begin
					state <= GET_ADDR;
				end else begin
					state <= IDLE;
				end
			end
			
			GET_ADDR: begin
				new_data_tx <= 1'b0;
				data_tx <= 8'h00;
				if(new_data_rx) begin
					addr <= data_rx;
					state <= WAIT_ADDR;
				end else begin
					state <= GET_ADDR;
				end
			end
			
			
			WAIT_ADDR: begin
				new_data_tx <= 1'b0;
				data_tx <= 8'h00;
				if(!busy) begin
					new_data_tx <= 1'b1;
					data_tx <= data;
					state <= IDLE;
				end else begin
					state <= WAIT_ADDR;
				end
			end
			
		 endcase
	end
end
 
endmodule
