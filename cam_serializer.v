//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:48:44 12/04/2013 
// Design Name: 
// Module Name:    cam_serializer 
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
module cam_serializer(
	input clk,
	input rst,
	input [7:0] ybuss,
	input vsync,
	input href,
	input pclk,
	output reg [7:0] tx_data,
	output reg new_tx_data,
	input tx_busy,
	input [7:0] rx_data,
	input new_rx_data
    );

localparam STATE_SIZE = 1;
localparam IDLE = 0,
				PRINT_PIXEL = 1;
				
reg [STATE_SIZE-1:0] state_d, state_q;

always@(*) begin
	state_d = state_q;
	new_tx_data = 1'b0;
	
	case(state_q)
		IDLE: begin
			if(new_rx_data && rx_data =="h")
				state_d = PRINT_PIXEL;
		end
		
		PRINT_PIXEL: begin
			if(!tx_busy) begin
				new_tx_data = 1'b1;
				tx_data = ybuss;
				state_d = IDLE;
			end
		end
		default: state_d = IDLE;
	endcase
end

always @(posedge clk) begin
	if(rst) begin
		state_q <= IDLE;
	end else begin
		state_q <= state_d;
	end
end

endmodule
