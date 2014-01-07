//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:27:58 12/03/2013 
// Design Name: 
// Module Name:    cam_driver 
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
module cam_driver(
	input clk,
	input start,
	output reg ena,
	output reg [7:0] addr,
	output reg [7:0] data_wr,
	output reg [7:0] sub_addr,
	input [7:0] data_rd,
	output reg rw,
	input ack_err,
	input busy,
	input rst
	);
	
localparam STATE_SIZE = 4;
localparam  idle = 0,
				sending1 = 1,
				wait1 = 2,
				sending2 = 3,
				wait2 = 4,
				sending3 = 5,
				wait3 = 6,
				sending4 = 7,
				wait4 = 8,
				sending5 = 9,
				wait5 = 10,
				sending6 = 11,
				wait6 = 12,
				sending7 = 13,
				wait7 = 14,
				sending8 = 15;				
				
reg [STATE_SIZE:0] state;
integer busy_count = 0;
reg prev_busy = 0;
reg prev_start = 0;
//reg [7:0] addr; 
//reg ena;
//reg [7:0] data_wr;
//reg rw;

always @(posedge comm_clk or posedge rst) begin
	if(rst) begin
		ena <= 0;
		prev_start <= 0;
		state <= idle;
		addr <= 8'h00;
		data_wr<= 8'h00;
		sub_addr<= 8'h00;
	end else begin
		case(state)
			//wait for button press to start the initialization
			idle:begin
				ena <= 0;
				addr<= 8'h00;
				data_wr<= 8'h00;
				sub_addr<= 8'h00;
				if((start == 1) && prev_start == 0)begin
						state <= sending1;
						prev_start <= 1;
				end else begin
					state <= idle;
				end
			end
			//send the clkrc
			sending1:begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h11;
				data_wr <= 8'h04;	//4 gives just over 10 frames per second
				if(busy == 1) begin
					ena <= 0;
					state <= wait1;
				end else begin
					state <= sending1;
				end
			end
			wait1:begin
				if(busy == 0)begin
					state <= sending2;
				end else begin
					state <= wait1;
				end
			end
			//send the comc 
			sending2:begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h14;
				data_wr <= 8'h20;
				if(busy == 1) begin
					ena <= 0;
					state <= wait2;
				end else begin
					state <= sending2;
				end 
			end
			wait2: begin
				if(busy == 0) begin
					state <= sending3;
				end else begin
					state <= wait2;
				end
			end
			//send the coml which sets pclk to only work with href
			sending3: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h39;
				data_wr <= 8'h40;
				if(busy == 1) begin
					ena <= 0;
					state <= wait3;
				end else begin
					state <= sending3;
				end
			end
			wait3: begin
				if(busy == 0) begin
					state <= sending4;
				end else begin
					state <= wait3;
				end
			end
			//send the comh to setup for black/white mode
			sending4: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h28;
				data_wr <= 8'hE0;
				if(busy == 1) begin
					ena <= 0;
					state <= wait4;
				end else begin
					state <= sending4;
				end
			end
			wait4: begin
				if(busy == 0) begin
					state <= sending5;
				end else begin
					state <= wait4;
				end
			end
			//send the hrefst
			sending5: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h17;
				data_wr <= 8'h38;
				if(busy == 1) begin
					ena <= 0;
					state <= wait5;
				end else begin
					state <= sending5;
				end
			end
			wait5: begin
				if(busy == 0) begin
					state <= sending6;
				end else begin
					state <= wait5;
				end
			end
			//send the hrefend
			sending6: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h18;
				data_wr <= 8'h6A;
				if(busy == 1) begin
					ena <= 0;
					state <= wait6;
				end else begin
					state <= sending6;
				end
			end
			wait6: begin
				if(busy == 0) begin
					state <= sending7;
				end else begin
					state <= wait6;
				end
			end
			//send vstrt
			sending7: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h19;
				data_wr <= 8'h03;
				if(busy == 1) begin
					ena <= 0;
					state <= wait7;
				end else begin
					state <= sending7;
				end
			end
			wait7: begin
				if(busy == 0) begin
					state <= sending8;
				end else begin
					state <= wait7;
				end
			end
			//send vend
			sending8: begin
				prev_start <= 1;
				ena <= 1;
				rw <= 0;
				addr<= 8'b11000000;
				sub_addr <=8'h1A;
				data_wr <= 8'h35;
				if(busy == 1) begin
					ena <= 0;
					state <= idle;
				end else begin
					state <= sending8;
				end
			end
		endcase
	end
end
endmodule
