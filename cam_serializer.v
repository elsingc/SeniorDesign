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
	input new_rx_data,
	output reg start_config
    );
	 
reg [15:0] addrW;
reg [15:0] addrR;
reg [7:0] din;
wire [7:0] dout;
 
//reg [7:0] ram [2499:0];

reg wea, SAVE;

reg [11:0] x = 12'd0;
reg [11:0] y = 12'd0;

reg go, start, old_start, rstb;
reg [11:0] count_p, send_p;

/*
framebuffer framebuffer(
    .clk(clk),
    .we(wea),
    .addrW(count_p),
    .din(din),
    .addrR(send_p),
    .dout(dout)
);
*/

framebuffer framebuffer(
    .clka(~clk),
    .wea(wea),
    .addra(count_p),
    .dina(din),
    .clkb(~clk), 
	 .rstb(rstb),
    .addrb(send_p),
    .doutb(dout) 
);

localparam STATE_SIZE = 4; 
localparam IDLE = 0,
				PRINT_PIXEL = 1,
				SEND_PIXEL = 2,
				WAIT_NOT_BUSY = 3,
				UPDATE_P_COUNT = 4;				
reg [STATE_SIZE:0] state = IDLE;


localparam PIXEL_STATE_SIZE = 2; 
localparam GET_PIXEL = 0,
				LATCH_PIXEL = 1,
				UPDATE_P_READ = 2;				
reg [PIXEL_STATE_SIZE:0] pixel_state = GET_PIXEL;


localparam HREF_STATE_SIZE = 2; 
localparam WAIT_HREF = 0,
				UPDATE_HREF = 1;				
reg [HREF_STATE_SIZE:0] href_state = WAIT_HREF;

localparam VSYNC_STATE_SIZE = 2; 
localparam WAIT_VSYNC = 0,
				UPDATE_VSYNC = 1;				
reg [VSYNC_STATE_SIZE:0] vsync_state = WAIT_VSYNC;


//assign busy_v = busy;

always@(posedge clk or posedge rst) begin
	if(rst) begin
		state <= IDLE;
		start <= 1'b0;
		new_tx_data <= 1'b0;
		tx_data <= 8'h00;
	end else begin
		case(state)
			IDLE: begin
				new_tx_data <= 0;
				tx_data <= 8'h00;
				rstb <= 0;
				if(new_rx_data && (rx_data == "r" || rx_data == "R")) begin
					state <= PRINT_PIXEL;
				end else if(new_rx_data && (rx_data =="s" || rx_data == "S")) begin
					state <= SEND_PIXEL;
					send_p <= 1'b0;
				end else if (new_rx_data && (rx_data == "c" || rx_data == "C")) begin
					start_config <= 1'b1;
					state <= IDLE;
				end else begin
					state <= IDLE;
				end
			end
			
			PRINT_PIXEL: begin
				start <= ~start;
				state <= IDLE;
				rstb <= 1;
			end
			
			
			SEND_PIXEL: begin
				addrR <= send_p;
				tx_data <= dout;
				new_tx_data <= 1'b0;
				state <= WAIT_NOT_BUSY;
			end
			
			WAIT_NOT_BUSY: begin
				new_tx_data <= 1'b0;
				if(tx_busy) begin
					state <= WAIT_NOT_BUSY;
				end else begin
					state <= UPDATE_P_COUNT;
					new_tx_data <= 1'd1;
				end
			end
			
			UPDATE_P_COUNT:begin
				new_tx_data <= 1'b0;
				
				if(addrR < 12'd2499) begin
					send_p <= send_p + 12'b1;
					state<= SEND_PIXEL;
				end else begin
					state <= IDLE;
					send_p <= 12'd0;
					new_tx_data <= 1'b0;
				end
			end
		endcase
	end
end

//initial begin
//		old_start <= start;
//		busy_v <= 0;
//		wea <= 0;
//		count_p <= 0;
//		din <= 0;
//		addrW<= 0;
//	end
	
/*
always @(posedge clk)begin
	if(vsync) begin
		y <= 12'd0;
	end else begin
		case(href_state)
			WAIT_HREF:begin
				if(!href)begin
					href_state <= UPDATE_HREF;
				end else begin
					href_state <= WAIT_HREF;
				end
			end
			
			UPDATE_HREF: begin
				if(href) begin
					y <= y + 12'd50;
					href_state <= WAIT_HREF;
				end else begin
					href_state <= UPDATE_HREF;
				end
			end
			default: begin
				y <= 12'd0;
				href_state <= WAIT_HREF;
			end
		endcase
	end
end
*/
always @(posedge clk) begin
	case(vsync_state)
		WAIT_VSYNC:begin
			if(vsync)begin
				y <= 12'd0;
				x <= 12'd0;
				go <= 0;
				if(old_start != start)begin
					go <= 1;
					y <= 12'd0;
					x <= 12'd0;
					old_start <= start;
					vsync_state <= UPDATE_VSYNC;
				end				
			end else begin
				vsync_state <= WAIT_VSYNC;
			end
		end
		UPDATE_VSYNC: begin
			if(!vsync) begin
				y <= 12'd0;
				x <= 12'd0;
				vsync_state <= WAIT_VSYNC;			
			end else begin
				vsync_state <= UPDATE_VSYNC;
			end
		end
		default: begin
			vsync_state <= WAIT_VSYNC;
		end
	endcase
	
	case(href_state)
		WAIT_HREF:begin
			if(href && go)begin
				x <= 12'd0;
				href_state <= UPDATE_HREF;
			end else begin
				href_state <= WAIT_HREF;
			end
		end
		UPDATE_HREF: begin
			if(!href && go) begin
				y <= y + 12'd50;
				x <= 12'd0;
				href_state <= WAIT_HREF;			
			end else begin 
				href_state <= UPDATE_HREF;
			end
		end
		default: begin
			y <= 12'd0;
			href_state <= WAIT_HREF;
		end
	endcase

	case(pixel_state)
		GET_PIXEL:begin
			wea <= 1'd1;
			if(pclk && go)begin
				din <= ybuss;
				count_p <= x + y;
				pixel_state <= UPDATE_P_READ;//LATCH_PIXEL;
			end else begin
				wea <= 1'd0;
				pixel_state <= GET_PIXEL;
			end
		end
		UPDATE_P_READ: begin
			wea <= 1'd0;
			if(!pclk && go) begin
				x <= x + 12'd1;
				pixel_state <= GET_PIXEL;
			end else begin
				pixel_state <= UPDATE_P_READ;
			end
		end
	endcase
end

endmodule
