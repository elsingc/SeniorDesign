module Altimeter_Controller(
	output reg [7:0]debug,
	output reg [23:0] pressure,
	output reg [15:0] temp,
	output reg ena,
	output reg rw,
	output reg [7:0] data_wr,
	input [7:0] data_rd,
   input ready,
   input busy,
   input ack_err,
   input rst,
   input clk,
   output reg start_transfer,
   output reg stop_transfer,
	output reg r_start
     );
    
    //make signals active high
    
	localparam STATE_SIZE = 10;
	reg [STATE_SIZE:0] state = INIT_OSR_START;
	reg [7:0] STATUS_REG = 8'hff;
	localparam 		INIT_OSR_START 					= 0,
						INIT_OSR_START_WAIT 				= 1,
						INIT_OSR_SEND_C0 					= 2,
						INIT_OSR_WAIT_C0 					= 3,
						INIT_OSR_SEND_26 					= 4,
						INIT_OSR_WAIT_26 					= 5,
						INIT_OSR_SEND_B8 					= 6,
						INIT_OSR_WAIT_B8 					= 7, 
						INIT_OSR_STOP 						= 8,
						INIT_OSR_WAIT_STOP 				= 9,
						
						INIT_PT_DATA_CFG_START 			= 10,
						INIT_PT_DATA_CFG_WAIT_START 	= 11,
						INIT_PT_DATA_CFG_SEND_C0 		= 12,
						INIT_PT_DATA_CFG_WAIT_C0 		= 13,
						INIT_PT_DATA_CFG_SEND_13 		= 14,
						INIT_PT_DATA_CFG_WAIT_13 		= 15,
						INIT_PT_DATA_CFG_SEND_07 		= 16,
						INIT_PT_DATA_CFG_WAIT_07 		= 17,
						INIT_PT_DATA_CFG_STOP 			= 18,
						INIT_PT_DATA_CFG_WAIT_STOP 	= 19,
						
						INIT_ACTIVE_START 				= 20,
						INIT_ACTIVE_WAIT_START 			= 21,
						INIT_ACTIVE_SEND_C0 				= 22,
						INIT_ACTIVE_WAIT_C0 				= 23,
						INIT_ACTIVE_SEND_26				= 24,
						INIT_ACTIVE_WAIT_26 				= 25,
						INIT_ACTIVE_SEND_B9 				= 26,
						INIT_ACTIVE_WAIT_B9 				= 27,
						INIT_ACTIVE_STOP 					= 28,
						INIT_ACTIVE_WAIT_STOP 			= 29,
						
						READ_STATUS_START 				= 30,
						READ_STATUS_WAIT_START 			= 31,
						READ_STATUS_SEND_C0 				= 32,
						READ_STATUS_WAIT_C0 				= 33,
						READ_STATUS_SEND_00 				= 34,
						READ_STATUS_WAIT_00 				= 35,
						READ_STATUS_RSTART 				= 36,
						READ_STATUS_WAIT_RSTART 		= 37,
						READ_STATUS_SEND_C1 				= 38,
						READ_STATUS_WAIT_C1 				= 39,
						READ_STATUS_READ 					= 40,
						READ_STATUS_WAIT_READ 			= 41,
						READ_STATUS_STOP 					= 42,
						READ_STATUS_WAIT_STOP 			= 43,
						READ_STATUS_CONTINUE 			= 44,
						
						OUT_P_MSB_START					= 45,
						OUT_P_MSB_WAIT_START 			= 46,
						OUT_P_MSB_SEND_C0 				= 47,
						OUT_P_MSB_WAIT_C0 				= 48,
						OUT_P_MSB_SEND_01 				= 49,
						OUT_P_MSB_WAIT_01 				= 50,
						OUT_P_MSB_RSTART 					= 51,
						OUT_P_MSB_WAIT_RSTART 			= 52,
						OUT_P_MSB_SEND_C1 				= 53,
						OUT_P_MSB_WAIT_C1 				= 54,
						OUT_P_MSB_READ 					= 55,
						OUT_P_MSB_WAIT_READ 				= 56,
						OUT_P_MSB_STOP 					= 57,
						OUT_P_MSB_WAIT_STOP 				= 58,
						
						OUT_P_CSB_START					= 59,
						OUT_P_CSB_WAIT_START 			= 60,
						OUT_P_CSB_SEND_C0 				= 61,
						OUT_P_CSB_WAIT_C0 				= 62,
						OUT_P_CSB_SEND_02 				= 63,
						OUT_P_CSB_WAIT_02 				= 64,
						OUT_P_CSB_RSTART 					= 65,
						OUT_P_CSB_WAIT_RSTART	 		= 66,
						OUT_P_CSB_SEND_C1 				= 67,
						OUT_P_CSB_WAIT_C1 				= 68,
						OUT_P_CSB_READ 					= 69,
						OUT_P_CSB_WAIT_READ 				= 70,
						OUT_P_CSB_STOP 					= 71,
						OUT_P_CSB_WAIT_STOP 				= 72,
						
						OUT_P_LSB_START					= 73,
						OUT_P_LSB_WAIT_START 			= 74,
						OUT_P_LSB_SEND_C0 				= 75,
						OUT_P_LSB_WAIT_C0 				= 76,
						OUT_P_LSB_SEND_03 				= 77,
						OUT_P_LSB_WAIT_03 				= 78,
						OUT_P_LSB_RSTART 					= 79,
						OUT_P_LSB_WAIT_RSTART	 		= 80,
						OUT_P_LSB_SEND_C1 				= 81,
						OUT_P_LSB_WAIT_C1 				= 82,
						OUT_P_LSB_READ 					= 83,
						OUT_P_LSB_WAIT_READ 				= 84,
						OUT_P_LSB_STOP 					= 85,
						OUT_P_LSB_WAIT_STOP 				= 86,
						
						OUT_T_MSB_START					= 87,
						OUT_T_MSB_WAIT_START 			= 88,
						OUT_T_MSB_SEND_C0 				= 89,
						OUT_T_MSB_WAIT_C0 				= 90,
						OUT_T_MSB_SEND_04 				= 91,
						OUT_T_MSB_WAIT_04 				= 92,
						OUT_T_MSB_RSTART 					= 93,
						OUT_T_MSB_WAIT_RSTART	 		= 94,
						OUT_T_MSB_SEND_C1 				= 95,
						OUT_T_MSB_WAIT_C1 				= 96,
						OUT_T_MSB_READ 					= 97,
						OUT_T_MSB_WAIT_READ 				= 98,
						OUT_T_MSB_STOP 					= 99,
						OUT_T_MSB_WAIT_STOP 				= 100,
						
						OUT_T_LSB_START					= 101,
						OUT_T_LSB_WAIT_START 			= 102,
						OUT_T_LSB_SEND_C0 				= 103,
						OUT_T_LSB_WAIT_C0 				= 104,
						OUT_T_LSB_SEND_05 				= 105,
						OUT_T_LSB_WAIT_05 				= 106,
						OUT_T_LSB_RSTART 					= 107,
						OUT_T_LSB_WAIT_RSTART	 		= 108,
						OUT_T_LSB_SEND_C1 				= 109,
						OUT_T_LSB_WAIT_C1 				= 110,
						OUT_T_LSB_READ 					= 111,
						OUT_T_LSB_WAIT_READ 				= 112,
						OUT_T_LSB_STOP 					= 113,
						OUT_T_LSB_WAIT_STOP 				= 114;
						 
	
 
always @(posedge clk or posedge rst) begin
	if(rst)begin
		state <= INIT_OSR_START;
		ena <= 1'b0;
		data_wr <= 8'h00;
		rw <= 1'b0;
		r_start <= 1'b0;
		start_transfer <= 1'b0;
		stop_transfer <= 1'b0;
		STATUS_REG <= 8'hff;
	end else begin
		pressure <= pressure;
		temp <= temp;
		case(state)
	//====================INIT_OSR==========================================
			INIT_OSR_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= INIT_OSR_START_WAIT;
				end else begin
					state <= INIT_OSR_START;
				end 
			end
			
			INIT_OSR_START_WAIT:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= INIT_OSR_SEND_C0;
				end else begin
					state <= INIT_OSR_START_WAIT;
				end
			end
			
			INIT_OSR_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_OSR_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_OSR_WAIT_C0;
				end
			end
			
			INIT_OSR_WAIT_C0: begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= INIT_OSR_SEND_26;
				end else begin
					state <= INIT_OSR_WAIT_C0;
				end
			end
				
			INIT_OSR_SEND_26:begin
				ena <= 1'b1;
				data_wr <= 8'h26;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_OSR_SEND_26;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_OSR_WAIT_26;
				end
			end 				
			
			INIT_OSR_WAIT_26: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_OSR_SEND_B8;
					end else begin
						state <= INIT_OSR_WAIT_26;
					end
				end 				
			
			INIT_OSR_SEND_B8:begin
				ena <= 1'b1;
				data_wr <= 8'hB8;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_OSR_SEND_B8;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_OSR_WAIT_B8;
				end
			end 				
			
			INIT_OSR_WAIT_B8: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_OSR_STOP;
					end else begin
						state <= INIT_OSR_WAIT_B8;
					end
				end 				
			
			INIT_OSR_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_OSR_STOP;
					end else begin
						state <= INIT_OSR_WAIT_STOP;
					end
				end 					
			
			INIT_OSR_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_PT_DATA_CFG_START;
					end else begin
						state <= INIT_OSR_WAIT_STOP;
					end
				end
			
	//====================INIT_PT_DATA======================================
		INIT_PT_DATA_CFG_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= INIT_PT_DATA_CFG_WAIT_START;
				end else begin
					state <= INIT_PT_DATA_CFG_START;
				end 
			end 
			
			INIT_PT_DATA_CFG_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= INIT_PT_DATA_CFG_SEND_C0;
				end else begin
					state <= INIT_PT_DATA_CFG_WAIT_START;
				end
			end
			
			INIT_PT_DATA_CFG_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_PT_DATA_CFG_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_PT_DATA_CFG_WAIT_C0;
				end
			end 	
			
			INIT_PT_DATA_CFG_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_PT_DATA_CFG_SEND_13;
					end else begin
						state <= INIT_PT_DATA_CFG_WAIT_C0;
					end
				end 
			
			INIT_PT_DATA_CFG_SEND_13:begin
				ena <= 1'b1;
				data_wr <= 8'h13;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_PT_DATA_CFG_SEND_13;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_PT_DATA_CFG_WAIT_13;
				end
			end
			
			INIT_PT_DATA_CFG_WAIT_13: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_PT_DATA_CFG_SEND_07;
					end else begin
						state <= INIT_PT_DATA_CFG_WAIT_13;
					end
				end 	
			
			INIT_PT_DATA_CFG_SEND_07:begin
				ena <= 1'b1;
				data_wr <= 8'h07;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_PT_DATA_CFG_SEND_07;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_PT_DATA_CFG_WAIT_07;
				end
			end
			
			INIT_PT_DATA_CFG_WAIT_07: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_PT_DATA_CFG_STOP;
					end else begin
						state <= INIT_PT_DATA_CFG_WAIT_07;
					end
				end	
			
			INIT_PT_DATA_CFG_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_PT_DATA_CFG_STOP;
					end else begin
						state <= INIT_PT_DATA_CFG_WAIT_STOP;
					end
				end 		
			
			INIT_PT_DATA_CFG_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_ACTIVE_START;
					end else begin
						state <= INIT_PT_DATA_CFG_WAIT_STOP;
					end
				end
			
	//====================INIT_ACTIVE=======================================
			INIT_ACTIVE_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= INIT_ACTIVE_WAIT_START;
				end else begin
					state <= INIT_ACTIVE_START;
				end 
			end 		
			
			INIT_ACTIVE_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= INIT_ACTIVE_SEND_C0;
				end else begin
					state <= INIT_ACTIVE_WAIT_START;
				end
			end
			
			INIT_ACTIVE_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_ACTIVE_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_ACTIVE_WAIT_C0;
				end
			end
			
			INIT_ACTIVE_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_ACTIVE_SEND_26;
					end else begin
						state <= INIT_ACTIVE_WAIT_C0;
					end
				end
			
			INIT_ACTIVE_SEND_26:begin
				ena <= 1'b1;
				data_wr <= 8'h26;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_ACTIVE_SEND_26;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_ACTIVE_WAIT_26;
				end
			end
			
			INIT_ACTIVE_WAIT_26: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_ACTIVE_SEND_B9;
					end else begin
						state <= INIT_ACTIVE_WAIT_26;
					end
				end 	
			
			INIT_ACTIVE_SEND_B9:begin
				ena <= 1'b1;
				data_wr <= 8'hB9;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= INIT_ACTIVE_SEND_B9;
				end else begin
					start_transfer <= 1'b0;
					state <= INIT_ACTIVE_WAIT_B9;
				end
			end
			
			INIT_ACTIVE_WAIT_B9: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_ACTIVE_STOP;
					end else begin
						state <= INIT_ACTIVE_WAIT_B9;
					end
				end 
			
			INIT_ACTIVE_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= INIT_ACTIVE_STOP;
					end else begin
						state <= INIT_ACTIVE_WAIT_STOP;
					end
				end 	
			
			INIT_ACTIVE_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_START; 
					end else begin
						state <= INIT_ACTIVE_WAIT_STOP;
					end
				end		
			
	//====================READ_STATUS=======================================	
			READ_STATUS_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= READ_STATUS_WAIT_START;
				end else begin
					state <= READ_STATUS_START;
				end 
			end			
			
			READ_STATUS_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= READ_STATUS_SEND_C0;
				end else begin
					state <= READ_STATUS_WAIT_START;
				end
			end
			
			READ_STATUS_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= READ_STATUS_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= READ_STATUS_WAIT_C0;
				end
			end
			
			READ_STATUS_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_SEND_00;
					end else begin
						state <= READ_STATUS_WAIT_C0;
					end
				end 
			
			READ_STATUS_SEND_00:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= READ_STATUS_SEND_00;
				end else begin
					start_transfer <= 1'b0;
					state <= READ_STATUS_WAIT_00;
				end
			end
			
			READ_STATUS_WAIT_00: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_RSTART;
					end else begin
						state <= READ_STATUS_WAIT_00;
					end
				end
			
			READ_STATUS_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= READ_STATUS_RSTART;
					end else begin
						state <= READ_STATUS_WAIT_RSTART;
					end
				end
			
			READ_STATUS_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_SEND_C1;
					end else begin
						state <= READ_STATUS_WAIT_RSTART;
					end
				end
			
			READ_STATUS_SEND_C1:begin
				ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= READ_STATUS_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= READ_STATUS_WAIT_C1;
				end
			end
			
			READ_STATUS_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_READ;
					end else begin
						state <= READ_STATUS_WAIT_C1;
					end
				end
			
			READ_STATUS_READ: begin
				ena <= 1'b1;
				data_wr <= 8'hFF;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= READ_STATUS_READ;
				end else begin
					start_transfer <= 1'b0;
					state <= READ_STATUS_WAIT_READ;
				end
			end 
			
			READ_STATUS_WAIT_READ: begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					STATUS_REG <= data_rd;
					state <= READ_STATUS_STOP;
				end else begin
					state <= READ_STATUS_WAIT_READ;
				end
			end
			
			READ_STATUS_STOP: begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= READ_STATUS_STOP;
				end else begin
					state <= READ_STATUS_WAIT_STOP;
				end
			end
			
			READ_STATUS_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_CONTINUE;
					end else begin
						state <= READ_STATUS_WAIT_STOP;
					end
				end
			
			READ_STATUS_CONTINUE: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if((STATUS_REG[2] == 1'b1) || (STATUS_REG[3] == 1'b1)) begin
						state <= OUT_P_MSB_START;   // OUT_P_MSB_START
					end else begin
						state <= READ_STATUS_START;		// READ_STATUS_START
					end
				end 		
			
	//====================OUT_P_MSB=========================================
			OUT_P_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= OUT_P_MSB_WAIT_START;
				end else begin
					state <= OUT_P_MSB_START;
				end 
			end	
			
			OUT_P_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= OUT_P_MSB_SEND_C0;
				end else begin
					state <= OUT_P_MSB_WAIT_START;
				end
			end
			
			OUT_P_MSB_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_MSB_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_MSB_WAIT_C0;
				end
			end
			
			OUT_P_MSB_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_MSB_SEND_01;
					end else begin
						state <= OUT_P_MSB_WAIT_C0;
					end
				end
			
			OUT_P_MSB_SEND_01:begin
				ena <= 1'b1;
				data_wr <= 8'h01;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_MSB_SEND_01;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_MSB_WAIT_01;
				end
			end
			
			OUT_P_MSB_WAIT_01: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_MSB_RSTART;
					end else begin
						state <= OUT_P_MSB_WAIT_01;
					end
				end
			
			OUT_P_MSB_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= OUT_P_MSB_RSTART;
					end else begin
						state <= OUT_P_MSB_WAIT_RSTART;
					end
				end
			
			OUT_P_MSB_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_MSB_SEND_C1;
					end else begin
						state <= OUT_P_MSB_WAIT_RSTART;
					end
				end
			
			OUT_P_MSB_SEND_C1:begin
				ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_MSB_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_MSB_WAIT_C1;
				end
			end
			
			OUT_P_MSB_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_MSB_READ;
					end else begin
						state <= OUT_P_MSB_WAIT_C1;
					end
				end
			
			OUT_P_MSB_READ: begin
					ena <= 1'b1;
					data_wr <= 8'hff;
					rw <= 1'b1;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						start_transfer <= 1'b1;
						state <= OUT_P_MSB_READ;
					end else begin
						start_transfer <= 1'b0;
						state <= OUT_P_MSB_WAIT_READ;
					end
				end 
				
			
			OUT_P_MSB_WAIT_READ: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b1;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						pressure[23:16] <= data_rd;
						state <= OUT_P_MSB_STOP;
					end else begin
						state <= OUT_P_MSB_WAIT_READ;
					end
				end
			
			OUT_P_MSB_STOP: begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= OUT_P_MSB_STOP;
				end else begin
					state <= OUT_P_MSB_WAIT_STOP;
				end
			end
			
			OUT_P_MSB_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_START;
					end else begin
						state <= OUT_P_MSB_WAIT_STOP;
					end
				end 			

	//====================OUT_P_CSB=========================================
			OUT_P_CSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= OUT_P_CSB_WAIT_START;
				end else begin
					state <= OUT_P_CSB_START;
				end 
			end	
			
			OUT_P_CSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= OUT_P_CSB_SEND_C0;
				end else begin
					state <= OUT_P_CSB_WAIT_START;
				end
			end
			
			OUT_P_CSB_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_CSB_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_CSB_WAIT_C0;
				end
			end
			
			OUT_P_CSB_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_SEND_02;
					end else begin
						state <= OUT_P_CSB_WAIT_C0;
					end
				end
			
			OUT_P_CSB_SEND_02:begin
				ena <= 1'b1;
				data_wr <= 8'h02;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_CSB_SEND_02;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_CSB_WAIT_02;
				end
			end
			
			OUT_P_CSB_WAIT_02: begin
					ena <= 1'b1;
					data_wr <= 8'h02;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_RSTART;
					end else begin
						state <= OUT_P_CSB_WAIT_02;
					end
				end
			
			OUT_P_CSB_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= OUT_P_CSB_RSTART;
					end else begin
						state <= OUT_P_CSB_WAIT_RSTART;
					end
				end 		
			
			OUT_P_CSB_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_SEND_C1;
					end else begin
						state <= OUT_P_CSB_WAIT_RSTART;
					end
				end	
			
			OUT_P_CSB_SEND_C1:begin
				ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_CSB_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_CSB_WAIT_C1;
				end
			end
			
			OUT_P_CSB_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_READ;
					end else begin
						state <= OUT_P_CSB_WAIT_C1;
					end
				end
			
			OUT_P_CSB_READ: begin
					ena <= 1'b1;
					data_wr <= 8'hff;
					rw <= 1'b1;
					start_transfer <= 1'b1;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_READ;
					end else begin
						state <= OUT_P_CSB_WAIT_READ;
					end
				end 
			
			OUT_P_CSB_WAIT_READ: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b1;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						pressure[15:8] <= data_rd;
						state <= OUT_P_CSB_STOP;
					end else begin
						state <= OUT_P_CSB_WAIT_READ;
					end
				end
			
			OUT_P_CSB_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_CSB_STOP;
					end else begin
						state <= OUT_P_CSB_WAIT_STOP;
					end
				end 
			
			
			OUT_P_CSB_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_START;
					end else begin
						state <= OUT_P_CSB_WAIT_STOP;
					end
				end
		
	//====================OUT_P_LSB=========================================
			OUT_P_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= OUT_P_LSB_WAIT_START;
				end else begin
					state <= OUT_P_LSB_START;
				end 
			end	
			
			OUT_P_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= OUT_P_LSB_SEND_C0;
				end else begin
					state <= OUT_P_LSB_WAIT_START;
				end
			end
			
			OUT_P_LSB_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_LSB_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_LSB_WAIT_C0;
				end
			end
			
			OUT_P_LSB_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_SEND_03;
					end else begin
						state <= OUT_P_LSB_WAIT_C0;
					end
				end
			
			OUT_P_LSB_SEND_03:begin
				ena <= 1'b1;
				data_wr <= 8'h03;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_LSB_SEND_03;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_LSB_WAIT_03;
				end
			end
			
			OUT_P_LSB_WAIT_03: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_RSTART;
					end else begin
						state <= OUT_P_LSB_WAIT_03;
					end
				end
			
			OUT_P_LSB_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= OUT_P_LSB_RSTART;
					end else begin
						state <= OUT_P_LSB_WAIT_RSTART;
					end
				end
			
			OUT_P_LSB_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_SEND_C1;
					end else begin
						state <= OUT_P_LSB_WAIT_RSTART;
					end
				end
			
			OUT_P_LSB_SEND_C1:begin
				ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_P_LSB_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_P_LSB_WAIT_C1;
				end
			end
			
			OUT_P_LSB_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_READ;
					end else begin
						state <= OUT_P_LSB_WAIT_C1;
					end
				end
			
			OUT_P_LSB_READ: begin
					ena <= 1'b1;
					data_wr <= 8'hff;
					rw <= 1'b1;
					start_transfer <= 1'b1;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_READ;
					end else begin
						state <= OUT_P_LSB_WAIT_READ;
					end
				end  	
			
			OUT_P_LSB_WAIT_READ: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b1;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						pressure[7:0] <= data_rd[7:0];
						state <= OUT_P_LSB_STOP;
					end else begin
						//pressure[15:8] <= data_rd;
						state <= OUT_P_LSB_WAIT_READ;
					end
				end
			
			OUT_P_LSB_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_P_LSB_STOP;
					end else begin
						state <= OUT_P_LSB_WAIT_STOP;
					end
				end 	
			
			OUT_P_LSB_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_START;
					end else begin
						state <= OUT_P_LSB_WAIT_STOP;
					end
				end			
			
	//====================OUT_T_MSB=========================================
			OUT_T_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= OUT_T_MSB_WAIT_START;
				end else begin
					state <= OUT_T_MSB_START;
				end 
			end		
			
			OUT_T_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= OUT_T_MSB_SEND_C0;
				end else begin
					state <= OUT_T_MSB_WAIT_START;
				end
			end
			
			OUT_T_MSB_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_MSB_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_MSB_WAIT_C0;
				end
			end
			
			OUT_T_MSB_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_SEND_04;
					end else begin
						state <= OUT_T_MSB_WAIT_C0;
					end
				end 			

			OUT_T_MSB_SEND_04:begin
				ena <= 1'b1;
				data_wr <= 8'h04;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_MSB_SEND_04;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_MSB_WAIT_04;
				end
			end 			

			OUT_T_MSB_WAIT_04: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_RSTART;
					end else begin
						state <= OUT_T_MSB_WAIT_04;
					end
				end 			

			OUT_T_MSB_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= OUT_T_MSB_RSTART;
					end else begin
						state <= OUT_T_MSB_WAIT_RSTART;
					end
				end 				

			OUT_T_MSB_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_SEND_C1;
					end else begin
						state <= OUT_T_MSB_WAIT_RSTART;
					end
				end	 	

			OUT_T_MSB_SEND_C1:begin
				ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_MSB_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_MSB_WAIT_C1;
				end
			end 			

			OUT_T_MSB_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_READ;
					end else begin
						state <= OUT_T_MSB_WAIT_C1;
					end
				end 			

			OUT_T_MSB_READ: begin
					ena <= 1'b1;
					data_wr <= 8'hff;
					rw <= 1'b1;
					start_transfer <= 1'b1;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_READ;
					end else begin
						state <= OUT_T_MSB_WAIT_READ;
					end
				end  				

			OUT_T_MSB_WAIT_READ: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b1;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						temp[15:8] <= data_rd;
						state <= OUT_T_MSB_STOP;
					end else begin
						state <= OUT_T_MSB_WAIT_READ;
					end
				end 			

			OUT_T_MSB_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_MSB_STOP;
					end else begin
						state <= OUT_T_MSB_WAIT_STOP;
					end
				end 				

			OUT_T_MSB_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_START;
					end else begin
						state <= OUT_T_MSB_WAIT_STOP;
					end
				end 			
			
	//====================OUT_T_LSB=========================================
			OUT_T_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state <= OUT_T_LSB_WAIT_START;
				end else begin
					state <= OUT_T_LSB_START;
				end 
			end	
			
			OUT_T_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'hA0;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(ready && !busy) begin
					state <= OUT_T_LSB_SEND_C0;
				end else begin
					state <= OUT_T_LSB_WAIT_START;
				end
			end
			
			OUT_T_LSB_SEND_C0:begin
				ena <= 1'b1;
				data_wr <= 8'hC0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_LSB_SEND_C0;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_LSB_WAIT_C0;
				end
			end			
		
			OUT_T_LSB_WAIT_C0: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_SEND_05;
					end else begin
						state <= OUT_T_LSB_WAIT_C0;
					end
				end 			

			OUT_T_LSB_SEND_05:begin
				ena <= 1'b1;
				data_wr <= 8'h05;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_LSB_SEND_05;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_LSB_WAIT_05;
				end
			end			
			
			OUT_T_LSB_WAIT_05: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_RSTART;
					end else begin
						state <= OUT_T_LSB_WAIT_05;
					end
				end 			
			
			OUT_T_LSB_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b1;
					if(!busy && ready) begin
						state <= OUT_T_LSB_RSTART;
					end else begin
						state <= OUT_T_LSB_WAIT_RSTART;
					end
				end 				
			
			OUT_T_LSB_WAIT_RSTART: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_SEND_C1;
					end else begin
						state <= OUT_T_LSB_WAIT_RSTART;
					end
				end	 	
			
			OUT_T_LSB_SEND_C1:begin
			ena <= 1'b1;
				data_wr <= 8'hC1;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= OUT_T_LSB_SEND_C1;
				end else begin
					start_transfer <= 1'b0;
					state <= OUT_T_LSB_WAIT_C1;
				end
			end			

			OUT_T_LSB_WAIT_C1: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_READ;
					end else begin
						state <= OUT_T_LSB_WAIT_C1;
					end
				end 			

			OUT_T_LSB_READ: begin
					ena <= 1'b1;
					data_wr <= 8'hff;
					rw <= 1'b1;
					start_transfer <= 1'b1;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_READ;
					end else begin
						state <= OUT_T_LSB_WAIT_READ;
					end
				end  				

			OUT_T_LSB_WAIT_READ: begin
					ena <= 1'b1;
					data_wr <= 8'h00;
					rw <= 1'b1;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						temp[7:0] <= data_rd;
						state <= OUT_T_LSB_STOP;
					end else begin
						state <= OUT_T_LSB_WAIT_READ;
					end
				end 			

			OUT_T_LSB_STOP: begin
					start_transfer <= 1'b0;
					stop_transfer <= 1'b1;
					r_start <= 1'b0;
					if(!busy) begin
						state <= OUT_T_LSB_STOP;
					end else begin
						state <= OUT_T_LSB_WAIT_STOP;
					end
				end 				

			OUT_T_LSB_WAIT_STOP: begin
					ena <= 1'b0;
					data_wr <= 8'h00;
					rw <= 1'b0;
					start_transfer <= 1'b0;
					stop_transfer <= 1'b0;
					r_start <= 1'b0;
					if(!busy) begin
						state <= READ_STATUS_START;
					end else begin
						state <= OUT_T_LSB_WAIT_STOP;
					end
				end 		
				
	//====================END===============================================		
		 endcase
	 end
 end
 
 endmodule