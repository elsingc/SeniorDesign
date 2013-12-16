module I2C_Driver(
	input clk,
	input rst,
	input ena,
	input [7:0] addr,
	input rw,
	input [7:0]data_wr,
	input [7:0]sub_addr,
	output reg busy,
	output reg [7:0]data_rd,
	output reg ack_err,
	inout SDA,
	inout SCL
	//inout SCL
   );
	
	
	
localparam STATE_SIZE = 5;
localparam clock_speed = 50000000;
localparam com_speed = 1000;//00;
localparam divider =(clock_speed/com_speed)/4;



localparam 	ready = 0,
				start = 1,
				device_addr_send = 2,
				slv_ack1 = 3,
				sub_addr_send = 4,
				slv_ack2 = 5,
				wr = 6,
				rd = 7,
				slv_ack3 = 8,
				mstr_ack = 9,
				stop = 10;


reg [STATE_SIZE:0] state;

//integer bitcount = 7;
reg [3:0] bitcount;

//integer count = 0;
reg [16:0] count;

reg data_clk;
reg scl_clk;
reg scl_ena;
reg sda_int;
reg sda_ena_n;
reg [7:0] addr_rw;
reg [7:0] data_tx = 8'h00;
reg [7:0] data_rx = 8'h00;
reg [7:0] sub_addr_tx = 8'h00;
reg stretch;

assign SDA = sda_ena_n == 1'b0 ? 1'b0 : 1'bz;
assign SCL = scl_ena == 1'b1 ? scl_clk : 1'bz;
//assign toggle = data_clk;


always @(*) begin
	if(state == start) begin
		sda_ena_n <= data_clk;
	end else if(state == stop) begin
		sda_ena_n <= ~data_clk;
	end else begin
		sda_ena_n <= sda_int;
	end
end

always @(posedge clk or posedge rst)begin
	if(rst) begin
		stretch <= 0;
		count <= 0;
	end else begin
		if(count ==((divider*4)-1)) begin
			count <= 0;
		end else if(stretch == 0) begin
			count <= count + 1;
		end
		if((count >= 0) && (count < (divider))) begin
			scl_clk <= 0;
			data_clk <= 0;
		end else if((count >= divider) && (count < (divider*2))) begin
			scl_clk <= 0;
			data_clk <= 1;
		end else if((count >= (divider*2)) && (count <=divider*3)) begin
			scl_clk <= 1'bZ;
			if(SCL == 0)begin
				stretch <= 1;
			end else begin
				stretch <= 0;
			end
			data_clk <= 1;
		end else begin
			scl_clk <= 1'bz;
			data_clk <= 0;
		end
	end
end

always @(posedge data_clk or posedge rst)begin
	if(rst) begin
		state <= ready;
		busy <= 1'b1;
		scl_ena <= 1'b0;
		sda_int <= 1;
		bitcount <= 7;
		data_rd <= 8'b0;;
	end else begin// if(data_clk == 1'b1) begin
		case (state)
			ready: begin
				if(ena==1'b1)begin
					busy <= 1'b1;
					addr_rw <= addr;//&rw;
					data_tx <= data_wr;
					sub_addr_tx <= sub_addr;
					state <= start;
					bitcount <= 7;
				end else begin
					busy <= 1'b0;
					state <= ready;
				end
			end
			
			start: begin
				busy <= 1'b1;
				scl_ena <= 1'b1;
				sda_int <= 1;// addr_rw[bitcount];
				state <= device_addr_send;
			end
			
			device_addr_send: begin
				busy <= 1'b1;
				if(bitcount == 0) begin
					sda_int <= 1'b1;
					bitcount <= 7;
					state <= slv_ack1;
				end else begin
					sda_int <= addr_rw[bitcount-1];
					bitcount <= bitcount - 1;
					state <= device_addr_send;
				end
			end			
			
			slv_ack1: begin
				busy <= 1'b1;
				sda_int <= 0;//data_tx[bitcount];
				state <= sub_addr_send;
			end
			
			sub_addr_send:begin
				busy <= 1'b1;
				if(bitcount == 0) begin
					sda_int <= 1'b1;
					bitcount <= 7;
					state <= slv_ack2;
				end else begin
					sda_int <= sub_addr_tx[bitcount-1];
					bitcount <= bitcount - 1;
					state <= sub_addr_send;
				end
			end
			
			slv_ack2:begin
				busy <= 1'b1;
				if(rw == 1'b1) begin 
					state <= rd;
				end else begin 
					sda_int <= data_wr[bitcount];
					state <= wr;
				end
			end
			
			wr: begin
				busy <= 1'b1;
				if(bitcount == 0) begin
					sda_int <= 1'b1;
					bitcount <= 7; 
					state <= slv_ack3;
				end else begin
					bitcount <= bitcount - 1;
					sda_int <= data_tx[bitcount-1]; //<---------might be wrong, bitcound -1 might be right
					state <= wr;
				end
			end
			
			rd: begin
				busy <= 1;
				if(bitcount == 0) begin
					if(rw == 1'b1) begin
						sda_int <= 1'b0;	
					end else begin
						sda_int <= 1'b1;
					end
					bitcount <= 7;
					data_rd <= data_rx;
					state <= mstr_ack;
				end else begin
					bitcount <= bitcount - 1;;
					state <= rd;
				end
			end
			
			slv_ack3: begin
				busy <= 1'b1;
				scl_ena <= 1'b0;
				state <= stop;
			end
			
			mstr_ack: begin
				busy <= 1'b1;
				scl_ena <= 1'b0;
				state <= stop;
			end 
			
			stop: begin
				busy <= 1'b0;
				state <= ready;
			end
		endcase
	end
	

end
always @(negedge data_clk or posedge rst) begin
	if(rst) begin
		ack_err <= 1'b0;
	end else if(data_clk == 0) begin
		case(state)
			start:begin
				ack_err <= 1'b0;
			end
			
			slv_ack1: begin
				ack_err <= SDA | ack_err;
			end
			
			rd: begin
				data_rx[bitcount] <= SDA;
			end
			
			slv_ack2: begin
				ack_err <= SDA | ack_err;
			end
			
			slv_ack3: begin
				ack_err <= SDA | ack_err;
			end
			
			default:begin
			end
		endcase
	end
end

/*
always@(posedge clk or posedge rst) begin //state logic
	if(rst)begin
		state <= IDLE;
	end else begin
		case(state)
		IDLE: begin
			if(ena == 1) begin
				state <= START;
			end
		end
		
		START: begin
			state <= ADDRESS_1;
		end
	
		ADDRESS_1: begin
			state <= ADDRESS_2;
		end
		
		ADDRESS_2: begin
			if ((bitcount - 1) >= 0) begin
				bitcount <= bitcount - 1;
				state <= ADDRESS_1;
			end else begin
				bitcount <= 7;
				state <= ADDRESS_ACK_1;
			end
		end
		
		ADDRESS_ACK_1: begin
			state <= ADDRESS_ACK_2;
		end
		
		ADDRESS_ACK_2: begin
			if (SDA == 1'b1) begin 
				state <= EE;
			end else begin
				state <= SUB_1;
			end

		end
		
		SUB_1: begin
			state <= SUB_2;
		end
		
		SUB_2: begin
			if ((bitcount - 1) >= 0) begin
				bitcount <= bitcount - 1;
				state <= SUB_1;
			end else begin
				bitcount <= 7;
				state <= SUB_ACK_1;
			end
		end
		
		SUB_ACK_1: begin
			state <= SUB_ACK_2;
		end
		
		SUB_ACK_2: begin
			if (SDA == 1) begin 
				state <= EE;
			end else begin
				state <= RW_1;
			end
		end
		
		RW_1: begin
			state <= RW_2;
		end
		
		RW_2: begin
			if ((bitcount - 1) >= 0) begin
				bitcount <= bitcount - 1;
				state <= RW_1;
			end else begin
				bitcount <= 7;
				state <= RW_ACK_1;
			end
		end
		
		RW_ACK_1: begin
			state <= RW_ACK_2;
		end
		
		RW_ACK_2: begin
			if (SDA == 1) begin
				ack_err <= 1'b1;
				state <= EE;
			end else begin
				bitcount <= 7;
				state <= STOP;
			end
		end
		
		STOP: begin
			state <= IDLE;
		end
		
		EE: begin
			
		end
		
		
		default: begin
			state <= IDLE;
		end 
		endcase
	end
end

always@(*)begin //output logic
	case(state)
	IDLE: begin
			SCL <= 1'b1;
			SDA01 <= 1;
			busy<= 1'b0;
		end
		
		START: begin
			SCL <= 1'b0;
			busy <= 1'b1;
			if(rw == 1)begin
				write_addr <= 8'hC0;
			end else begin
				write_addr <= 8'hC1;
			end
		end
	
		ADDRESS_1: begin
			SCL <= 1'b1;
			SDA01 <= write_addr[bitcount];
		end
		
		ADDRESS_2: begin
			SCL <= 1'b0;
		end
		
		ADDRESS_ACK_1: begin
			SCL <= 1'b1;
			SDA01 <= 1'b1;
		end
		
		ADDRESS_ACK_2: begin
			SCL <= 1'b0;
			ack <= SDA; // 0 is ack 1 if errs

		end
		
		SUB_1: begin
			SCL <= 1'b1;
			SDA01 <= addr[bitcount];
		end
		
		SUB_2: begin
			SCL <= 1'b0;
		end
		
		SUB_ACK_1: begin
			SCL <= 1'b0;
			SDA01 <= 1'b1;
		end
		
		SUB_ACK_2: begin
			SCL <= 1'b0;
		end
		
		RW_1: begin
			SCL <= 1'b1;
			if(rw == 1) begin
				SDA01 <= data_wr[bitcount];
			end else begin
				data_rd[bitcount] <= SDA01;
			end
		end
		
		RW_2: begin
			SCL <= 1'b1;
		end
		
		RW_ACK_1: begin
			SCL <= 1'b0;
			SDA01 <= 1'b1;
		end
		
		RW_ACK_2: begin
			SCL <= 1'b0;
		end
		
		STOP: begin
			SCL <= 1'b1;
			SDA01 <= 1'b1;
		end
		
		EE: begin
			SCL <= 1'b1;
			SDA01 <= 1'b1;
			ack <= 1'b1;
			busy <= 1'b0;
		end
		
		default: begin
			SCL <= 1'b0;
			SDA01 <= 1'b0;//1;
			busy <= 1'b0;
		end 
	endcase
end

*/

/*
always @(state_q) begin
	ack <= 0;
	SDA01 <= 1;
	SCL <= 1;
	busy <= 0;
	ack_err <= 0;
	
	case(state_q)
		IDLE: begin
			SCL <= 1'b1;
			SDA01 <= 1'b0;//1;
			//busy<= 1'b0;
			if(ena == 1) begin
				state_d <= START;
			//	busy <= 1'b0;
			end
		end
		
		START: begin
			SCL <= 1'b0;
			SDA01 <= 1'b1;//0;
			//if(rw == 1)begin
			//	write_addr <= 8'hC1;
			//end else begin
			//	write_addr <= 8'hC0;
			//end
			state_d <= IDLE;
		end
	
		ADDRESS_1: begin
			SCL <= 0;
			//SDA01 <= write_addr[bitcount];
			state_d <= ADDRESS_2;
		end
		
		ADDRESS_2: begin
			SCL <= 1'b1;
		//	if ((bitcount - 1) >= 0) begin
			//	bitcount <= bitcount - 1;
		//		state_d <= ADDRESS_1;
		//	end else begin
			//	bitcount <= 7;
				state_d <= ADDRESS_ACK_1;
		//	end
		end
		
		ADDRESS_ACK_1: begin
			SCL <= 1'b0;
			//SDA01 <= 1'b1;
			state_d <= IDLE; //ADDRESS_ACK_2/
		end
		
		ADDRESS_ACK_2: begin
			SCL <= 1'b1;
			ack <= SDA; // 0 is ack 1 if err
			if (ack == 1'b1) begin 
				state_d <= EE;
			end else begin
				bitcount <= 7;
			end

		end
		
		SUB_1: begin
			SCL <= 1'b0;
			SDA01 <= addr[bitcount];
			state_d <= SUB_2;
		end
		
		SUB_2: begin
			SCL <= 1'b1;
			if ((bitcount - 1) >= 0) begin
				bitcount <= bitcount - 1;
				state_d <= SUB_1;
			end else begin
				bitcount <= 7;
				state_d <= SUB_ACK_1;
			end
		end
		
		SUB_ACK_1: begin
			SCL <= 1'b0;
			SDA01 <= 1'b1;
			state_d <= SUB_ACK_2;
		end
		
		SUB_ACK_2: begin
			SCL <= 1'b1;
			ack <= SDA; // 0 is ack 1 if err
			if (ack == 1) begin 
				ack_err <= 1'b1;
				state_d <= EE;
			end else begin
				bitcount <= 7;
				state_d <= RW_1;
			end
		end
		
		RW_1: begin
			SCL <= 1'b0;
			if(rw == 1) begin
				SDA01 <= data_wr[bitcount];
			end else begin
				data_rd[bitcount] <= SDA01;
			end
			state_d <= RW_2;
		end
		
		RW_2: begin
			SCL <= 1'b1;
			if ((bitcount - 1) >= 0) begin
				bitcount <= bitcount - 1;
				state_d <= RW_1;
			end else begin
				bitcount <= 7;
				state_d <= RW_ACK_1;
			end
		end
		
		RW_ACK_1: begin
			SCL <= 1'b0;
			SDA01 <= 1'b1;
			state_d <= RW_ACK_2;
		end
		
		RW_ACK_2: begin
			SCL <= 1'b1;
			ack <= SDA; // 0 is ack 1 if err
			if (ack == 1) begin
				ack_err <= 1'b1;
				state_d <= EE;
			end else begin
				bitcount <= 7;
				state_d <= STOP;
			end
		end
		
		STOP: begin
			SCL <= 1'b1;
			SDA01 <= 1'b1;
		end
		
		EE: begin
			SCL <= 1'b1;
			SDA01 <= 1'b1;
			ack <= 1'b1;
			busy <= 1'b0;
		end
		
		default: begin
			state_d <= IDLE;
		end 
	endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_q <= IDLE;
    end else begin
        state_q <= state_d;
    end
end
		*/
		
endmodule
