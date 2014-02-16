module serial_tx_dualspeed
  #(
    parameter CLK_PER_BIT_SLOW = 5208,
    parameter CLK_PER_BIT_FAST =  434,
    parameter CTR_SIZE = 13
    )
   (
    input       clk,
    input       rst,
    input       requested_speed,
    output      current_speed,
    output      tx,
    input       block,
    output      busy,
    input [7:0] data,
    input       new_data
    );

   localparam STATE_SIZE = 2;
   localparam IDLE = 2'd0,
     START_BIT = 2'd1,
     DATA = 2'd2,
     STOP_BIT = 2'd3;

   reg [CTR_SIZE-1:0] ctr_d, ctr_q;
   reg [2:0]          bit_ctr_d, bit_ctr_q;
   reg [7:0]          data_d, data_q;
   reg [STATE_SIZE-1:0] state_d, state_q = IDLE;
   reg                  tx_d, tx_q;
   reg                  busy_d, busy_q;
   reg                  block_d, block_q;
   reg                  speed_d, speed_q;

   assign tx = tx_q;
   assign busy = busy_q;
   assign current_speed = speed_q;

   wire [CTR_SIZE-1:0]  ctr_max;
   assign ctr_max = (speed_q == 0) ? CLK_PER_BIT_SLOW
                    : CLK_PER_BIT_FAST;

   always @(*) begin
      block_d = block;
      ctr_d = ctr_q;
      bit_ctr_d = bit_ctr_q;
      data_d = data_q;
      state_d = state_q;
      busy_d = busy_q;
      speed_d = speed_q;

      case (state_q)
	IDLE: begin
	   if (requested_speed != speed_q) begin
	      speed_d = requested_speed;
	   end
	   if (block_q) begin
	      busy_d = 1'b1;
	      tx_d = 1'b1;
	   end else begin
	      busy_d = 1'b0;
	      tx_d = 1'b1;
	      bit_ctr_d = 3'b0;
	      ctr_d = 1'b0;
	      if (new_data) begin
		 data_d = data;
		 state_d = START_BIT;
		 busy_d = 1'b1;
	      end
	   end
	end
	START_BIT: begin
	   busy_d = 1'b1;
	   ctr_d = ctr_q + 1'b1;
	   tx_d = 1'b0;
	   if (ctr_q == ctr_max - 1) begin
	      ctr_d = 1'b0;
	      state_d = DATA;
	   end
	end
	DATA: begin
	   busy_d = 1'b1;
	   tx_d = data_q[bit_ctr_q];
	   ctr_d = ctr_q + 1'b1;
	   if (ctr_q == ctr_max - 1) begin
	      ctr_d = 1'b0;
	      bit_ctr_d = bit_ctr_q + 1'b1;
	      if (bit_ctr_q == 7) begin
		 state_d = STOP_BIT;
	      end
	   end
	end
	STOP_BIT: begin
	   busy_d = 1'b1;
	   tx_d = 1'b1;
	   ctr_d = ctr_q + 1'b1;
	   if (ctr_q == ctr_max - 1) begin
	      state_d = IDLE;
	   end
	end
	default: begin
	   state_d = IDLE;
	end
      endcase
   end

   always @(posedge clk) begin
      if (rst) begin
	 state_q <= IDLE;
	 tx_q <= 1'b1;
	 speed_q <= 0;
      end else begin
	 state_q <= state_d;
	 tx_q <= tx_d;
	 speed_q <= speed_d;
      end

      block_q <= block_d;
      data_q <= data_d;
      bit_ctr_q <= bit_ctr_d;
      ctr_q <= ctr_d;
      busy_q <= busy_d;
   end

endmodule
