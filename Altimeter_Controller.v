`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:04:45 12/15/2013 
// Design Name: 
// Module Name:    Altimeter_Controller 
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
module Altimeter_Controller(

    output [19:0] pressure,
        output [19:0] temp,
        output [19:0] delta_pressure,
        output [19:0] delta_temp,
        output [19:0] min_pressure,
        output [19:0] max_pressure,
        output [19:0] min_temp,
        output [19:0] max_temp,
        output ena,
        output [7:0] addr,
        output [7:0] sub_addr,
        output [7:0] data_wr,
        input [7:0] data_rd,
    input ready,
        input busy,
        input ack_err,
        input rst,
        input clk,
    output reg start_transfer,
    output reg stop_transfer
    );

  //make signals active high
  wire rst = ~rst_n;

localparam STATE_SIZE = 1;
localparam START_STATE=0;
localparam START_WAIT_STATE=1;
localparam SEND_C0_STATE=2;
localparam WAIT_C0_STATE=3;
localparam SEND_26_STATE=4;
localparam WAIT_26_STATE=5;
localparam SEND_B8_STATE=6;
localparam WAIT_B8_STATE=7;
localparam RSTART_STATE=8;
localparam RSTART_WAIT_STATE=9;
localparam SEND_13_STATE=10;
localparam WAIT_13_STATE=11;
localparam SEND_07_STATE=12;
localparam WAIT_07_STATE=13;
localparam SEND_B9_STATE=14;
localparam WAIT_B9_STATE=15;
localparam state_control=0;

reg [STATE_SIZE:0] state;
always @(*) begin
 state <= START_STATE;

 /*START
 SLave adress[7:0] receive or send [8]
 ack
 data
 stop
 */

 /*Start 0,1
 send oxC0 2,3
 send ox26 4,5
 send oxB8 6,7

 RStart    8,9
 send oxC0 10,11
 send ox13 12,13
 send ox07 14,15

 Rstart   16,17
 send oxC0  18,19
 send ox26  20,21
 send oxB9  22,23

 //read
 send oxc0  24,25
 send ox00 26,27(Adress to read from)
 Rstart 28,29
 send oxc1 30,31
 start read
 if( byte & ox08)
  Read 5 bytes

repeated start

send 0xc0 32,33
send 0x00 34,35
send repeated start 36,37
send 0xc1 38,39

send 0xc0 40,41
send 0x01 42,43
send repeated start 44,45
send 0xc1 46,47

send 0xc0 48,49
send 0x02 50,51
send repeated start 52,53
send 0xc1 54,55
 else
  restart send block
 Stop
 */

 case(state)

//START
  START_STATE:begin
   ena <= 1'b1;
   if(busy) begin
    ena <= 1'b0;
    state_control <= state_control + 1;
    state <= START_WAIT_STATE;
   end else begin
    state <= START_STATE;
   end
  end

  START_WAIT_STATE:begin
   ena <= 1'b1;
   if(ready && busy) begin// mistake
    state_control <= state_control + 1;
    state <= SEND_C0_STATE;
   end else begin
    state <= START_WAIT_STATE;
   end
  end
//SEND 0xC0
  SEND_C0_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'hC0;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_C0_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_C0_STATE;
   end
  end

//WAIT_C0
  /*
    send 0xc0 32,33
send 0x00 34,35
send repeated start 36,37
send 0xc1 38,39

send 0xc0 40,41
send 0x01 42,43
send repeated start 44,45
send 0xc1 46,47

send 0xc0 48,49
send 0x02 50,51
send repeated start 52,53
send 0xc1 54,55
  */
  WAIT_C0_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    if(state_control == 3 || state_control == 19)begin
     state_control <= state_control + 1;
     state <= SEND_26_STATE;
    end else if(state_control == 11)begin
     state_control <= state_control + 1;
     state <= SEND_13_STATE;
    end else if(state_conrol == 25 || state_control == 33) begin
     state_control <= state_control + 1;
 state <= SEND_00_STATE;
end else if(state_control == 41) begin
 state_control <= state_control + 1;
 state <= SEND_01_STATE;
     //state <= address to read from beginning of I2C read block
    end else if(state_control == 49) begin
 state_control <= state_control + 1;
 state <= SEND_02_STATE;
     //state <= address to read from beginning of I2C read block
    end
   end else begin
    state <=  WAIT_C0_STATE;
   end
  end
//SEND 0x26
  SEND_26_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h26;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_26_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_26_STATE;
   end
  end

//WAIT_26

  WAIT_26_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    if(state_control == 5)begin
     state_control <= state_control + 1;
     state <= SEND_B8_STATE;
    end else if(state_control == 21)begin
     state_control <= state_control + 1;
     state <= SEND_B9_STATE;
    end
   end else begin
    state <=  WAIT_26_STATE;
   end
  end

//SEND 0xB8
  SEND_B8_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'hB8;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_B8_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_B8_STATE;
   end
  end

//WAIT_B8

  WAIT_B8_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
// should be 0 when waiting?
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= RSTART_STATE;
   end else begin
    state <=  WAIT_B8_STATE;
   end
  end

//REPEATED START(to be implemented)
//SEND 0x13
  SEND_13_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h13;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_13_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_13_STATE;
   end
  end

//WAIT_13

  WAIT_13_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= SEND_07_STATE;
   end else begin
    state <=  WAIT_13_STATE;
   end
  end
//SEND 0x07
  SEND_07_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h07;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_07_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_07_STATE;
   end
  end

//WAIT_07

  WAIT_07_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= RSTART_STATE;
   end else begin
    state <=  WAIT_07_STATE;
   end
  end
//SEND 0xB9
  SEND_B9_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'hB9;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_B9_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_B9_STATE;
   end
  end

//WAIT_B9

  WAIT_B9_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= SEND_C0_STATE;
   end else begin
    state <=  WAIT_B9_STATE;
   end
  end

  SEND_00_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_00_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_00_STATE;
   end
  end

//WAIT_26

  WAIT_00_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= SEND_C0_STATE;
   end else begin
    state <=  RSTART_STATE;
end
  end

   SEND_01_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h01;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_01_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_01_STATE;
   end
  end

//WAIT_26

  WAIT_01_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= SEND_C0_STATE;
   end else begin
    state <=  RSTART_STATE;
end
  end

  SEND_02_STATE:begin
   ena <= 1'b1;
   data_wr <= 8'h02;
   rw <= 1'b0;
   if(!busy && ready) begin
    start_transfer <= 1'b1;
    stop_transfer <= 1'b0;
    state <= SEND_02_STATE;
   end else begin
    start_transfer <= 1'b0;
    stop_transfer <= 1'b0;
    state_control <= state_control + 1;
    state <= WAIT_02_STATE;
   end
  end

//WAIT_26

  WAIT_02_SATE:begin
   ena <= 1'b1;
   data_wr <= 8'h00;
   rw <= 1'b0;
   start_transfer <= 1'b0;
   stop_transfer <= 1'b0;
   if(!busy) begin
    state_control <= state_control + 1;
    state <= SEND_C0_STATE;
   end else begin
    state <=  RSTART_STATE;
end
  end

  // one successful conditional result of repeated start leads here
   SEND_C1_STATE:begin
ena <= 1'b1;
data_wr <= 8'hc1;
rw <= 1'b0;
if(!busy && ready) begin
start_transfer <= 1'b1;
stop_transfer <= 1'b0;
state <= SEND_C1_STATE;
end else begin
start_transfer <= 1'b0;
stop_transfer <= 1'b0;
state_control <= state_control + 1;
state <= WAIT_C1_STATE;
end
  end

//WAIT_13

  WAIT_C1_SATE:begin
ena <= 1'b1;
data_wr <= 8'h00;
rw <= 1'b0;
start_transfer <= 1'b0;
stop_transfer <= 1'b0;
if(!busy) begin
// else condition yet to be determined
if( data_rd == 0x08) begin
 state_control <= state_control + 1;
 state <= SEND_C0_STATE;
end
else if (state_control == 39) begin
 pressure[19:12] <= data_rd;
 state_control <= state_control + 1;
 state <=SEND_CO_STATE;
end
else if (state_control == 47) begin
 pressure[11:4] <= data_rd;
 state_control <= state_control + 1;
 state <=SEND_CO_STATE;
end
else if (state_control == 55) begin
 pressure[3:0] <= data_rd;
 state_control <= state_control + 1;
 state <=SEND_CO_STATE;
end
end else begin
state <=  WAIT_C1_STATE;
end
  end
  endcase
end
endmodule
