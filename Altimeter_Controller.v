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


reg [STATE_SIZE:0] state;

always @(*) begin
	state <= START_STATE;
	
	/*START
	SLave adress[7:0] receive or send [8]
	ack
	data
	stop
	*/
	
	/*Start
	send oxC0
	send ox26
	send oxB8
	RStart
	send oxC0
	send ox13
	send ox07
	Rstart
	send oxC0
	send ox26
	send oxB9
	//read
	send oxc0
	send ox00(Adress to read from)
	Rstart
	send oxc1
	start read
	if( byte & ox08)
		Read 5 bytes
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
				state <= START_WAIT_STATE;
			end else begin
				state <= START_STATE;
			end		
		end
		
		START_WAIT_STATE:begin
			ena <= 1'b1;
			if(ready && busy) begin
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
				state <= WAIT_SEND_STATE;
			end
		end
		
//WAIT_C0		
		
		WAIT_C0_SATE:begin
			ena <= 1'b1;
			data_wr <= 8'h00;
			rw <= 1'b0;
			start_transfer <= 1'b0;
			stop_transfer <= 1'b0;
			if(!busy) begin
				state <= SEND_26_STATE;
			end else begin
				state <=  WAIT_C0_STATE;
			end
		end
