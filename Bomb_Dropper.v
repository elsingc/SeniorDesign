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
module Bomb_Dropper(
	output reg [7:0] debug,
	input drop,
	output servoDrive,
	input rst,
	input clk
    );
	 
reg pwm_d, pwm_q;
reg [19:0] ctr_d, ctr_q;
 
assign servoDrive = pwm_q;
 
always @(*) begin
	ctr_d = ctr_q + 1'b1;
	if(drop == 1'b1) begin
		if (9'd165 > ctr_q[19:8])
			pwm_d = 1'b1;
		else
			pwm_d = 1'b0;
	end else begin	 
		if (9'd200 > ctr_q[19:8])
			pwm_d = 1'b1;
		else
			pwm_d = 1'b0;
	end
end
 
always @(posedge clk) begin
    if (rst) begin
        ctr_q <= 1'b0;
    end else begin
        ctr_q <= ctr_d;
    end
 
    pwm_q <= pwm_d;
end
 
endmodule
