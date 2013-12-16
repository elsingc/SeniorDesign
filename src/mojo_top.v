module mojo_top(
    input clk,
    input rst_n,
    input cclk,
    output[7:0]led,
    output spi_miso,
    input spi_ss,
    input spi_mosi,
    input spi_sck,
    output [3:0] spi_channel,
    input avr_tx,
    output avr_rx,
    input avr_rx_busy
	 //input [7:0] pixel,
	 //input pclk,
	 //input href,
//	 //input vsync,
//	 inout sda_out,
//	 inout scl_out,
//	 input start
    );

wire rst = ~rst_n;
//
//assign spi_miso = 1'bz;
//assign avr_rx = 1'bz;
//assign spi_channel = 4'bzzzz;
//assign toggle_check = toggle;

//assign sda_out = 1'bz;
//assign scl_out = 1'b1;

Data_Controller Data_Controller(
	.busy(),
	.block(), //set to 0
	.new_data_tx(),
	.data_tx(),
	.new_data_rx(),
	.data_rx(),
	.data(),
	.addr(),
	.rst(rst),
	.clk(clk)
);
	 
Data_serial_rx serial_rx #(
	parameter CLK_PER_BIT = 50,
	parameter CTR_SIZE = 6
	)(
	.clk(clk),
	.rst(rst),
	.rx(),
	.data(),
	.new_data()
);	 
	
Data_serial_tx serial_tx #(
	parameter CLK_PER_BIT = 50,
	parameter CTR_SIZE = 6
	)(
	.clk(clk),
	.rst(rst),
	.tx(),
	.block(),//remove only used for the avr side of stuff
	.busy(),
	.data(),
	.new_data()
	);
	
Altimeter_Controller Altimeter_Controller(
	.pressure,
	.temp,
	.delta_pressure,
	.delta_temp,
	.min_pressure,
	.max_pressure,
	.min_temp,
	.max_temp,
	.ena,
	.addr,
	.sub_addr,
	.data_wr,
	.data_rd,
	.busy,
	.ack_err,
	.rst(clk),
	.clk(clk)
    );

Alt_I2C_Driver I2C_Driver(
	.clk(),
	.rst(rst),
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
	
Gyro_Controller Gyro_Controller(
	output [19:0] roll,
	output [19:0] pitch,
	output [19:0] yaw,
	output ena,
	output [7:0] data_wr,
	input [7:0] data_rd,
	input busy,
	input new,
	input rst,
	input clk
    );

Gyro_Spi_Master Spi_Master(
	input clk,
	input rst,
	input ss,
	input mosi,
	output miso,
	input sck,
	output done,
	input [7:0] din,
	output [7:0] dout
    );
	 
Accelerometer_Controller Accelerometer_Controller(
	output [19:0] x_accl,
	output [19:0] y_accl,
	output [19:0] z_accl,
	output ena,
	output [7:0] data_wr,
	output [7:0] data_rd,
	input busy,
	output new,
	input rst,
	input clk
    );
	 
Accl_Spi_Master Spi_Master(
	input clk,
	input rst,
	input ss,
	input mosi,
	output miso,
	input sck,
	output done,
	input [7:0] din,
	output [7:0] dout
    ); 
	 
GPS_Controller GPS_Controller(
	output [19:0] x_gps,
	output [19:0] y_gps,
	output [19:0] z_gps,
	output [19:0] time_gps,
	output [19:0] ground_speed,
	output ena,
	output [7:0]data_wr,
	input [7:0] data_rd,
	input busy,
	input new,
	input rst,
	input clk
    );
	 
GPS_Spi_Master Spi_Master(
	input clk,
	input rst,
	input ss,
	input mosi,
	output miso,
	input sck,
	output done,
	input [7:0] din,
	output [7:0] dout
    ); 
	 
Analog_Controller Analog_Controller(
	output [7:0] ch0,
	output [7:0] ch1,
	output [7:0] ch4,
	output [7:0] ch5,
	output [7:0] ch6,
	output [7:0] ch7,
	output [7:0] ch8,
	output [7:0] ch9,
	output [7:0] channel,
	input new_sample,
	input [7:0] sample,
	input [7:0] sample_channel,
	input rst,
	input clk
    );
	 
avr_interface avr_interface (
    .clk(clk),
    .rst(rst),
    .cclk(cclk),
    .spi_miso(spi_miso),
    .spi_mosi(spi_mosi),
    .spi_sck(spi_sck),
    .spi_ss(spi_ss),
    .spi_channel(spi_channel),
    .tx(avr_rx),
    .rx(avr_tx),
    .channel(channel),
    .new_sample(new_sample),
    .sample(sample),
    .sample_channel(sample_channel),
    .tx_data(8'h00),
    .new_tx_data(1'b0),
    .tx_busy(),
    .tx_block(avr_rx_busy),
    .rx_data(),
    .new_rx_data()
);

Camera_Initializer Camera_Initializer(
	input start,
	input new_data,
	input [7:0] data,
	output ena,
	output [7:0] addr,
	output [7:0] sub_addr,
	output [7:0] data_wr,
	input [7:0] data_rd,
	input busy,
	input ack_err,
	input rst,
	input clk
    );

Camera_Digitizer Camera_Digitizer(
	input busy,
	input block,
	input new_data,
	input [7:0] data,
	input dtr,
	input vsync,
	input href,
	input pclk,
	input [7:0] ybus,
	input rst,
	input clk
    );
	 
cam_serial_rx serial_rx #(
	parameter CLK_PER_BIT = 50,
	parameter CTR_SIZE = 6
	)(
	input clk,
	input rst,
	input rx,
	output [7:0] data,
	output new_data
	);
	
cam_serial_tx serial_tx #(
	parameter CLK_PER_BIT = 50,
	parameter CTR_SIZE = 6
	)(
	input clk,
	input rst,
	output tx,
	input block,//remove only used for the avr side of stuff
	output busy,
	input [7:0] data,
	input new_data
	);















//
//
//wire ena, rw, ack_err, busy, pulse;
//wire [7:0] addr, data_wr, data_rd, sub_addr;
//
//cam_driver sends_one(
//	.comm_clk(clk),
//	.start(start),
//	.ena(ena),
//	.addr(addr),
//	.data_wr(data_wr),
//	.data_rd(data_rd),
//	.sub_addr(sub_addr),
//	.rw(rw),
//	.ack_err(ack_err),
//	.busy(busy),
//	.rst(rst)
//	);
//
//I2C_Driver cam_i2c(
//	.SDA(sda_out),
//	.SCL(scl_out),
//	.clk(clk),
//	.rst(rst),
//	.ena(ena),
//	.addr(addr),
//	.rw(rw),
//	.data_wr(data_wr),
//	.data_rd(data_rd),
//	.sub_addr(sub_addr),
//	.ack_err(ack_err),
//	.busy(busy)
//   );
//initializer_pulse pulser(
//	.clk(clk),
//   .rst(rst),
//   .pulse(pulse)
//    );
	 
endmodule