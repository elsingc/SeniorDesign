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
	 
Data_serial_rx #(.CLK_PER_BIT(100), .CTR_SIZE(7)) serial_rx (
	.clk(clk),
	.rst(rst),
	.rx(),
	.data(),
	.new_data()
);	 
	
Data_serial_tx #(.CLK_PER_BIT(100), .CTR_SIZE(7)) serial_tx (
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
	.clk(clk),
	.rst(rst),
	.ena(),
	.addr(),
	.rw(),
	.data_wr(),
	.sub_addr(),
	.busy(),
	.data_rd(),
	.ack_err(),
	.SDA(),
	.SCL()
	//inout SCL
);
	
Gyro_Controller Gyro_Controller(
	.roll(),
	.pitch(),
	.yaw(),
	.ena(),
	.data_wr(),
	.data_rd(),
	.busy(),
	.new(),
	.rst(rst),
	.clk(clk)
);

Gyro_Spi_Master Spi_Master(
	.clk(clk),
	.rst(rst),
	.ss(),
	.mosi(),
	.miso(),
	.sck(),
	.done(),
	.din(),
	.dout()
);
	 
Accelerometer_Controller Accelerometer_Controller(
	.x_accl(),
	.y_accl(),
	.z_accl(),
	.ena(),
	.data_wr(),
	.data_rd(),
	.busy(),
	.new(),
	.rst(rst),
	.clk(clk)
);
	 
Accl_Spi_Master Spi_Master(
	.clk(clk),
	.rst(rst),
	.ss(),
	.mosi(),
	.miso(),
	.sck(),
	.done(),
	.din(),
	.dout()
); 
	 
GPS_Controller GPS_Controller(
	.x_gps(),
	.y_gps(),
	.z_gps(),
	.time_gps(),
	.ground_speed(),
	.ena(),
	.data_wr(),
	.data_rd(),
	.busy(),
	.new(),
	.rst(rst),
	.clk(clk)
);
	 
GPS_Spi_Master Spi_Master(
	.clk(clk),
	.rst(rst),
	.ss(),
	.mosi(),
	.miso(),
	.sck(),
	.done(),
	.din(),
	.dout()
); 
	 
Analog_Controller Analog_Controller(
	.ch0(),
	.ch1(),
	.ch4(),
	.ch5(),
	.ch6(),
	.ch7(),
	.ch8(),
	.ch9(),
	.channel(),
	.new_sample(),
	.sample(),
	.sample_channel(),
	.rst(),
	.clk(clk)
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
	.start(),
	.new_data(),
	.data(),
	.ena(),
	.addr(),
	.sub_addr(),
	.data_wr(),
	.data_rd(),
	.busy(),
	.ack_err(),
	.rst(rst),
	.clk(clk)
);

Camera_Digitizer Camera_Digitizer(
	.busy(),
	.block(),
	.new_data(),
	.data(),
	.dtr(),
	.vsync(),
	.href(),
	.pclk(),
	.ybus(),
	.rst(rst),
	.clk(clk)
    );
	 
cam_serial_rx#(.CLK_PER_BIT(100), .CTR_SIZE(7)) serial_rx (
	.clk(clk),
	.rst(rst),
	.rx(),
	.data(),
	.new_data()
	);
	
cam_serial_tx #(.CLK_PER_BIT(100), .CTR_SIZE(7)) serial_tx (
	.clk(clk),
	.rst(rst),
	.tx(),
	.block(),//remove only used for the avr side of stuff
	.busy(),
	.data(),
	.new_data()
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