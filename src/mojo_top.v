module mojo_top(
    input clk,
    input rst_n,
	 output[7:0]led,
  /*  input cclk,
    
    output spi_miso,
    input spi_ss,
    input spi_mosi,
    input spi_sck,
    output [3:0] spi_channel,
    input avr_tx,
    output avr_rx,
    input avr_rx_busy,
	
	 input [7:0] pixel,
	 input pclk,
	 input href,
	 input vsync,
	 input cam_tx,
	 output cam_rx,
	  */
	 inout alt_sda_out,
	 inout alt_scl_out,
	 inout imu_sda_out,
	 inout imu_scl_out,
	 /*inout cam_sda_out,
	 inout cam_scl_out,*/
	 
	 input gps_rx_pin,
	 output gps_tx_pin,
	 
	 //input start,
	 output data_tx, 
	 input data_rx
    );

wire rst = ~rst_n;

assign spi_miso = 1'bz;
assign avr_rx = 1'bz;
assign spi_channel = 4'bzzzz;
//assign toggle_check = toggle;

//assign led[7:1] = 7'h00;
//assign led[0] = 1'b1;

wire [23:0] pressure;
wire [31:0] gps_lon, gps_lat, gps_time, ground_speed;
wire [15:0] alt_temp, gyro_temp, gyro_x, gyro_y, gyro_z, accl_x, accl_y, accl_z, magm_x, magm_y, magm_z;
//====================DATA CONTROLLER===================================
wire data_busy, data_block, data_new_data_tx, data_new_data_rx;
wire [7:0] data_data_addr, snsr_data, snsr_addr, data_data_tx, data_data_rx;
Data_Controller Data_Controller(
	.debug(led),
	.busy(data_busy),
	.block(data_block), //set to 0
	.new_data_tx(data_new_data_tx),
	.data_tx(data_data_tx),
	.new_data_rx(data_new_data_rx),
	.data_rx(data_data_rx),
	.data(snsr_data),
	.addr(snsr_addr),
	.rst(rst),
	.clk(clk)
);
	 
serial_rx #(.CLK_PER_BIT(434), .CTR_SIZE(13)) Data_serial_rx (
	.clk(clk),
	.rst(rst),
	.rx(data_rx),
	.data(data_data_rx),
	.new_data(data_new_data_rx)
);	 
	
serial_tx #(.CLK_PER_BIT(434), .CTR_SIZE(13)) Data_serial_tx (
	.clk(clk),
	.rst(rst),
	.tx(data_tx),
	.block(data_block),//remove only used for the avr side of stuff
	.busy(data_busy),
	.data(data_data_tx),
	.new_data(data_new_data_tx)
	);
	
Sensor_Reg Sensor_Reg(
	.data(snsr_data),
	.addr(snsr_addr),
	.pressure(pressure),
	.alt_temp(alt_temp),
	.gyro_temp(gyro_temp),
	.gyro_x(gyro_x),
	.gyro_y(gyro_y),
	.gyro_z(gyro_z),
	.x_accl(accl_x),
	.y_accl(accl_y),
	.z_accl(accl_z),
	.magm_x(magm_x),
	.magm_y(magm_y),
	.magm_z(magm_z),
   .gps_lon(gps_lon),
	.gps_lat(gps_lat),
	.gps_time(gps_time),
	.ground_speed(ground_speed),
	.air_speed_p(),
	.air_speed_n(),
	.rst(rst),
	.clk(clk)
	);

//====================ALTIMETER=========================================
wire alt_ena, alt_rw, alt_busy, alt_ready, alt_ack_err, alt_start_transfer, alt_stop_transfer, alt_r_start;
wire [7:0] alt_data_wr, alt_data_rd;
Altimeter_Controller Altimeter_Controller(
	.pressure(pressure),
	.temp(alt_temp),
	.ena(alt_ena),
	.rw(alt_rw),
	.data_wr(alt_data_wr),
	.data_rd(alt_data_rd),
	.ready(alt_ready),
	.busy(alt_busy),
	.ack_err(alt_ack_err),
	.start_transfer(alt_start_transfer),
	.stop_transfer(alt_stop_transfer),
	.r_start(alt_r_start),
	.rst(rst),
	.clk(clk)
    );

I2C_Driver Alt_I2C_Driver(
	.clk(clk),
	.rst(rst),
	.rw(alt_rw),
	.data_wr(alt_data_wr),
	.data_rd(alt_data_rd),
	.busy(alt_busy),
	.ready(alt_ready),
	.ack_err(alt_ack_err),
	.ena(alt_ena),
	.start_transfer(alt_start_transfer),
	.stop_transfer(alt_stop_transfer),
	.r_start(alt_r_start),
	.SDA(alt_sda_out),
	.SCL(alt_scl_out)
);	 
//====================IMU CONTROLLER====================================
wire imu_ena, imu_rw, imu_busy, imu_ack_err, imu_start_transfer, imu_stop_transfer, imu_r_start;
wire [7:0] imu_data_wr, imu_data_rd;

IMU_Controller IMU_Controller(
	.GYRO_TEMP(gyro_temp),
	.GYRO_X(gyro_x),
	.GYRO_Y(gyro_y),
	.GYRO_Z(gyro_z),
	.ACCL_X(accl_x),
	.ACCL_Y(accl_y),
	.ACCL_Z(accl_z),
	.MAGM_X(magm_x),
	.MAGM_Y(magm_y),
	.MAGM_Z(magm_z),
	.ena(imu_ena),
	.rw(imu_rw),
	.data_wr(imu_data_wr),
	.r_start(imu_r_start),
	.start_transfer(imu_start_transfer),
	.stop_transfer(imu_stop_transfer),
	.data_rd(imu_data_rd),
	.busy(imu_busy),
	.ready(imu_ready),
	.rst(rst),
	.clk(clk)
);

I2C_Driver IMU_I2C_Driver(
	.clk(clk),
	.rst(rst),
	.rw(imu_rw),
	.data_wr(imu_data_wr),
	.data_rd(imu_data_rd),
	.busy(imu_busy),
	.ready(imu_ready), 
	.ack_err(imu_ack_err),
	.ena(imu_ena),
	.start_transfer(imu_start_transfer),
	.stop_transfer(imu_stop_transfer),
	.r_start(imu_r_start),
	.SDA(imu_sda_out),
	.SCL(imu_scl_out)
);	 

//====================GPS===============================================
wire gps_tx_send, gps_tx_busy, gps_rx_new_data;
wire [7:0] gps_tx, gps_rx;
serial_tx #(.CLK_PER_BIT(5208), .CTR_SIZE(13)) GPS_serial_tx (
	.clk(clk),
	.rst(rst),
	.tx(gps_tx_pin),
	.block(),
	.busy(gps_tx_busy),
	.data(gps_tx),
	.new_data(gps_tx_send)
	);

serial_rx #(.CLK_PER_BIT(5208), .CTR_SIZE(13)) GPS_serial_rx (
	.clk(clk),
	.rst(rst),
	.rx(gps_rx_pin),
	.data(gps_rx),
	.new_data(gps_rx_new_data)
);


GPS_Controller GPS_Controller(
	.lon(gps_lon),
	.lat(gps_lat),
	.time_(gps_time),
	.ground_speed(ground_speed),

	.tx_data(gps_tx),
	.tx_send(gps_tx_send),
	.tx_busy(gps_tx_busy),
	
	.rx_data(gps_rx),
	.rx_new(gps_rx_new_data),

	.busy(),
	.new(),
	.rst(rst),
	.clk(clk)
);

//====================ANALOG CONGROLLER=================================
/*Analog_Controller Analog_Controller(
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
	 */
//====================CAM CONTROLLER====================================
wire cam_ena, cam_rw, cam_ack_err, cam_busy, cam_pulse, cam_new_tx_data, cam_tx_busy, cam_new_rx_data;
wire [7:0] cam_addr, cam_data_wr, cam_data_rd, cam_sub_addr, cam_tx_data, cam_rx_data;
/*
serial_rx #(.CLK_PER_BIT(5208), .CTR_SIZE(8)) serial_rx (
	.clk(clk),
	.rst(rst),
	.rx(cam_rx),
	.data(cam_rx_data),
	.new_data(cam_new_rx_data)
);

serial_tx #(.CLK_PER_BIT(5208), .CTR_SIZE(8)) serial_tx (
	.clk(clk),
	.rst(rst),
	.tx(cam_tx),
	.block(cam_tx_block),
	.busy(cam_tx_busy),
	.data(cam_tx_data),
	.new_data(cam_new_tx_data)
);

cam_serializer cam_serializer(
	.clk(clk),
	.rst(rst),
	.ybuss(pixel),
	.vsync(vsync),
	.href(href),
	.pclk(pclk),
	.tx_data(cam_tx_data),
	.new_tx_data(cam_new_tx_data),
	.tx_busy(cam_tx_busy),
	.rx_data(cam_rx_data),
	.new_rx_data(cam_new_rx_data),
	.start_config(cam_start)
);

cam_driver sends_one(
	.clk(clk),
	.start(cam_start),
	.ena(cam_ena),
	.addr(cam_addr),
	.data_wr(cam_data_wr),
	.data_rd(cam_data_rd),
	.sub_addr(cam_sub_addr),
	.rw(cam_rw),
	.ack_err(cam_ack_err),
	.busy(cam_busy),
	.rst(rst)
);

cam_I2C_Driver cam_I2C_Driver(
	.SDA(cam_sda_out),
	.SCL(cam_scl_out),
	.clk(clk),
	.rst(rst),
	.ena(cam_ena),
	.addr(cam_addr),
	.rw(cam_rw),
	.data_wr(cam_data_wr),
	.data_rd(cam_data_rd),
	.sub_addr(cam_sub_addr),
	.ack_err(cam_ack_err),
	.busy(cam_busy)
);
*/
//====================END===============================================
//initializer_pulse pulser(
//	.clk(clk),
//   .rst(rst),
//   .pulse(pulse)
//    );
	 
endmodule
