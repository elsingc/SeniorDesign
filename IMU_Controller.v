//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:06:23 12/15/2013 
// Design Name: 
// Module Name:    Gyro_Controller 
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
//////////////////////////////////////////////////////////////////////////////////
module IMU_Controller(
	output reg [7:0] debug,
	output reg [15:0] GYRO_TEMP,
	output reg [15:0] GYRO_X = 16'd0,
	output reg [15:0] GYRO_Y = 16'd0,
	output reg [15:0] GYRO_Z = 16'd0,
	output reg [15:0] ACCL_X = 16'd0,
	output reg [15:0] ACCL_Y = 16'd0,
	output reg [15:0] ACCL_Z = 16'd0,
	output reg [15:0] MAGM_X = 16'd0,
	output reg [15:0] MAGM_Y = 16'd0,
	output reg [15:0] MAGM_Z = 16'd0,
	output reg ena,
	output reg rw,
	output reg [7:0] data_wr,
	output reg r_start,
	output reg start_transfer, 
	output reg stop_transfer,
	input [7:0] data_rd,
	input busy,
	input ready,
	input rst,
	input clk
);
  
localparam STATE_SIZE = 10;
//================GYRO INIT===========================
localparam 			GYRO_INIT_DLPF_START					= 0,
						GYRO_INIT_DLPF_START_WAIT 			= 1,
						GYRO_INIT_DLPF_SEND_D0 				= 2,
						GYRO_INIT_DLPF_WAIT_D0 				= 3,
						GYRO_INIT_DLPF_SEND_16 				= 4,
						GYRO_INIT_DLPF_WAIT_16 				= 5,
						GYRO_INIT_DLPF_SEND_18 				= 6,
						GYRO_INIT_DLPF_WAIT_18 				= 7, 
						GYRO_INIT_DLPF_STOP 					= 8,
						GYRO_INIT_DLPF_WAIT_STOP 			= 9,
						
						GYRO_INIT_INTRPT_CFG_START 		= 10,
						GYRO_INIT_INTRPT_CFG_WAIT_START 	= 11,
						GYRO_INIT_INTRPT_CFG_SEND_D0 		= 12,
						GYRO_INIT_INTRPT_CFG_WAIT_D0 		= 13,
						GYRO_INIT_INTRPT_CFG_SEND_13 		= 14,
						GYRO_INIT_INTRPT_CFG_WAIT_13 		= 15,
						GYRO_INIT_INTRPT_CFG_SEND_07 		= 16,
						GYRO_INIT_INTRPT_CFG_WAIT_07 		= 17,
						GYRO_INIT_INTRPT_CFG_STOP 			= 18,
						GYRO_INIT_INTRPT_CFG_WAIT_STOP 	= 19,
//================ACCL INIT===========================
						ACCL_INIT_DLPF_START					= 20,
						ACCL_INIT_DLPF_START_WAIT 			= 21,
						ACCL_INIT_DLPF_SEND_A6 				= 22,
						ACCL_INIT_DLPF_WAIT_A6 				= 23,
						ACCL_INIT_DLPF_SEND_16 				= 24,
						ACCL_INIT_DLPF_WAIT_16 				= 25,
						ACCL_INIT_DLPF_SEND_10 				= 26,
						ACCL_INIT_DLPF_WAIT_10 				= 27, 
						ACCL_INIT_DLPF_STOP 					= 28,
						ACCL_INIT_DLPF_WAIT_STOP 			= 29,
						
						ACCL_INIT_INTRPT_CFG_START 		= 30,
						ACCL_INIT_INTRPT_CFG_WAIT_START 	= 31,
						ACCL_INIT_INTRPT_CFG_SEND_A6 		= 32,
						ACCL_INIT_INTRPT_CFG_WAIT_A6 		= 33,
						ACCL_INIT_INTRPT_CFG_SEND_13 		= 34,
						ACCL_INIT_INTRPT_CFG_WAIT_13 		= 35,
						ACCL_INIT_INTRPT_CFG_SEND_07 		= 36,
						ACCL_INIT_INTRPT_CFG_WAIT_07 		= 37,
						ACCL_INIT_INTRPT_CFG_STOP 			= 38,
						ACCL_INIT_INTRPT_CFG_WAIT_STOP 	= 39,
						
//================MAGM INIT===========================
						MAGM_INIT_CRA_START					= 40,
						MAGM_INIT_CRA_START_WAIT 			= 41,
						MAGM_INIT_CRA_SEND_3C 				= 42,
						MAGM_INIT_CRA_WAIT_3C 				= 43,
						MAGM_INIT_CRA_SEND_00 				= 44,
						MAGM_INIT_CRA_WAIT_00 				= 45,
						MAGM_INIT_CRA_SEND_18 				= 46,
						MAGM_INIT_CRA_WAIT_18 				= 47, 
						MAGM_INIT_CRA_STOP 					= 48,
						MAGM_INIT_CRA_WAIT_STOP 			= 49,
						
						MAGM_INIT_MR_START 					= 50,
						MAGM_INIT_MR_WAIT_START 			= 51,
						MAGM_INIT_MR_SEND_3C 				= 52,
						MAGM_INIT_MR_WAIT_3C 				= 53,
						MAGM_INIT_MR_SEND_02 				= 54,
						MAGM_INIT_MR_WAIT_02 				= 55,
						MAGM_INIT_MR_SEND_00 				= 56,
						MAGM_INIT_MR_WAIT_00 				= 57,
						MAGM_INIT_MR_STOP 					= 58,
						MAGM_INIT_MR_WAIT_STOP 				= 59,
//================GYRO READ===========================						
						GYRO_T_MSB_START						= 60,
						GYRO_T_MSB_WAIT_START 				= 61,
						GYRO_T_MSB_SEND_D0 					= 62,
						GYRO_T_MSB_WAIT_D0 					= 63,
						GYRO_T_MSB_SEND_1B 					= 64,
						GYRO_T_MSB_WAIT_1B 					= 65,
						GYRO_T_MSB_RSTART 					= 66,
						GYRO_T_MSB_WAIT_RSTART 				= 67,
						GYRO_T_MSB_SEND_D1 					= 68,
						GYRO_T_MSB_WAIT_D1 					= 69,
						GYRO_T_MSB_READ 						= 70,
						GYRO_T_MSB_WAIT_READ 				= 71,
						GYRO_T_MSB_STOP 						= 72,
						GYRO_T_MSB_WAIT_STOP 				= 73,
							
						GYRO_T_LSB_START						= 74,
						GYRO_T_LSB_WAIT_START 				= 75,
						GYRO_T_LSB_SEND_D0 					= 76,
						GYRO_T_LSB_WAIT_D0 					= 77,
						GYRO_T_LSB_SEND_1C 					= 78,
						GYRO_T_LSB_WAIT_1C 					= 79,
						GYRO_T_LSB_RSTART 					= 80,
						GYRO_T_LSB_WAIT_RSTART 				= 81,
						GYRO_T_LSB_SEND_D1 					= 82,
						GYRO_T_LSB_WAIT_D1 					= 83,
						GYRO_T_LSB_READ 						= 84,
						GYRO_T_LSB_WAIT_READ 				= 85,
						GYRO_T_LSB_STOP 						= 86,
						GYRO_T_LSB_WAIT_STOP 				= 87,
							
						GYRO_X_MSB_START						= 88,
						GYRO_X_MSB_WAIT_START 				= 89,
						GYRO_X_MSB_SEND_D0 					= 90,
						GYRO_X_MSB_WAIT_D0 					= 91,
						GYRO_X_MSB_SEND_1D 					= 92,
						GYRO_X_MSB_WAIT_1D 					= 93,
						GYRO_X_MSB_RSTART 					= 94,
						GYRO_X_MSB_WAIT_RSTART 				= 95,
						GYRO_X_MSB_SEND_D1 					= 96,
						GYRO_X_MSB_WAIT_D1 					= 97,
						GYRO_X_MSB_READ 						= 98,
						GYRO_X_MSB_WAIT_READ 				= 99,
						GYRO_X_MSB_STOP 						= 100,
						GYRO_X_MSB_WAIT_STOP 				= 101,
							
						GYRO_X_LSB_START						= 102,
						GYRO_X_LSB_WAIT_START 				= 103,
						GYRO_X_LSB_SEND_D0 					= 104,
						GYRO_X_LSB_WAIT_D0 					= 105,
						GYRO_X_LSB_SEND_1E 					= 106,
						GYRO_X_LSB_WAIT_1E 					= 107,
						GYRO_X_LSB_RSTART 					= 108,
						GYRO_X_LSB_WAIT_RSTART 				= 109,
						GYRO_X_LSB_SEND_D1 					= 110,
						GYRO_X_LSB_WAIT_D1 					= 111,
						GYRO_X_LSB_READ 						= 112,
						GYRO_X_LSB_WAIT_READ 				= 113,
						GYRO_X_LSB_STOP 						= 114,
						GYRO_X_LSB_WAIT_STOP 				= 115,
							
						GYRO_Y_MSB_START						= 116,
						GYRO_Y_MSB_WAIT_START 				= 117,
						GYRO_Y_MSB_SEND_D0 					= 118,
						GYRO_Y_MSB_WAIT_D0 					= 119,
						GYRO_Y_MSB_SEND_1F 					= 120,
						GYRO_Y_MSB_WAIT_1F 					= 121,
						GYRO_Y_MSB_RSTART 					= 122,
						GYRO_Y_MSB_WAIT_RSTART 				= 123,
						GYRO_Y_MSB_SEND_D1 					= 124,
						GYRO_Y_MSB_WAIT_D1 					= 125,
						GYRO_Y_MSB_READ 						= 126,
						GYRO_Y_MSB_WAIT_READ 				= 127,
						GYRO_Y_MSB_STOP 						= 128,
						GYRO_Y_MSB_WAIT_STOP 				= 129,
							
						GYRO_Y_LSB_START						= 130,
						GYRO_Y_LSB_WAIT_START 				= 131,
						GYRO_Y_LSB_SEND_D0 					= 132,
						GYRO_Y_LSB_WAIT_D0 					= 133,
						GYRO_Y_LSB_SEND_20 					= 134,
						GYRO_Y_LSB_WAIT_20 					= 135,
						GYRO_Y_LSB_RSTART 					= 136,
						GYRO_Y_LSB_WAIT_RSTART 				= 137,
						GYRO_Y_LSB_SEND_D1 					= 138,
						GYRO_Y_LSB_WAIT_D1 					= 139,
						GYRO_Y_LSB_READ 						= 140,
						GYRO_Y_LSB_WAIT_READ 				= 141,
						GYRO_Y_LSB_STOP 						= 142,
						GYRO_Y_LSB_WAIT_STOP 				= 143,
							
						GYRO_Z_MSB_START						= 144,
						GYRO_Z_MSB_WAIT_START 				= 145,
						GYRO_Z_MSB_SEND_D0 					= 146,
						GYRO_Z_MSB_WAIT_D0 					= 147,
						GYRO_Z_MSB_SEND_21 					= 148,
						GYRO_Z_MSB_WAIT_21 					= 149,
						GYRO_Z_MSB_RSTART 					= 150,
						GYRO_Z_MSB_WAIT_RSTART 				= 151,
						GYRO_Z_MSB_SEND_D1 					= 152,
						GYRO_Z_MSB_WAIT_D1 					= 153,
						GYRO_Z_MSB_READ 						= 154,
						GYRO_Z_MSB_WAIT_READ 				= 155,
						GYRO_Z_MSB_STOP 						= 156,
						GYRO_Z_MSB_WAIT_STOP 				= 157,
							
						GYRO_Z_LSB_START						= 158,
						GYRO_Z_LSB_WAIT_START 				= 159,
						GYRO_Z_LSB_SEND_D0 					= 160,
						GYRO_Z_LSB_WAIT_D0 					= 161,
						GYRO_Z_LSB_SEND_22 					= 162,
						GYRO_Z_LSB_WAIT_22 					= 163,
						GYRO_Z_LSB_RSTART 					= 164,
						GYRO_Z_LSB_WAIT_RSTART 				= 165,
						GYRO_Z_LSB_SEND_D1 					= 166,
						GYRO_Z_LSB_WAIT_D1 					= 167,
						GYRO_Z_LSB_READ 						= 168,
						GYRO_Z_LSB_WAIT_READ 				= 169,
						GYRO_Z_LSB_STOP 						= 170,
						GYRO_Z_LSB_WAIT_STOP 				= 171,
						
//================ACCL READ===========================
						ACCL_X_MSB_START						= 172,
						ACCL_X_MSB_WAIT_START 				= 173,
						ACCL_X_MSB_SEND_A6 					= 174,
						ACCL_X_MSB_WAIT_A6 					= 175,
						ACCL_X_MSB_SEND_50 					= 176,
						ACCL_X_MSB_WAIT_50 					= 177,
						ACCL_X_MSB_RSTART 					= 178,
						ACCL_X_MSB_WAIT_RSTART 				= 179,
						ACCL_X_MSB_SEND_A7 					= 180,
						ACCL_X_MSB_WAIT_A7 					= 181,
						ACCL_X_MSB_READ 						= 182,
						ACCL_X_MSB_WAIT_READ 				= 183,
						ACCL_X_MSB_STOP 						= 184,
						ACCL_X_MSB_WAIT_STOP 				= 185,
							
						ACCL_X_LSB_START						= 186,
						ACCL_X_LSB_WAIT_START 				= 187,
						ACCL_X_LSB_SEND_A6 					= 188,
						ACCL_X_LSB_WAIT_A6 					= 189,
						ACCL_X_LSB_SEND_51 					= 190,
						ACCL_X_LSB_WAIT_51 					= 191,
						ACCL_X_LSB_RSTART 					= 192,
						ACCL_X_LSB_WAIT_RSTART 				= 193,
						ACCL_X_LSB_SEND_A7 					= 194,
						ACCL_X_LSB_WAIT_A7 					= 195,
						ACCL_X_LSB_READ 						= 196,
						ACCL_X_LSB_WAIT_READ 				= 197,
						ACCL_X_LSB_STOP 						= 198,
						ACCL_X_LSB_WAIT_STOP 				= 199,
							
						ACCL_Y_MSB_START						= 200,
						ACCL_Y_MSB_WAIT_START 				= 201,
						ACCL_Y_MSB_SEND_A6 					= 202,
						ACCL_Y_MSB_WAIT_A6 					= 203,
						ACCL_Y_MSB_SEND_52 					= 204,
						ACCL_Y_MSB_WAIT_52 					= 205,
						ACCL_Y_MSB_RSTART 					= 206,
						ACCL_Y_MSB_WAIT_RSTART 				= 207,
						ACCL_Y_MSB_SEND_A7 					= 208,
						ACCL_Y_MSB_WAIT_A7 					= 209,
						ACCL_Y_MSB_READ 						= 210,
						ACCL_Y_MSB_WAIT_READ 				= 211,
						ACCL_Y_MSB_STOP 						= 212,
						ACCL_Y_MSB_WAIT_STOP 				= 213,
							
						ACCL_Y_LSB_START						= 214,
						ACCL_Y_LSB_WAIT_START 				= 215,
						ACCL_Y_LSB_SEND_A6 					= 216,
						ACCL_Y_LSB_WAIT_A6 					= 217,
						ACCL_Y_LSB_SEND_53 					= 218,
						ACCL_Y_LSB_WAIT_53 					= 219,
						ACCL_Y_LSB_RSTART 					= 220,
						ACCL_Y_LSB_WAIT_RSTART 				= 221,
						ACCL_Y_LSB_SEND_A7 					= 222,
						ACCL_Y_LSB_WAIT_A7 					= 223,
						ACCL_Y_LSB_READ 						= 224,
						ACCL_Y_LSB_WAIT_READ 				= 225,
						ACCL_Y_LSB_STOP 						= 226,
						ACCL_Y_LSB_WAIT_STOP 				= 227,
							
						ACCL_Z_MSB_START						= 228,
						ACCL_Z_MSB_WAIT_START 				= 229,
						ACCL_Z_MSB_SEND_A6 					= 230,
						ACCL_Z_MSB_WAIT_A6 					= 231,
						ACCL_Z_MSB_SEND_54 					= 232,
						ACCL_Z_MSB_WAIT_54 					= 233,
						ACCL_Z_MSB_RSTART 					= 234,
						ACCL_Z_MSB_WAIT_RSTART 				= 235,
						ACCL_Z_MSB_SEND_A7 					= 236,
						ACCL_Z_MSB_WAIT_A7 					= 237,
						ACCL_Z_MSB_READ 						= 238,
						ACCL_Z_MSB_WAIT_READ 				= 239,
						ACCL_Z_MSB_STOP 						= 240,
						ACCL_Z_MSB_WAIT_STOP 				= 241,
							
						ACCL_Z_LSB_START						= 242,
						ACCL_Z_LSB_WAIT_START 				= 243,
						ACCL_Z_LSB_SEND_A6 					= 244,
						ACCL_Z_LSB_WAIT_A6 					= 245,
						ACCL_Z_LSB_SEND_55 					= 246,
						ACCL_Z_LSB_WAIT_55 					= 247,
						ACCL_Z_LSB_RSTART 					= 248,
						ACCL_Z_LSB_WAIT_RSTART 				= 249,
						ACCL_Z_LSB_SEND_A7 					= 250,
						ACCL_Z_LSB_WAIT_A7 					= 251,
						ACCL_Z_LSB_READ 						= 252,
						ACCL_Z_LSB_WAIT_READ 				= 253,
						ACCL_Z_LSB_STOP 						= 254,
						ACCL_Z_LSB_WAIT_STOP 				= 255,
//================MAGM READ===========================				
						MAGM_X_MSB_START						= 256,
						MAGM_X_MSB_WAIT_START 				= 257,
						MAGM_X_MSB_SEND_3C 					= 258,
						MAGM_X_MSB_WAIT_3C 					= 259,
						MAGM_X_MSB_SEND_03 					= 260,
						MAGM_X_MSB_WAIT_03 					= 261,
						MAGM_X_MSB_RSTART 					= 262,
						MAGM_X_MSB_WAIT_RSTART 				= 263,
						MAGM_X_MSB_SEND_3D 					= 264,
						MAGM_X_MSB_WAIT_3D 					= 265,
						MAGM_X_MSB_READ 						= 266,
						MAGM_X_MSB_WAIT_READ 				= 267,
						MAGM_X_MSB_STOP 						= 268,
						MAGM_X_MSB_WAIT_STOP 				= 269,
							
						MAGM_X_LSB_START						= 270,
						MAGM_X_LSB_WAIT_START 				= 271,
						MAGM_X_LSB_SEND_3C 					= 272,
						MAGM_X_LSB_WAIT_3C 					= 273,
						MAGM_X_LSB_SEND_04 					= 274,
						MAGM_X_LSB_WAIT_04 					= 275,
						MAGM_X_LSB_RSTART 					= 276,
						MAGM_X_LSB_WAIT_RSTART 				= 277,
						MAGM_X_LSB_SEND_3D 					= 278,
						MAGM_X_LSB_WAIT_3D 					= 279,
						MAGM_X_LSB_READ 						= 280,
						MAGM_X_LSB_WAIT_READ 				= 281,
						MAGM_X_LSB_STOP 						= 282,
						MAGM_X_LSB_WAIT_STOP 				= 283,
							
						MAGM_Z_MSB_START						= 284,
						MAGM_Z_MSB_WAIT_START 				= 285,
						MAGM_Z_MSB_SEND_3C 					= 286,
						MAGM_Z_MSB_WAIT_3C 					= 287,
						MAGM_Z_MSB_SEND_05 					= 288,
						MAGM_Z_MSB_WAIT_05 					= 289,
						MAGM_Z_MSB_RSTART 					= 290,
						MAGM_Z_MSB_WAIT_RSTART 				= 291,
						MAGM_Z_MSB_SEND_3D 					= 292,
						MAGM_Z_MSB_WAIT_3D 					= 293,
						MAGM_Z_MSB_READ 						= 294,
						MAGM_Z_MSB_WAIT_READ 				= 295,
						MAGM_Z_MSB_STOP 						= 296,
						MAGM_Z_MSB_WAIT_STOP 				= 297,
							
						MAGM_Z_LSB_START						= 298,
						MAGM_Z_LSB_WAIT_START 				= 299,
						MAGM_Z_LSB_SEND_3C 					= 300,
						MAGM_Z_LSB_WAIT_3C 					= 301,
						MAGM_Z_LSB_SEND_06 					= 302,
						MAGM_Z_LSB_WAIT_06 					= 303,
						MAGM_Z_LSB_RSTART 					= 304,
						MAGM_Z_LSB_WAIT_RSTART 				= 305,
						MAGM_Z_LSB_SEND_3D 					= 306,
						MAGM_Z_LSB_WAIT_3D 					= 307,
						MAGM_Z_LSB_READ 						= 308,
						MAGM_Z_LSB_WAIT_READ 				= 309,
						MAGM_Z_LSB_STOP 						= 310,
						MAGM_Z_LSB_WAIT_STOP 				= 311,
							
						MAGM_Y_MSB_START						= 312,
						MAGM_Y_MSB_WAIT_START 				= 313,
						MAGM_Y_MSB_SEND_3C 					= 314,
						MAGM_Y_MSB_WAIT_3C 					= 315,
						MAGM_Y_MSB_SEND_07 					= 316,
						MAGM_Y_MSB_WAIT_07 					= 317,
						MAGM_Y_MSB_RSTART 					= 318,
						MAGM_Y_MSB_WAIT_RSTART 				= 319,
						MAGM_Y_MSB_SEND_3D 					= 320,
						MAGM_Y_MSB_WAIT_3D 					= 321,
						MAGM_Y_MSB_READ 						= 322,
						MAGM_Y_MSB_WAIT_READ 				= 323,
						MAGM_Y_MSB_STOP 						= 324,
						MAGM_Y_MSB_WAIT_STOP 				= 325,
							
						MAGM_Y_LSB_START						= 326,
						MAGM_Y_LSB_WAIT_START 				= 327,
						MAGM_Y_LSB_SEND_3C 					= 328,
						MAGM_Y_LSB_WAIT_3C 					= 329,
						MAGM_Y_LSB_SEND_08 					= 330,
						MAGM_Y_LSB_WAIT_08 					= 331,
						MAGM_Y_LSB_RSTART 					= 332,
						MAGM_Y_LSB_WAIT_RSTART 				= 333,
						MAGM_Y_LSB_SEND_3D 					= 334,
						MAGM_Y_LSB_WAIT_3D 					= 335,
						MAGM_Y_LSB_READ 						= 336,
						MAGM_Y_LSB_WAIT_READ 				= 337,
						MAGM_Y_LSB_STOP 						= 338,
						MAGM_Y_LSB_WAIT_STOP 				= 339;
						
//reg accelerometer_register = 8'h32;
//reg magnetometer_register = 8'h03;
//reg gyro_register = 8'h1D;

reg [STATE_SIZE:0] state = GYRO_INIT_DLPF_START;

always @(posedge clk or posedge rst) begin
 
//START GYRO
	//accelerometer_register = 8'h32;
	//magnetometer_register = 8'h03; 
	//gyro_register = 8'h1D;
 
	if(rst) begin
		ena <= 0;
		state <= GYRO_INIT_DLPF_START;
		data_wr <= 8'h00;
		rw <= 1'b0;
		start_transfer <= 1'b0;
		stop_transfer <= 1'b0;
		GYRO_TEMP <= 16'd0;
		GYRO_X <= 16'd0;
		GYRO_Y <= 16'd0;
		GYRO_Z <= 16'd0;
		ACCL_X <= 16'd0;
		ACCL_Y <= 16'd0;
		ACCL_Z <= 16'd0;
		MAGM_X <= 16'd0;
		MAGM_Y <= 16'd0;
		MAGM_Z <= 16'd0;
	end else begin
		case(state)
//================		
			GYRO_INIT_DLPF_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_INIT_DLPF_START_WAIT;
				end else begin
					state<=GYRO_INIT_DLPF_START;
				end
			end 			
		 					
			GYRO_INIT_DLPF_START_WAIT:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_DLPF_SEND_D0;
				end else begin
					state <= GYRO_INIT_DLPF_START_WAIT;
				end
			end 			
		 	 		
			GYRO_INIT_DLPF_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_DLPF_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_DLPF_WAIT_D0;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_DLPF_SEND_16;
				end else begin
					state <= GYRO_INIT_DLPF_WAIT_D0;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_SEND_16:begin
				ena <= 1'b1;
				data_wr <= 8'h16;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_DLPF_SEND_16;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_DLPF_WAIT_16;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_WAIT_16:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_DLPF_SEND_18;
				end else begin
					state <= GYRO_INIT_DLPF_WAIT_16;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_SEND_18:begin
				ena <= 1'b1;
				data_wr <= 8'h18;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_DLPF_SEND_18;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_DLPF_WAIT_18;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_WAIT_18:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_DLPF_STOP;
				end else begin
					state <= GYRO_INIT_DLPF_WAIT_18;
				end
			end 			
		 	 			
			GYRO_INIT_DLPF_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_DLPF_STOP;
				end else begin
					state <= GYRO_INIT_DLPF_WAIT_STOP;
				end
			end 			
		 	 				
			GYRO_INIT_DLPF_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_START;
				end else begin
					state <= GYRO_INIT_DLPF_WAIT_STOP;
				end
			end 			
		 	 		
//================		
			GYRO_INIT_INTRPT_CFG_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_INIT_INTRPT_CFG_WAIT_START;
				end else begin
					state<=GYRO_INIT_INTRPT_CFG_START;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_SEND_D0;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_START;
				end
			end 			
		 	 
			GYRO_INIT_INTRPT_CFG_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_INTRPT_CFG_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_INTRPT_CFG_WAIT_D0;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_SEND_13;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_D0;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_SEND_13:begin
				ena <= 1'b1;
				data_wr <= 8'h13;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_INTRPT_CFG_SEND_13;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_INTRPT_CFG_WAIT_13;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_WAIT_13:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_SEND_07;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_13;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_SEND_07:begin
				ena <= 1'b1;
				data_wr <= 8'h07;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_INIT_INTRPT_CFG_SEND_07;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_INIT_INTRPT_CFG_WAIT_07;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_WAIT_07:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_STOP;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_07;
				end
			end 			
		 	 	
			GYRO_INIT_INTRPT_CFG_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_INIT_INTRPT_CFG_STOP;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_STOP;
				end
			end 			
		 	 		
			GYRO_INIT_INTRPT_CFG_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_START;
				end else begin
					state <= GYRO_INIT_INTRPT_CFG_WAIT_STOP;
				end
			end 			
		 	 
//=======ACCL INIT=======================
			ACCL_INIT_DLPF_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_INIT_DLPF_START_WAIT;
				end else begin
					state<=ACCL_INIT_DLPF_START;
				end
			end 			
		 					
			ACCL_INIT_DLPF_START_WAIT:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_SEND_A6;
				end else begin
					state <= ACCL_INIT_DLPF_START_WAIT;
				end
			end 			
		 	 		
			ACCL_INIT_DLPF_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_DLPF_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_DLPF_WAIT_A6;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_SEND_16;
				end else begin
					state <= ACCL_INIT_DLPF_WAIT_A6;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_SEND_16:begin
				ena <= 1'b1;
				data_wr <= 8'h16;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_DLPF_SEND_16;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_DLPF_WAIT_16;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_WAIT_16:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_SEND_10;
				end else begin
					state <= ACCL_INIT_DLPF_WAIT_16;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_SEND_10:begin
				ena <= 1'b1;
				data_wr <= 8'h10;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_DLPF_SEND_10;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_DLPF_WAIT_10;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_WAIT_10:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_STOP;
				end else begin
					state <= ACCL_INIT_DLPF_WAIT_10;
				end
			end 			
		 	 			
			ACCL_INIT_DLPF_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_DLPF_STOP;
				end else begin
					state <= ACCL_INIT_DLPF_WAIT_STOP;
				end
			end 			
		 	 				
			ACCL_INIT_DLPF_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_START;
				end else begin
					state <= ACCL_INIT_DLPF_WAIT_STOP;
				end
			end 			
		 	 		
//================		
			ACCL_INIT_INTRPT_CFG_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_INIT_INTRPT_CFG_WAIT_START;
				end else begin
					state<=ACCL_INIT_INTRPT_CFG_START;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_SEND_A6;
				end else begin
					state <= ACCL_INIT_INTRPT_CFG_WAIT_START;
				end
			end 			
		 	
			ACCL_INIT_INTRPT_CFG_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_INTRPT_CFG_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_INTRPT_CFG_WAIT_A6;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_SEND_13;
				end else begin
					state <= ACCL_INIT_INTRPT_CFG_WAIT_A6;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_SEND_13:begin
				ena <= 1'b1;
				data_wr <= 8'h13;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_INTRPT_CFG_SEND_13;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_INTRPT_CFG_WAIT_13;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_WAIT_13:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_SEND_07;
				end else begin
					state <= ACCL_INIT_INTRPT_CFG_WAIT_13;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_SEND_07:begin
				ena <= 1'b1;
				data_wr <= 8'h07;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_INIT_INTRPT_CFG_SEND_07;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_INIT_INTRPT_CFG_WAIT_07;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_WAIT_07:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_STOP;
				end else begin
					state <=ACCL_INIT_INTRPT_CFG_WAIT_07;
				end
			end 			
		 	 	
			ACCL_INIT_INTRPT_CFG_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_INIT_INTRPT_CFG_STOP;
				end else begin
					state <= ACCL_INIT_INTRPT_CFG_WAIT_STOP;
				end
			end 			
		 	 		
			ACCL_INIT_INTRPT_CFG_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_START;
				end else begin
					state <= ACCL_INIT_INTRPT_CFG_WAIT_STOP;
				end
			end 			
		 	 
		
//=======MAGM INIT=======================
			MAGM_INIT_CRA_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_INIT_CRA_START_WAIT;
				end else begin
					state<=MAGM_INIT_CRA_START;
				end
			end 			
		 					
			MAGM_INIT_CRA_START_WAIT:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_SEND_3C;
				end else begin
					state <= MAGM_INIT_CRA_START_WAIT;
				end
			end 			
		 	 		
			MAGM_INIT_CRA_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_CRA_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_CRA_WAIT_3C;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_SEND_00;
				end else begin
					state <= MAGM_INIT_CRA_WAIT_3C;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_SEND_00:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_CRA_SEND_00;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_CRA_WAIT_00;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_WAIT_00:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_SEND_18;
				end else begin
					state <= MAGM_INIT_CRA_WAIT_00;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_SEND_18:begin
				ena <= 1'b1;
				data_wr <= 8'h18;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_CRA_SEND_18;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_CRA_WAIT_18;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_WAIT_18:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_STOP;
				end else begin
					state <= MAGM_INIT_CRA_WAIT_18;
				end
			end 			
		 	 			
			MAGM_INIT_CRA_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_CRA_STOP;
				end else begin
					state <= MAGM_INIT_CRA_WAIT_STOP;
				end
			end 			
		 	 				
			MAGM_INIT_CRA_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_START;
				end else begin
					state <= MAGM_INIT_CRA_WAIT_STOP;
				end
			end 			
		 	 		
//================		
			MAGM_INIT_MR_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_INIT_MR_WAIT_START;
				end else begin
					state<=MAGM_INIT_MR_START;
				end
			end 			
		 	 				
			MAGM_INIT_MR_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_SEND_3C;
				end else begin
					state <= MAGM_INIT_MR_WAIT_START;
				end
			end 			
		 	 		
			MAGM_INIT_MR_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_MR_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_MR_WAIT_3C;
				end
			end 			
		 	 			
			MAGM_INIT_MR_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_SEND_02;
				end else begin
					state <= MAGM_INIT_MR_WAIT_3C;
				end
			end 			
		 	 			
			MAGM_INIT_MR_SEND_02:begin
				ena <= 1'b1;
				data_wr <= 8'h02;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_MR_SEND_02;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_MR_WAIT_02;
				end
			end 			
		 	 			
			MAGM_INIT_MR_WAIT_02:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_SEND_00;
				end else begin
					state <= MAGM_INIT_MR_WAIT_02;
				end
			end 			
		 	 			
			MAGM_INIT_MR_SEND_00:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_INIT_MR_SEND_00;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_INIT_MR_WAIT_00;
				end
			end 			
		 	 			
			MAGM_INIT_MR_WAIT_00:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_STOP;
				end else begin
					state <= MAGM_INIT_MR_WAIT_00;
				end
			end 			
		 	 			
			MAGM_INIT_MR_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_INIT_MR_STOP;
				end else begin
					state <= MAGM_INIT_MR_WAIT_STOP;
				end
			end 			
		 	 				
			MAGM_INIT_MR_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_START;
				end else begin
					state <= MAGM_INIT_MR_WAIT_STOP;
				end
			end 			
		 	 			
//=======GYRO READ=======================	
			GYRO_T_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_T_MSB_WAIT_START;
				end else begin
					state<=GYRO_T_MSB_START;
				end
			end 			
		 						
			GYRO_T_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_SEND_D0;
				end else begin
					state <= GYRO_T_MSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_T_MSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_MSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_T_MSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_SEND_1B;
				end else begin
					state <= GYRO_T_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_T_MSB_SEND_1B:begin
				ena <= 1'b1;
				data_wr <= 8'h1B;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_MSB_SEND_1B;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_MSB_WAIT_1B;
				end
			end 			
		 	 				
			GYRO_T_MSB_WAIT_1B:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_RSTART;
				end else begin
					state <= GYRO_T_MSB_WAIT_1B;
				end
			end 			
		 	 				
			GYRO_T_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_T_MSB_RSTART;
				end else begin
					state <= GYRO_T_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_T_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_SEND_D1;
				end else begin
					state <= GYRO_T_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_T_MSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_MSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_T_MSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_READ;
				end else begin
					state <= GYRO_T_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_T_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_MSB_READ;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_MSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_T_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_TEMP[15:8] <= data_rd;
					debug <= data_rd;
					state <= GYRO_T_MSB_STOP;
				end else begin
					state <= GYRO_T_MSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_T_MSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_STOP;
				end else begin
					state <= GYRO_T_MSB_WAIT_STOP;
				end
			end 			
		 	 					
			GYRO_T_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_START;
				end else begin
					state <= GYRO_T_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_T_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_T_LSB_WAIT_START;
				end else begin
					state<=GYRO_T_LSB_START;
				end
			end 			
		 						
			GYRO_T_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_SEND_D0;
				end else begin
					state <= GYRO_T_LSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_T_LSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_LSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_T_LSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_SEND_1C;
				end else begin
					state <= GYRO_T_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_T_LSB_SEND_1C:begin
				ena <= 1'b1;
				data_wr <= 8'h1C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_LSB_SEND_1C;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_LSB_WAIT_1C;
				end
			end 			
		 	 				
			GYRO_T_LSB_WAIT_1C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_RSTART;
				end else begin
					state <= GYRO_T_LSB_WAIT_1C;
				end
			end 			
		 	 				
			GYRO_T_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_T_LSB_RSTART;
				end else begin
					state <= GYRO_T_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_T_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_SEND_D1;
				end else begin
					state <= GYRO_T_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_T_LSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_LSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_T_LSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_READ;
				end else begin
					state <= GYRO_T_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_T_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;				
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					start_transfer <= 1'b1;
					state <= GYRO_T_LSB_READ;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_T_LSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_T_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_TEMP[7:0] <= data_rd;
					debug <= data_rd;
					state <= GYRO_T_LSB_STOP;
				end else begin
					state <= GYRO_T_LSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_T_LSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_STOP;
				end else begin
					state <= GYRO_T_LSB_WAIT_STOP;
				end
			end 			
		 	 					
			GYRO_T_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_LSB_WAIT_STOP;
				end else begin
					state <= GYRO_X_MSB_START;
				end
			end 			
		 	 			
//================			
			GYRO_X_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_X_MSB_WAIT_START;
				end else begin
					state<=GYRO_X_MSB_START;
				end
			end 			
		 						
			GYRO_X_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_SEND_D0;
				end else begin
					state <= GYRO_X_MSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_X_MSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_MSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_X_MSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_SEND_1D;
				end else begin
					state <= GYRO_X_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_X_MSB_SEND_1D:begin
				ena <= 1'b1;
				data_wr <= 8'h1D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_MSB_SEND_1D;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_MSB_WAIT_1D;
				end
			end 			
		 	 				
			GYRO_X_MSB_WAIT_1D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_RSTART;
				end else begin
					state <= GYRO_X_MSB_WAIT_1D;
				end
			end 			
		 	 				
			GYRO_X_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_X_MSB_RSTART;
				end else begin
					state <= GYRO_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_X_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_SEND_D1;
				end else begin
					state <= GYRO_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_X_MSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_MSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_X_MSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_READ;
				end else begin
					state <= GYRO_X_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_X_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_READ;
					start_transfer <= 1'b1;
				end else begin
					state <= GYRO_X_MSB_WAIT_READ;
					start_transfer <= 1'b0;
				end
			end 			
		 	 					
			GYRO_X_MSB_WAIT_READ:begin 
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_X[15:8] <= data_rd;
					debug <= data_rd;
					state <= GYRO_X_MSB_STOP;
				end else begin
					state <= GYRO_X_MSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_X_MSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_MSB_STOP;
				end else begin
					state <= GYRO_X_MSB_WAIT_STOP;
				end
			end 			
		 	 					
			GYRO_X_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_START;
				end else begin
					state <= GYRO_X_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_X_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_X_LSB_WAIT_START;
				end else begin
					state<=GYRO_X_LSB_START;
				end
			end 			
		 						
			GYRO_X_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_SEND_D0;
				end else begin
					state <= GYRO_X_LSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_X_LSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_LSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_X_LSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_SEND_1E;
				end else begin
					state <= GYRO_X_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_X_LSB_SEND_1E:begin
				ena <= 1'b1;
				data_wr <= 8'h1E;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_LSB_SEND_1E;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_LSB_WAIT_1E;
				end
			end 			
		 	 				
			GYRO_X_LSB_WAIT_1E:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_RSTART;
				end else begin
					state <= GYRO_X_LSB_WAIT_1E;
				end
			end 			
		 	 				
			GYRO_X_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_X_LSB_RSTART;
				end else begin
					state <= GYRO_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_X_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_SEND_D1;
				end else begin
					state <= GYRO_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_X_LSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_X_LSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_X_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_X_LSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_READ;
				end else begin
					state <= GYRO_X_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_X_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_READ;
				end else begin
					state <= GYRO_X_LSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_X_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_X[7:0] <= data_rd;
					debug <= data_rd;
					state <= GYRO_X_LSB_STOP;
				end else begin
					state <= GYRO_X_LSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_X_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_X_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= GYRO_X_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			GYRO_X_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_START;
				end else begin
					state <= GYRO_X_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_Y_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_Y_MSB_WAIT_START;
				end else begin
					state<=GYRO_Y_MSB_START;
				end
			end 			
		 						
			GYRO_Y_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_SEND_D0;
				end else begin
					state <= GYRO_Y_MSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_Y_MSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_MSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Y_MSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_SEND_1F;
				end else begin
					state <= GYRO_Y_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Y_MSB_SEND_1F:begin
				ena <= 1'b1;
				data_wr <= 8'h1F;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_MSB_SEND_1F;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_MSB_WAIT_1F;
				end
			end 			
		 	 				
			GYRO_Y_MSB_WAIT_1F:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_RSTART;
				end else begin
					state <= GYRO_Y_MSB_WAIT_1F;
				end
			end 			
		 	 				
			GYRO_Y_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_Y_MSB_RSTART;
				end else begin
					state <= GYRO_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_Y_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_SEND_D1;
				end else begin
					state <= GYRO_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_Y_MSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_MSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Y_MSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_READ;
				end else begin
					state <= GYRO_Y_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Y_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_READ;
				end else begin
					state <= GYRO_Y_MSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_Y_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_Y[15:8] <= data_rd;
					state <= GYRO_Y_MSB_STOP;
				end else begin
					state <= GYRO_Y_MSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_Y_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= GYRO_Y_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			GYRO_Y_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_START;
				end else begin
					state <= GYRO_Y_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_Y_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_Y_LSB_WAIT_START;
				end else begin
					state<=GYRO_Y_LSB_START;
				end
			end 			
		 						
			GYRO_Y_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_SEND_D0;
				end else begin
					state <= GYRO_Y_LSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_Y_LSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_LSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Y_LSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_SEND_20;
				end else begin
					state <= GYRO_Y_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Y_LSB_SEND_20:begin
				ena <= 1'b1;
				data_wr <= 8'h20;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_LSB_SEND_20;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_LSB_WAIT_20;
				end
			end 			
		 	 				
			GYRO_Y_LSB_WAIT_20:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_RSTART;
				end else begin
					state <= GYRO_Y_LSB_WAIT_20;
				end
			end 			
		 	 				
			GYRO_Y_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_Y_LSB_RSTART;
				end else begin
					state <= GYRO_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_Y_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_SEND_D1;
				end else begin
					state <= GYRO_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_Y_LSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Y_LSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Y_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Y_LSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_READ;
				end else begin
					state <= GYRO_Y_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Y_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_READ;
				end else begin
					state <= GYRO_Y_LSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_Y_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_Y[7:0] <= data_rd;
					state <= GYRO_Y_LSB_STOP;
				end else begin
					state <= GYRO_Y_LSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_Y_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Y_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= GYRO_Y_LSB_WAIT_STOP;
					stop_transfer <= 1'b1;
				end
			end 			
		 	 					
			GYRO_Y_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_START;
				end else begin
					state <= GYRO_Y_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_Z_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_Z_MSB_WAIT_START;
				end else begin
					state<=GYRO_Z_MSB_START;
				end
			end 			
		 						
			GYRO_Z_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_SEND_D0;
				end else begin
					state <= GYRO_Z_MSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_Z_MSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_MSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Z_MSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_SEND_21;
				end else begin
					state <= GYRO_Z_MSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Z_MSB_SEND_21:begin
				ena <= 1'b1;
				data_wr <= 8'h21;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_MSB_SEND_21;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_MSB_WAIT_21;
				end
			end 			
		 	 				
			GYRO_Z_MSB_WAIT_21:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_RSTART;
				end else begin
					state <= GYRO_Z_MSB_WAIT_21;
				end
			end 			
		 	 				
			GYRO_Z_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_Z_MSB_RSTART;
				end else begin
					state <= GYRO_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_Z_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_SEND_D1;
				end else begin
					state <= GYRO_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_Z_MSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_MSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Z_MSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_READ;
				end else begin
					state <= GYRO_Z_MSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Z_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_READ;
				end else begin
					state <= GYRO_Z_MSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_Z_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_Z[15:8] <= data_rd;
					state <= GYRO_Z_MSB_STOP;
				end else begin
					state <= GYRO_Z_MSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_Z_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= GYRO_Z_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			GYRO_Z_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_START;
				end else begin
					state <= GYRO_Z_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			GYRO_Z_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=GYRO_Z_LSB_WAIT_START;
				end else begin
					state<=GYRO_Z_LSB_START;
				end
			end 			
		 						
			GYRO_Z_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_SEND_D0;
				end else begin
					state <= GYRO_Z_LSB_WAIT_START;
				end
			end 			
		 	 			
			GYRO_Z_LSB_SEND_D0:begin
				ena <= 1'b1;
				data_wr <= 8'hD0;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_LSB_SEND_D0;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Z_LSB_WAIT_D0:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_SEND_22;
				end else begin
					state <= GYRO_Z_LSB_WAIT_D0;
				end
			end 			
		 	 				
			GYRO_Z_LSB_SEND_22:begin
				ena <= 1'b1;
				data_wr <= 8'h22;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_LSB_SEND_22;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_LSB_WAIT_22;
				end
			end 			
		 	 				
			GYRO_Z_LSB_WAIT_22:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_RSTART;
				end else begin
					state <= GYRO_Z_LSB_WAIT_22;
				end
			end 			
		 	 				
			GYRO_Z_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= GYRO_Z_LSB_RSTART;
				end else begin
					state <= GYRO_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			GYRO_Z_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_SEND_D1;
				end else begin
					state <= GYRO_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			GYRO_Z_LSB_SEND_D1:begin
				ena <= 1'b1;
				data_wr <= 8'hD1;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= GYRO_Z_LSB_SEND_D1;
				end else begin
					start_transfer <= 1'b0;
					state <= GYRO_Z_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Z_LSB_WAIT_D1:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_READ;
				end else begin
					state <= GYRO_Z_LSB_WAIT_D1;
				end
			end 			
		 	 				
			GYRO_Z_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <=GYRO_Z_LSB_READ;
				end else begin
					state <= GYRO_Z_LSB_WAIT_READ;
				end
			end 			
		 	 					
			GYRO_Z_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					GYRO_Z[7:0] <= data_rd;
					state <=GYRO_Z_LSB_STOP;
				end else begin
					state <= GYRO_Z_LSB_WAIT_READ;
				end
			end 			
		 	 			
			GYRO_Z_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_Z_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= GYRO_Z_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			GYRO_Z_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_START;
				end else begin
					state <= GYRO_Z_LSB_WAIT_STOP;
				end
			end 			
		 	 			
		
//=======ACCL READ=======================
			ACCL_X_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_X_MSB_WAIT_START;
				end else begin
					state<=ACCL_X_MSB_START;
				end
			end 			
		 						
			ACCL_X_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_SEND_A6;
				end else begin
					state <= ACCL_X_MSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_X_MSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_MSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_X_MSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_SEND_50;
				end else begin
					state <= ACCL_X_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_X_MSB_SEND_50:begin
				ena <= 1'b1;
				data_wr <= 8'h50;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_MSB_SEND_50;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_MSB_WAIT_50;
				end
			end 			
		 	 				
			ACCL_X_MSB_WAIT_50:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_RSTART;
				end else begin
					state <= ACCL_X_MSB_WAIT_50;
				end
			end 			
		 	 				
			ACCL_X_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_X_MSB_RSTART;
				end else begin
					state <= ACCL_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_X_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_SEND_A7;
				end else begin
					state <= ACCL_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_X_MSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_MSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_MSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_X_MSB_WAIT_A7:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_READ;
				end else begin
					state <= ACCL_X_MSB_WAIT_A7;
				end
			end 			
		 					
			ACCL_X_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b0;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_READ;
				end else begin
					state <= ACCL_X_MSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_X_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_X[15:8] <= data_rd;
					state <= ACCL_X_MSB_STOP;
				end else begin
					state <= ACCL_X_MSB_WAIT_READ;
				end
			end 			
		 				
			ACCL_X_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_X_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_X_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_START;
				end else begin
					state <= ACCL_X_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			ACCL_X_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_X_LSB_WAIT_START;
				end else begin
					state<=ACCL_X_LSB_START;
				end
			end 			
		 						
			ACCL_X_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_SEND_A6;
				end else begin
					state <= ACCL_X_LSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_X_LSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_LSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_X_LSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_SEND_51;
				end else begin
					state <= ACCL_X_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_X_LSB_SEND_51:begin
				ena <= 1'b1;
				data_wr <= 8'h51;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_LSB_SEND_51;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_LSB_WAIT_51;
				end
			end 			
		 	 				
			ACCL_X_LSB_WAIT_51:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_RSTART;
				end else begin
					state <= ACCL_X_LSB_WAIT_51;
				end
			end 			
		 	 				
			ACCL_X_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_X_LSB_RSTART;
				end else begin
					state <= ACCL_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_X_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_SEND_A7;
				end else begin
					state <= ACCL_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_X_LSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_X_LSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_X_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_X_LSB_WAIT_A7:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_READ;
				end else begin
					state <= ACCL_X_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_X_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_READ;
				end else begin
					state <= ACCL_X_LSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_X_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_X[7:0] <= data_rd;
					state <= ACCL_X_LSB_STOP;
				end else begin
					state <= ACCL_X_LSB_WAIT_READ;
				end
			end 			
		 	 			
			ACCL_X_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_X_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_X_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_X_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_START;
				end else begin
					state <= ACCL_X_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			ACCL_Y_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_Y_MSB_WAIT_START;
				end else begin
					state<=ACCL_Y_MSB_START;
				end
			end 			
		 						
			ACCL_Y_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_SEND_A6;
				end else begin
					state <= ACCL_Y_MSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_Y_MSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_MSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Y_MSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_SEND_52;
				end else begin
					state <= ACCL_Y_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Y_MSB_SEND_52:begin
				ena <= 1'b1;
				data_wr <= 8'h52;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_MSB_SEND_52;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_MSB_WAIT_52;
				end
			end 			
		 	 				
			ACCL_Y_MSB_WAIT_52:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_RSTART;
				end else begin
					state <= ACCL_Y_MSB_WAIT_52;
				end
			end 			
		 	 				
			ACCL_Y_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_Y_MSB_RSTART;
				end else begin
					state <= ACCL_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_Y_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_SEND_A7;
				end else begin
					state <= ACCL_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_Y_MSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b1;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_MSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_MSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Y_MSB_WAIT_A7 :begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_READ;
				end else begin
					state <= ACCL_Y_MSB_WAIT_A7;
				end
			end 			
		 					
			ACCL_Y_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_READ;
				end else begin
					state <= ACCL_Y_MSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_Y_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_Y[15:8] <= data_rd;
					state <= ACCL_Y_MSB_STOP;
				end else begin
					state <=ACCL_Y_MSB_WAIT_READ;
				end
			end 			
		 	 			
			ACCL_Y_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_Y_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_Y_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_START;
				end else begin
					state <= ACCL_Y_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			ACCL_Y_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_Y_LSB_WAIT_START;
				end else begin
					state<=ACCL_Y_LSB_START;
				end
			end 			
		 						
			ACCL_Y_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_SEND_A6;
				end else begin
					state <= ACCL_Y_LSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_Y_LSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_LSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Y_LSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_SEND_53;
				end else begin
					state <= ACCL_Y_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Y_LSB_SEND_53:begin
				ena <= 1'b1;
				data_wr <= 8'h53;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_LSB_SEND_53;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_LSB_WAIT_53;
				end
			end 			
		 	 				
			ACCL_Y_LSB_WAIT_53:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_RSTART;
				end else begin
					state <= ACCL_Y_LSB_WAIT_53;
				end
			end 			
		 	 				
			ACCL_Y_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_Y_LSB_RSTART;
				end else begin
					state <= ACCL_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_Y_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_SEND_A7;
				end else begin
					state <= ACCL_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_Y_LSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Y_LSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Y_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Y_LSB_WAIT_A7:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_READ;
				end else begin
					state <= ACCL_Y_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Y_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_READ;
				end else begin
					state <= ACCL_Y_LSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_Y_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_Y[7:0] <= data_rd;
					state <= ACCL_Y_LSB_STOP;
				end else begin
					state <= ACCL_Y_LSB_WAIT_READ;
				end
			end 			
		 	 			
			ACCL_Y_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Y_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_Y_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_Y_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_START;
				end else begin
					state <= ACCL_Y_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			ACCL_Z_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_Z_MSB_WAIT_START;
				end else begin
					state<=ACCL_Z_MSB_START;
				end
			end 			
		 						
			ACCL_Z_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_SEND_A6;
				end else begin
					state <= ACCL_Z_MSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_Z_MSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_MSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Z_MSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_SEND_54;
				end else begin
					state <= ACCL_Z_MSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Z_MSB_SEND_54:begin
				ena <= 1'b1;
				data_wr <= 8'h54;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_MSB_SEND_54;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_MSB_WAIT_54;
				end
			end 			
		 	 				
			ACCL_Z_MSB_WAIT_54:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_RSTART;
				end else begin
					state <= ACCL_Z_MSB_WAIT_54;
				end
			end 			
		 	 				
			ACCL_Z_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_Z_MSB_RSTART;
				end else begin
					state <= ACCL_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_Z_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_SEND_A7;
				end else begin
					state <= ACCL_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_Z_MSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_MSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_MSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Z_MSB_WAIT_A7:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_READ;
				end else begin
					state <= ACCL_Z_MSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Z_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_READ;
				end else begin
					state <= ACCL_Z_MSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_Z_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_Z[15:8] <= data_rd;
					state <= ACCL_Z_MSB_STOP;
				end else begin
					state <= ACCL_Z_MSB_WAIT_READ;
				end
			end 			
		 	 			
			ACCL_Z_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_Z_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_Z_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_START;
				end else begin
					state <= ACCL_Z_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			ACCL_Z_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=ACCL_Z_LSB_WAIT_START;
				end else begin
					state<=ACCL_Z_LSB_START;
				end
			end 			
		 						
			ACCL_Z_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_SEND_A6;
				end else begin
					state <= ACCL_Z_LSB_WAIT_START;
				end
			end 			
		 	 			
			ACCL_Z_LSB_SEND_A6:begin
				ena <= 1'b1;
				data_wr <= 8'hA6;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_LSB_SEND_A6;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Z_LSB_WAIT_A6:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_SEND_55;
				end else begin
					state <= ACCL_Z_LSB_WAIT_A6;
				end
			end 			
		 	 				
			ACCL_Z_LSB_SEND_55:begin
				ena <= 1'b1;
				data_wr <= 8'h55;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_LSB_SEND_55;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_LSB_WAIT_55;
				end
			end 			
		 	 				
			ACCL_Z_LSB_WAIT_55:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_RSTART;
				end else begin
					state <= 	ACCL_Z_LSB_WAIT_55;
				end
			end 			
		 					
			ACCL_Z_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= ACCL_Z_LSB_RSTART;
				end else begin
					state <= ACCL_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			ACCL_Z_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <=ACCL_Z_LSB_SEND_A7;
				end else begin
					state <= ACCL_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			ACCL_Z_LSB_SEND_A7:begin
				ena <= 1'b1;
				data_wr <= 8'hA7;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= ACCL_Z_LSB_SEND_A7;
				end else begin
					start_transfer <= 1'b0;
					state <= ACCL_Z_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Z_LSB_WAIT_A7:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_READ;
				end else begin
					state <= ACCL_Z_LSB_WAIT_A7;
				end
			end 			
		 	 				
			ACCL_Z_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_READ;
				end else begin
					state <= ACCL_Z_LSB_WAIT_READ;
				end
			end 			
		 	 					
			ACCL_Z_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					ACCL_Z[7:0] <= data_rd;
					state <= ACCL_Z_LSB_STOP;
				end else begin
					state <= ACCL_Z_LSB_WAIT_READ;
				end
			end 			
		 	 			
			ACCL_Z_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= ACCL_Z_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= ACCL_Z_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			ACCL_Z_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_START;
				end else begin
					state <= ACCL_Z_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//=======MAGM READ=======================	
			MAGM_X_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_X_MSB_WAIT_START;
				end else begin
					state<=MAGM_X_MSB_START;
				end
			end 			
		 						
			MAGM_X_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_SEND_3C;
				end else begin
					state <= MAGM_X_MSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_X_MSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_MSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_X_MSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_SEND_03;
				end else begin
					state <= MAGM_X_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_X_MSB_SEND_03:begin
				ena <= 1'b1;
				data_wr <= 8'h03;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_MSB_SEND_03;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_MSB_WAIT_03;
				end
			end 			
		 	 				
			MAGM_X_MSB_WAIT_03:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_RSTART;
				end else begin
					state <= MAGM_X_MSB_WAIT_03;
				end
			end 			
		 	 				
			MAGM_X_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_X_MSB_RSTART;
				end else begin
					state <= MAGM_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_X_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_SEND_3D;
				end else begin
					state <= MAGM_X_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_X_MSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_MSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_X_MSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_READ;
				end else begin
					state <= MAGM_X_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_X_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_READ;
				end else begin
					state <= MAGM_X_MSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_X_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_X[15:0] <= data_rd;
					state <= MAGM_X_MSB_STOP;
				end else begin
					state <= MAGM_X_MSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_X_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_X_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			MAGM_X_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_START;
				end else begin
					state <= MAGM_X_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			MAGM_X_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_X_LSB_WAIT_START;
				end else begin
					state<=MAGM_X_LSB_START;
				end
			end 			
		 						
			MAGM_X_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_SEND_3C;
				end else begin
					state <= MAGM_X_LSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_X_LSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_LSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_X_LSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_SEND_04;
				end else begin
					state <= MAGM_X_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_X_LSB_SEND_04:begin
				ena <= 1'b1;
				data_wr <= 8'h04;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_LSB_SEND_04;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_LSB_WAIT_04;
				end
			end 			
		 	 				
			MAGM_X_LSB_WAIT_04:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_RSTART;
				end else begin
					state <= MAGM_X_LSB_WAIT_04;
				end
			end 			
		 	 				
			MAGM_X_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_X_LSB_RSTART;
				end else begin
					state <= MAGM_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_X_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_SEND_3D;
				end else begin
					state <= MAGM_X_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_X_LSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_X_LSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_X_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_X_LSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_READ;
				end else begin
					state <= MAGM_X_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_X_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_READ;
				end else begin
					state <= MAGM_X_LSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_X_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_X[7:0] <= data_rd;
					state <= MAGM_X_LSB_STOP;
				end else begin
					state <= MAGM_X_LSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_X_LSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_X_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_X_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			MAGM_X_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_START;
				end else begin
					state <= MAGM_X_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			MAGM_Z_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_Z_MSB_WAIT_START;
				end else begin
					state<=MAGM_Z_MSB_START;
				end
			end 			
		 						
			MAGM_Z_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_SEND_3C;
				end else begin
					state <= MAGM_Z_MSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_Z_MSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_MSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Z_MSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_SEND_05;
				end else begin
					state <= MAGM_Z_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Z_MSB_SEND_05:begin
				ena <= 1'b1;
				data_wr <= 8'h05;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_MSB_SEND_05;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_MSB_WAIT_05;
				end
			end 			
		 	 				
			MAGM_Z_MSB_WAIT_05:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_RSTART;
				end else begin
					state <= MAGM_Z_MSB_WAIT_05;
				end
			end 			
		 	 				
			MAGM_Z_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_Z_MSB_RSTART;
				end else begin
					state <= MAGM_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_Z_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_SEND_3D;
				end else begin
					state <= MAGM_Z_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_Z_MSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_MSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Z_MSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_READ;
				end else begin
					state <= MAGM_Z_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Z_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_READ;
				end else begin
					state <= MAGM_Z_MSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_Z_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_Z[15:0] <= data_rd;
					state <= MAGM_Z_MSB_STOP;
				end else begin
					state <= MAGM_Z_MSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_Z_MSB_STOP:begin
				start_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_Z_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			MAGM_Z_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_START;
				end else begin
					state <= MAGM_Z_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			MAGM_Z_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_Z_LSB_WAIT_START;
				end else begin
					state<=MAGM_Z_LSB_START;
				end
			end 			
		 						
			MAGM_Z_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_SEND_3C;
				end else begin
					state <= MAGM_Z_LSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_Z_LSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_LSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Z_LSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_SEND_06;
				end else begin
					state <= MAGM_Z_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Z_LSB_SEND_06:begin
				ena <= 1'b1;
				data_wr <= 8'h06;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_LSB_SEND_06;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_LSB_WAIT_06;
				end
			end 			
		 	 				
			MAGM_Z_LSB_WAIT_06:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_RSTART;
				end else begin
					state <= MAGM_Z_LSB_WAIT_06;
				end
			end 			
		 	 				
			MAGM_Z_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_Z_LSB_RSTART;
				end else begin
					state <= MAGM_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_Z_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_SEND_3D;
				end else begin
					state <= MAGM_Z_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_Z_LSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Z_LSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Z_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Z_LSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_READ;
				end else begin
					state <= MAGM_Z_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Z_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_READ;
				end else begin
					state <= MAGM_Z_LSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_Z_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_Z[7:0] <= data_rd;
					state <= MAGM_Z_LSB_STOP;
				end else begin
					state <= MAGM_Z_LSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_Z_LSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Z_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_Z_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			MAGM_Z_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_START;
				end else begin
					state <= MAGM_Z_LSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			MAGM_Y_MSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_Y_MSB_WAIT_START;
				end else begin
					state<=MAGM_Y_MSB_START;
				end
			end 			
		 						
			MAGM_Y_MSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_SEND_3C;
				end else begin
					state <= MAGM_Y_MSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_Y_MSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_MSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Y_MSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_SEND_07;
				end else begin
					state <= MAGM_Y_MSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Y_MSB_SEND_07:begin
				ena <= 1'b1;
				data_wr <= 8'h07;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_MSB_SEND_07;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_MSB_WAIT_07;
				end
			end 			
		 	 				
			MAGM_Y_MSB_WAIT_07:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_RSTART;
				end else begin
					state <= MAGM_Y_MSB_WAIT_07;
				end
			end 			
		 	 				
			MAGM_Y_MSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_Y_MSB_RSTART;
				end else begin
					state <= MAGM_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_Y_MSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_SEND_3D;
				end else begin
					state <= MAGM_Y_MSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_Y_MSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_MSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Y_MSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_READ;
				end else begin
					state <= MAGM_Y_MSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Y_MSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_READ;
				end else begin
					state <= MAGM_Y_MSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_Y_MSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_Y[15:8] <= data_rd;
					state <= MAGM_Y_MSB_STOP;
				end else begin
					state <= MAGM_Y_MSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_Y_MSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_MSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_Y_MSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 	 					
			MAGM_Y_MSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_START;
				end else begin
					state <= MAGM_Y_MSB_WAIT_STOP;
				end
			end 			
		 	 			
//================			
			MAGM_Y_LSB_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				r_start <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				if(busy) begin
					ena <= 1'b0;
					state<=MAGM_Y_LSB_WAIT_START;
				end else begin
					state<=MAGM_Y_LSB_START;
				end
			end 			
		 						
			MAGM_Y_LSB_WAIT_START:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_SEND_3C;
				end else begin
					state <= MAGM_Y_LSB_WAIT_START;
				end
			end 			
		 	 			
			MAGM_Y_LSB_SEND_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h3C;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_LSB_SEND_3C;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Y_LSB_WAIT_3C:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_SEND_08;
				end else begin
					state <= MAGM_Y_LSB_WAIT_3C;
				end
			end 			
		 	 				
			MAGM_Y_LSB_SEND_08:begin
				ena <= 1'b1;
				data_wr <= 8'h08;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_LSB_SEND_08;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_LSB_WAIT_08;
				end
			end 			
		 	 				
			MAGM_Y_LSB_WAIT_08:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_RSTART;
				end else begin
					state <= MAGM_Y_LSB_WAIT_08;
				end
			end 			
		 	 				
			MAGM_Y_LSB_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b1;
				if(!busy && ready) begin
					state <= MAGM_Y_LSB_RSTART;
				end else begin
					state <= MAGM_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 				
			MAGM_Y_LSB_WAIT_RSTART:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_SEND_3D;
				end else begin
					state <= MAGM_Y_LSB_WAIT_RSTART;
				end
			end 			
		 	 			
			MAGM_Y_LSB_SEND_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h3D;
				rw <= 1'b0;
				r_start <= 1'b0;
				stop_transfer <= 1'b0;
				if(!busy && ready) begin
					start_transfer <= 1'b1;
					state <= MAGM_Y_LSB_SEND_3D;
				end else begin
					start_transfer <= 1'b0;
					state <= MAGM_Y_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Y_LSB_WAIT_3D:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_READ;
				end else begin
					state <= MAGM_Y_LSB_WAIT_3D;
				end
			end 			
		 	 				
			MAGM_Y_LSB_READ:begin
				ena <= 1'b1;
				data_wr <= 8'hff;
				rw <= 1'b1;
				start_transfer <= 1'b1;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_READ;
				end else begin
					state <= MAGM_Y_LSB_WAIT_READ;
				end
			end 			
		 	 					
			MAGM_Y_LSB_WAIT_READ:begin
				ena <= 1'b1;
				data_wr <= 8'h00;
				rw <= 1'b1;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					MAGM_Y[7:0] <= data_rd;
					state <= MAGM_Y_LSB_STOP;
				end else begin
					state <= MAGM_Y_LSB_WAIT_READ;
				end
			end 			
		 	 			
			MAGM_Y_LSB_STOP:begin
				start_transfer <= 1'b0;
				stop_transfer <= 1'b1;
				r_start <= 1'b0;
				if(!busy) begin
					state <= MAGM_Y_LSB_STOP;
					stop_transfer <= 1'b1;
				end else begin
					state <= MAGM_Y_LSB_WAIT_STOP;
					stop_transfer <= 1'b0;
				end
			end 			
		 					
			MAGM_Y_LSB_WAIT_STOP:begin
				ena <= 1'b0;
				data_wr <= 8'h00;
				rw <= 1'b0;
				start_transfer <= 1'b0;
				stop_transfer <= 1'b0;
				r_start <= 1'b0;
				if(!busy) begin
					state <= GYRO_T_MSB_START;
				end else begin
					state <= MAGM_Y_LSB_WAIT_STOP;
				end
			end 			
//		
//			
//			wait_start:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state<= send_data;
//				end else begin
//					state <= wait_start;
//				end
//			end
// // sending write gyro address
//			send_data: begin
//				ena <= 1'b1;
//				// slave write address of the gyro 
//				data_wr <= 8'hd0;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send;
//				end
//			end
// 
//			wait_send: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_2;
//				end else begin
//					state <= wait_send;
//				end
//			end
//
//// sending reg address for reg 22
//			send_data_2: begin
//				ena <= 1'b1;
//				// d22 is x16
//				data_wr <= 8'h16;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_2;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_2;
//				end
//			end
//			 
//			wait_send_2: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_3;
//				end else begin
//					state <= wait_send_2;
//				end
//			end
//
//// send data for reg 22 for initialization 
//			send_data_3: begin
//				ena <= 1'b1;
//				// data is 0x18 to setup the recommended freq.
//				data_wr <= 8'h18;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_3;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_3;
//				end
//			end
// 
//			wait_send_3: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop;
//				end else begin
//					state <= wait_send_3;
//				end
//			end
//
//			send_stop: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop;
//				end else begin
//					state <= send_start_2;
//				end
//			end
//			
//			send_start_2:begin
//				ena <= 1'b1;
//				if(busy) begin
//					ena <= 1'b0;
//					state<=wait_start_2;
//				end else begin
//					state<=send_start_2;
//				end
//			end
//			
//			wait_start_2:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state<= send_data_4;
//				end else begin
//					state <= wait_start_2;
//				end
//			end
//			
//			// send write slave address for gyro 
//			send_data_4: begin
//				ena <= 1'b1;
//				data_wr <= 8'hd0;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_4;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_4;
//				end
//			end
//			
//			wait_send_4: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_5;
//				end else begin
//					state <= wait_send_4;
//				end
//			end
//			//READ XH
//			send_data_5: begin
//				ena <= 1'b1;
//				data_wr <= gyro_register;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_5;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_5;
//				end
//			end
//				
//			wait_send_5: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= repeated_start;
//				end else begin
//					state <= wait_send_5;
//				end
//			end
//			
//			repeated_start: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b1;
//				if(!busy && ready) begin
//					state <= repeated_start;
//				end else begin
//					state <= wait_repeated_start;
//				end
//			end
//				
//			wait_repeated_start: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_addr_2;
//				end else begin
//					state <= wait_repeated_start;
//				end
//			end
//			// send gyro read address
//			send_addr_2: begin
//				ena <= 1'b1;
//				data_wr <= 8'hd1;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_addr_2;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_addr_2;
//				end
//			end 
//			
//			wait_send_addr_2: begin
//				ena <= 1'b1;
//				data_wr <= 8'h53;// hope this does not do anything since start and stop trans are 0s
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data;
//				end else begin
//					state <= wait_send_addr_2;
//				end
//			end
//			
//			read_data: begin
//				ena <= 1'b1;
//				data_wr <= 8'hff;
//				rw <= 1'b1;
//				start_transfer <= 1'b1;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data;
//				end else begin
//					state <= wait_read_data;
//				end
//			end 
//				
//			wait_read_data: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b1;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= wait_read_data;
//				end else begin
//					state <= send_stop_2;
//				end
//			end
//			
//			
//			send_stop_2: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_2;
//				end else begin
//					state <= send_start_2;
//					if(gyro_register == 8'h1D) begin
//						xh <= data_rd;
//						gyro_register = gyro_register + 1;
//					end else if(gyro_register == 8'h1E) begin
//						xl <= data_rd;
//						gyro_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h1F) begin
//						yh <= data_rd;
//						gyro_register = gyro_register + 1;
//					end else if(gyro_register == 8'h20) begin
//						yl <= data_rd;
//						gyro_register = gyro_register + 1;
//					end else if(gyro_register == 8'h21) begin
//						zh <= data_rd;
//						gyro_register = gyro_register + 1;
//					end else if(gyro_register == 8'h22) begin
//						zl <= data_rd;
//						gyro_register = gyro_register + 1;
//					end else begin
//						state <= send_start_8; 
//					end
//				end
//			end
//			
//				
//			/******************************************************************************
//			start accelerometer
//			******************************************************************************/
//			send_start_8:begin
//				ena <= 1'b1;
//				if(busy) begin
//				ena <= 1'b0;
//					state<=wait_start_8;
//				end else begin
//					state<=send_start_8;
//				end
//			end
//			
//			wait_start_8:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state <= send_data_16;
//				end else begin
//					state <= wait_start_8;
//				end
//			end
//			
//			// sending write accelerometer address
//			send_data_16: begin
//				ena <= 1'b1;
//				// slave write address of the accelerometer
//				data_wr <= 8'h3A;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_16;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_16;
//				end
//			end
//				
//			wait_send_16: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_17;
//				end else begin
//					state <= wait_send_16;
//				end
//			end
//			
//			// sending reg address for reg 0x2D (Power CTL)
//			send_data_17: begin
//				ena <= 1'b1;
//				
//				data_wr <= 8'h2d;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_17;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_17;
//				end
//			end
//				
//			wait_send_17: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_18;
//				end else begin
//					state <= wait_send_17;
//				end
//			end
//			
//			// send data for reg 2d for initialization 
//			send_data_18: begin
//				ena <= 1'b1;
//				// data is 0x08 to setup measure bit mode.
//				data_wr <= 8'h08;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_18;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_18;
//				end
//			end
//			
//			wait_send_18: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_8;
//				end else begin
//					state <= wait_send_18;
//				end
//			end
//			
//			send_stop_8: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_8;
//				end else begin
//					state <= send_start_9;
//				end
//			end
//			
//			send_start_9:begin
//				ena <= 1'b1;
//				if(busy) begin
//					ena <= 1'b0;
//					state<=wait_start_9;
//				end else begin
//					state<=send_start_9;
//				end
//			end
//			
//			wait_start_9:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state <= send_data_19;
//				end else begin
//					state <= wait_start_9;
//				end
//			end
//			
//			// sending write accelerometer address
//			send_data_19: begin
//				ena <= 1'b1;
//				// slave write address of the accelerometer
//				data_wr <= 8'h3A;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_19;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_19;
//				end
//			end
//				
//			wait_send_19: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_19;
//				end else begin
//					state <= wait_send_19;
//				end
//			end
//			
//			// sending reg address for reg 0x31 (Data format)
//			send_data_19: begin
//				ena <= 1'b1;
//				
//				data_wr <= 8'h31;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_19;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_19;
//				end
//			end
//				
//			wait_send_19: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_20;
//				end else begin
//					state <= wait_send_19;
//				end
//			end
//			
//			// send data for reg 2d for initialization 
//			send_data_20: begin
//				ena <= 1'b1;
//				// data is 0x01 to enagle +/- 4 g range
//				data_wr <= 8'h01;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_20;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_20;
//				end
//			end
//			
//			wait_send_20: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_9;
//				end else begin
//					state <= wait_send_20;
//				end
//			end
//			
//			send_stop_9: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_9;
//				end else begin
//					state <= send_start_10;
//				end
//			end
//			
//			send_start_10:begin
//				ena <= 1'b1;
//				if(busy) begin
//					ena <= 1'b0;
//					state<=wait_start_10;
//				end else begin
//					state<=send_start_10;
//				end
//			end
//			
//			wait_start_10:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state<= send_data_21;
//				end else begin
//					state <= wait_start_10;
//				end
//			end
//			
//			// send write slave address for accel 
//			send_data_21: begin
//				ena <= 1'b1;
//				data_wr <= 8'h3a;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_21;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_21;
//				end
//			end
//				
//			wait_send_21: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_22;
//				end else begin
//					state <= wait_send_21;
//				end
//			end
//			
//			// send reg address to read from x0,x1,y0,y1,z0,z1 from accel
//			send_data_22: begin
//				ena <= 1'b1;
//				
//				data_wr <= accelerometer_register;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_22;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_22;
//				end
//			end
//				
//			wait_send_22: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= repeated_start_7;
//				end else begin
//					state <= wait_send_22;
//				end
//			end
//			
//			repeated_start_7: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b1;
//				if(!busy && ready) begin
//					state <= repeated_start_7;
//				end else begin
//					state <= wait_repeated_start_7;
//				end
//			end
//				
//			wait_repeated_start_7: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_addr_8;
//				end else begin
//					state <= wait_repeated_start_7;
//				end
//			end
//			
//			//accelerometer read adress
//			send_addr_8: begin
//				ena <= 1'b1;
//				data_wr <= 8'h3b;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_addr_8;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_addr_8;
//				end
//			end 
//			
//			wait_send_addr_8: begin
//				ena <= 1'b1;
//				data_wr <= 8'h53;// hope this does not do anything since start and stop trans are 0s
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data_6;
//				end else begin
//					state <= wait_send_addr_8;
//				end
//			end
//			
//			read_data_6: begin
//				ena <= 1'b1;
//				data_wr <= 8'hff;
//				rw <= 1'b1;
//				start_transfer <= 1'b1;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data_6;
//				end else begin
//					state <= wait_read_data_6;
//				end
//			end 
//				
//			wait_read_data_6: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b1;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= wait_read_data_6;
//				end else begin
//					state <= send_stop_10;
//				end
//			end
//			
//			send_stop_10: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_10;
//				end else begin
//					state <= send_start_10;
//					
//					if(accelerometer_register == 8'h32) begin
//						x0 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h33) begin
//						x1 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h34) begin
//						y0 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h35) begin
//						y1 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h36) begin
//						z0 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else if(accelerometer_register == 8'h37) begin
//						z1 <= data_rd;
//						accelerometer_register = accelerometer_register + 1;
//					end else begin
//						state <= send_start_11; 
//					end
//				end
//			end
//			/******************************************************************************
//			start Magnetometer
//			******************************************************************************/
//			send_start_11:begin
//				ena <= 1'b1;
//				if(busy) begin
//					ena <= 1'b0;
//					state<=wait_start_11;
//				end else begin
//					state<=send_start_11;
//				end
//			end
//			
//			wait_start_11:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state <= send_data_23;
//				end else begin
//					state <= wait_start_11;
//				end
//			end
//			
//			// sending write magnetometer address
//			send_data_23: begin
//				ena <= 1'b1;
//				// slave write address of the accelerometer
//				data_wr <= 8'h3c;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_23;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_23;
//				end
//			end
//				
//			wait_send_23: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_24;
//				end else begin
//					state <= wait_send_23;
//				end
//			end
//			
//			// sending reg address for mode reg 
//			send_data_24: begin
//				ena <= 1'b1;
//				
//				data_wr <= 8'h02;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_24;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_24;
//				end
//			end
//				
//			wait_send_24: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_25;
//				end else begin
//					state <= wait_send_24;
//				end
//			end
//			
//			// send data for continuous measurement mode
//			send_data_25: begin
//				ena <= 1'b1;
//				
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_25;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_25;
//				end
//			end
//			
//			wait_send_25: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_8;
//				end else begin
//					state <= wait_send_25;
//				end
//			end
//			
//			send_stop_11: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_11;
//				end else begin
//					state <= send_start_12;
//				end
//			end 
//				send_start_12:begin
//				ena <= 1'b1;
//				if(busy) begin
//					ena <= 1'b0;
//					state<=wait_start_12;
//				end else begin
//					state<=send_start_12;
//				end
//			end
//			
//			wait_start_12:begin
//				ena <= 1'b1;
//				r_start<=1'b0;
//				if(ready && !busy) begin
//					state<= send_data_26;
//				end else begin
//					state <= wait_start_12;
//				end
//			end
//			
//			// send write slave address for accel 
//			send_data_26: begin
//				ena <= 1'b1;
//				data_wr <= 8'h3a;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_26;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_26;
//				end
//			end
//				
//			wait_send_26: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_data_27;
//				end else begin
//					state <= wait_send_26;
//				end
//			end
//			
//			// send reg address to read from x0,x1,y0,y1,z0,z1 from accel
//			send_data_27: begin
//				ena <= 1'b1;
//				
//				data_wr <= magnetometer_register;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_data_27;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_27;
//				end
//			end
//				
//			wait_send_27: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= repeated_start_8;
//				end else begin
//					state <= wait_send_27;
//				end
//			end
//			
//			repeated_start_8: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b1;
//				if(!busy && ready) begin
//					state <= repeated_start_8;
//				end else begin
//					state <= wait_repeated_start_8;
//				end
//			end
//				
//			wait_repeated_start_8: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_addr_9;
//				end else begin
//					state <= wait_repeated_start_8;
//				end
//			end
//			// send gyro read address
//			send_addr_9: begin
//				ena <= 1'b1;
//				data_wr <= 8'h3b;
//				rw <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy && ready) begin
//					start_transfer <= 1'b1;
//					stop_transfer <= 1'b0;
//					state <= send_addr_9;
//				end else begin
//					start_transfer <= 1'b0;
//					stop_transfer <= 1'b0;
//					state <= wait_send_addr_9;
//				end
//			end 
//			
//			wait_send_addr_9: begin
//				ena <= 1'b1;
//				data_wr <= 8'h53;// hope this does not do anything since start and stop trans are 0s
//				rw <= 1'b0;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data_7;
//				end else begin
//					state <= wait_send_addr_9;
//				end
//			end
//			
//			read_data_7: begin
//				ena <= 1'b1;
//				data_wr <= 8'hff;
//				rw <= 1'b1;
//				start_transfer <= 1'b1;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= read_data_7;
//				end else begin
//					state <= wait_read_data_7;
//				end
//			end 
//				
//			wait_read_data_7: begin
//				ena <= 1'b1;
//				data_wr <= 8'h00;
//				rw <= 1'b1;
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b0;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= wait_read_data_7;
//				end else begin
//					state <= send_stop_12;
//				end
//			end
//			
//			send_stop_12: begin
//				start_transfer <= 1'b0;
//				stop_transfer <= 1'b1;
//				r_start <= 1'b0;
//				if(!busy) begin
//					state <= send_stop_12;
//				end else begin
//					state <= send_start_12;
//						
//					if(magnetometer_register == 8'h3) begin
//						mx0 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else if(magnetometer_register == 8'h4) begin
//						mx1 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else if(magnetometer_register == 8'h5) begin
//						my0 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else if(magnetometer_register == 8'h6) begin
//						my1 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else if(magnetometer_register == 8'h7) begin
//						mz0 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else if(magnetometer_register == 8'h8) begin
//						mz1 <= data_rd;
//						magnetometer_register = magnetometer_register + 1;
//					end else begin
//						state <= send_start_11; 
//					end
//				end
//			end				
		endcase
	 end
 end
 
 endmodule