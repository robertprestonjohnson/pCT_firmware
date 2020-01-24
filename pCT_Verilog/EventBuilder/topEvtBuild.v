//////////////////////////////////////////////////////////////////////////////////
// Company: UCSC
// Engineer: R. Johnson
// Modified August 20, 2013 to register the Ethernet busy signal and reduce number of clocks
// November 12, 2014 to interface with TimePix
// 
// Create Date:    July 9, 2013 
// Design Name:    FPGA_EvtBuild
// Module Name:    pCT_EvtBuild.v 
// Project Name:   proton computed tomography (pCT)
// Target Devices: ML605
//
// Description: Top level Verilog code for the pCT event builder.
//
//
//////////////////////////////////////////////////////////////////////////////////
module topEvtBuild(SYSCLK_P, SYSCLK_N,
			  RESET_SW, RESET_LED,
			  TrgInhibit,
			  UART_RX, UART_TX,
			  TACKex,
			  GPIO_LED,
			  FCS_B,
			  DVI1_DATA_P, DVI1_DATA_N, DVI1_CMD_P, DVI1_CMD_N, DVI1_RST_P, DVI1_RST_N,
			  DVI1_TRIGGER_P, DVI1_TRIGGER_N, DVI1_FASTOR_P, DVI1_FASTOR_N, 
			  DVI1_U1_P, DVI1_U1_N, DVI1_I2C_CLK, DVI1_I2C_DATA, DVI1_CLK_P, DVI1_CLK_N,
			  DVI2_DATA1_P, DVI2_DATA1_N, DVI2_CMD_P, DVI2_CMD_N, DVI2_RST_P, DVI2_RST_N,
			  DVI2_TRIGGER_P, DVI2_TRIGGER_N, DVI2_FASTOR_P, DVI2_FASTOR_N, 
			  DVI2_DATA2_P, DVI2_DATA2_N, DVI2_I2C_CLK, DVI2_I2C_DATA, DVI2_CLK_P, DVI2_CLK_N,
			  DVI3_DATA_P, DVI3_DATA_N, DVI3_CMD_P, DVI3_CMD_N, DVI3_RST_P, DVI3_RST_N,
			  DVI3_TRIGGER_P, DVI3_TRIGGER_N, DVI3_FASTOR_P, DVI3_FASTOR_N, 
			  DVI3_U1_P, DVI3_U1_N, DVI3_I2C_CLK, DVI3_I2C_DATA, DVI3_CLK_P, DVI3_CLK_N,
			  DVI4_DATA1_P, DVI4_DATA1_N, DVI4_CMD_P, DVI4_CMD_N, DVI4_RST_P, DVI4_RST_N,
			  DVI4_TRIGGER_P, DVI4_TRIGGER_N, DVI4_FASTOR_P, DVI4_FASTOR_N, 
			  DVI4_DATA2_P, DVI4_DATA2_N, DVI4_I2C_CLK, DVI4_I2C_DATA, DVI4_CLK_P, DVI4_CLK_N,
			  DVI5_DATA1_P, DVI5_DATA1_N, DVI5_CMD_P, DVI5_CMD_N, DVI5_RST_P, DVI5_RST_N,
			  DVI5_TRIGGER_P, DVI5_TRIGGER_N, DVI5_FASTOR_P, DVI5_FASTOR_N, 
			  DVI5_DATA2_P, DVI5_DATA2_N, DVI5_I2C_CLK, DVI5_I2C_DATA, DVI5_CLK_P, DVI5_CLK_N,
			  DVI6_DATA_P, DVI6_DATA_N, DVI6_CMD_P, DVI6_CMD_N, DVI6_RST_P, DVI6_RST_N,
			  DVI6_TRIGGER_P, DVI6_TRIGGER_N, DVI6_FASTOR_P, DVI6_FASTOR_N, 
			  DVI6_U1_P, DVI6_U1_N, DVI6_I2C_CLK, DVI6_I2C_DATA, DVI6_CLK_P, DVI6_CLK_N,
			  DVI7_DATA1_P, DVI7_DATA1_N, DVI7_CMD_P, DVI7_CMD_N, DVI7_RST_P, DVI7_RST_N,
			  DVI7_TRIGGER_P, DVI7_TRIGGER_N, DVI7_FASTOR_P, DVI7_FASTOR_N, 
			  DVI7_DATA2_P, DVI7_DATA2_N, DVI7_I2C_CLK, DVI7_I2C_DATA, DVI7_CLK_P, DVI7_CLK_N,
			  DVI8_DATA_P, DVI8_DATA_N, DVI8_CMD_P, DVI8_CMD_N, DVI8_RST_P, DVI8_RST_N,
			  DVI8_TRIGGER_P, DVI8_TRIGGER_N, DVI8_FASTOR_P, DVI8_FASTOR_N, 
			  DVI8_U1_P, DVI8_U1_N, DVI8_I2C_CLK, DVI8_I2C_DATA, DVI8_CLK_P, DVI8_CLK_N,
			  DVI9_DATA_P, DVI9_DATA_N, DVI9_CMD_P, DVI9_CMD_N, DVI9_CLKDIGI_P, DVI9_CLKDIGI_N,
			  DVI9_TRIGGER_P, DVI9_TRIGGER_N, DVI9_FASTOR_P, DVI9_FASTOR_N, 
			  DVI9_DATA2_P, DVI9_DATA2_N, DVI9_I2C_CLK, DVI9_I2C_DATA, DVI9_CLK_P, DVI9_CLK_N,
			  DVI10_DATA_P, DVI10_DATA_N, DVI10_CMD_P, DVI10_CMD_N, DVI10_CLKDIGI_P, DVI10_CLKDIGI_N,
			  DVI10_TRIGGER_P, DVI10_TRIGGER_N, DVI10_FASTOR_P, DVI10_FASTOR_N, 
			  DVI10_DATA2_P, DVI10_DATA2_N, DVI10_I2C_CLK, DVI10_I2C_DATA, DVI10_CLK_P, DVI10_CLK_N,
			  ACCEL_CLK_P, ACCEL_CLK_N, CLK_SDA, CLK_SCL, SDA, SCL,
			  ETH_0_P, ETH_0_N, ETH_1_P, ETH_1_N, ETH_2_P, ETH_2_N, ETH_3_P, ETH_3_N,
			  ETH_4_P, ETH_4_N, ETH_5_P, ETH_5_N, ETH_6_P, ETH_6_N, ETH_7_P, ETH_7_N,
			  ETH_8_P, ETH_8_N, ETH_9_P, ETH_9_N, ETH_10_P, ETH_10_N, ETH_11_P, ETH_11_N,
			  ETH_12_P, ETH_12_N, ETH_13_P, ETH_13_N, ETH_14_P, ETH_14_N, ETH_15_P, ETH_15_N,
			  ETH_16_P, ETH_16_N, ETH_17_P, ETH_17_N, ETH_18_P, ETH_18_N, ETH_19_P, ETH_19_N);

input SYSCLK_P, SYSCLK_N;
input RESET_SW;
output RESET_LED;
output UART_TX;
input UART_RX;
input TrgInhibit;
output TACKex;
output [7:0] GPIO_LED;
output FCS_B;
input  DVI1_DATA_P, DVI1_DATA_N, DVI1_FASTOR_P, DVI1_FASTOR_N, DVI1_U1_P, DVI1_U1_N;
output DVI1_CMD_P, DVI1_CMD_N, DVI1_RST_P, DVI1_RST_N, DVI1_TRIGGER_P, DVI1_TRIGGER_N; 
inout DVI1_I2C_CLK, DVI1_I2C_DATA;
output DVI1_CLK_P, DVI1_CLK_N;
input  DVI2_DATA1_P, DVI2_DATA1_N, DVI2_FASTOR_P, DVI2_FASTOR_N, DVI2_DATA2_P, DVI2_DATA2_N;
output DVI2_CMD_P, DVI2_CMD_N, DVI2_RST_P, DVI2_RST_N, DVI2_TRIGGER_P, DVI2_TRIGGER_N; 
inout DVI2_I2C_CLK, DVI2_I2C_DATA;
output DVI2_CLK_P, DVI2_CLK_N;
input  DVI3_DATA_P, DVI3_DATA_N, DVI3_FASTOR_P, DVI3_FASTOR_N, DVI3_U1_P, DVI3_U1_N;
output DVI3_CMD_P, DVI3_CMD_N, DVI3_RST_P, DVI3_RST_N, DVI3_TRIGGER_P, DVI3_TRIGGER_N; 
inout DVI3_I2C_CLK, DVI3_I2C_DATA;
output DVI3_CLK_P, DVI3_CLK_N;
input  DVI4_DATA1_P, DVI4_DATA1_N, DVI4_FASTOR_P, DVI4_FASTOR_N, DVI4_DATA2_P, DVI4_DATA2_N;
output DVI4_CMD_P, DVI4_CMD_N, DVI4_RST_P, DVI4_RST_N, DVI4_TRIGGER_P, DVI4_TRIGGER_N; 
inout DVI4_I2C_CLK, DVI4_I2C_DATA;
output DVI4_CLK_P, DVI4_CLK_N;
input  DVI5_DATA1_P, DVI5_DATA1_N, DVI5_FASTOR_P, DVI5_FASTOR_N, DVI5_DATA2_P, DVI5_DATA2_N;
output DVI5_CMD_P, DVI5_CMD_N, DVI5_RST_P, DVI5_RST_N, DVI5_TRIGGER_P, DVI5_TRIGGER_N; 
inout DVI5_I2C_CLK, DVI5_I2C_DATA;
output DVI5_CLK_P, DVI5_CLK_N;
input  DVI6_DATA_P, DVI6_DATA_N, DVI6_FASTOR_P, DVI6_FASTOR_N, DVI6_U1_P, DVI6_U1_N;
output DVI6_CMD_P, DVI6_CMD_N, DVI6_RST_P, DVI6_RST_N, DVI6_TRIGGER_P, DVI6_TRIGGER_N; 
inout DVI6_I2C_CLK, DVI6_I2C_DATA;
output DVI6_CLK_P, DVI6_CLK_N;
input  DVI7_DATA1_P, DVI7_DATA1_N, DVI7_FASTOR_P, DVI7_FASTOR_N, DVI7_DATA2_P, DVI7_DATA2_N;
output DVI7_CMD_P, DVI7_CMD_N, DVI7_RST_P, DVI7_RST_N, DVI7_TRIGGER_P, DVI7_TRIGGER_N; 
inout DVI7_I2C_CLK, DVI7_I2C_DATA;
output DVI7_CLK_P, DVI7_CLK_N;
input  DVI8_DATA_P, DVI8_DATA_N, DVI8_FASTOR_P, DVI8_FASTOR_N, DVI8_U1_P, DVI8_U1_N;
output DVI8_CMD_P, DVI8_CMD_N, DVI8_RST_P, DVI8_RST_N, DVI8_TRIGGER_P, DVI8_TRIGGER_N; 
inout DVI8_I2C_CLK, DVI8_I2C_DATA;
output DVI8_CLK_P, DVI8_CLK_N;
input  DVI9_DATA_P, DVI9_DATA_N, DVI9_FASTOR_P, DVI9_FASTOR_N, DVI9_DATA2_P, DVI9_DATA2_N;
output DVI9_CLKDIGI_P, DVI9_CLKDIGI_N, DVI9_CLK_P, DVI9_CLK_N;
output DVI9_CMD_P, DVI9_CMD_N, DVI9_TRIGGER_P, DVI9_TRIGGER_N; 
inout DVI9_I2C_CLK, DVI9_I2C_DATA;
input  DVI10_DATA_P, DVI10_DATA_N, DVI10_FASTOR_P, DVI10_FASTOR_N, DVI10_DATA2_P, DVI10_DATA2_N;
output DVI10_CLKDIGI_P, DVI10_CLKDIGI_N, DVI10_CLK_P, DVI10_CLK_N;
output DVI10_CMD_P, DVI10_CMD_N, DVI10_TRIGGER_P, DVI10_TRIGGER_N; 
inout DVI10_I2C_CLK, DVI10_I2C_DATA;
input ACCEL_CLK_P, ACCEL_CLK_N;
inout CLK_SDA, CLK_SCL, SDA, SCL;
input ETH_0_P, ETH_0_N;  //Inhibit signal from Zest-ET1 when the input FIFO is full
output  ETH_1_P, ETH_1_N, ETH_2_P, ETH_2_N, ETH_3_P, ETH_3_N;
output  ETH_4_P, ETH_4_N, ETH_5_P, ETH_5_N, ETH_6_P, ETH_6_N, ETH_7_P, ETH_7_N;
output  ETH_8_P, ETH_8_N, ETH_9_P, ETH_9_N, ETH_10_P, ETH_10_N, ETH_11_P, ETH_11_N;
output  ETH_12_P, ETH_12_N, ETH_13_P, ETH_13_N, ETH_14_P, ETH_14_N, ETH_15_P, ETH_15_N;
output  ETH_16_P, ETH_16_N, ETH_17_P, ETH_17_N, ETH_18_P, ETH_18_N, ETH_19_P, ETH_19_N;

//Set this signal high to keep the FLASH memory quiet during operation
assign FCS_B = 1'b1;

//Digital clock manager, derives the local clocks from the 200 MHz input clock
clock managed_clocks 	(.SYSCLK_P(SYSCLK_P), .SYSCLK_N(SYSCLK_N), .CLK(CLK), .CLK_DIGI(CLKDIGint), .DCMRESET(DCMRESET), .DCMLOCKED(DCMLOCKED));
assign CLK_PHASE = CLK;    //Removed two clocks to improve routing, since all were set to the same phase anyway
assign CLK_100 = CLK;

assign TACKex = ~Diagnostic2;   //Send the trigger acknowledge signal out to an SMA connector (TimePix needs active low)

//Send a 100 MHz system clock to the front-end boards
OBUFDS CLK1_BUFFER 		(.O(DVI1_CLK_P), .OB(DVI1_CLK_N), .I(CLK_100));  
OBUFDS CLK2_BUFFER 		(.O(DVI2_CLK_P), .OB(DVI2_CLK_N), .I(CLK_100));  
OBUFDS CLK3_BUFFER 		(.O(DVI3_CLK_P), .OB(DVI3_CLK_N), .I(CLK_100));  
OBUFDS CLK4_BUFFER 		(.O(DVI4_CLK_P), .OB(DVI4_CLK_N), .I(CLK_100));  
OBUFDS CLK5_BUFFER 		(.O(DVI5_CLK_P), .OB(DVI5_CLK_N), .I(CLK_100));  
OBUFDS CLK6_BUFFER 		(.O(DVI6_CLK_P), .OB(DVI6_CLK_N), .I(CLK_100));  
OBUFDS CLK7_BUFFER 		(.O(DVI7_CLK_P), .OB(DVI7_CLK_N), .I(CLK_100));  
OBUFDS CLK8_BUFFER 		(.O(DVI8_CLK_P), .OB(DVI8_CLK_N), .I(CLK_100));  
OBUFDS CLK9_BUFFER 		(.O(DVI9_CLK_P), .OB(DVI9_CLK_N), .I(CLK_100));  
OBUFDS CLK10_BUFFER 	(.O(DVI10_CLK_P), .OB(DVI10_CLK_N), .I(CLK_100));  

//Receive the accelerator RF clock
IBUFGDS CLKDigBuf (.I(ACCEL_CLK_P), .IB(ACCEL_CLK_N), .O(CLKDIGbuf));

//Send the digitizer clock or reset signal to the two energy-digitizer boards
reg ClkOrRst;
always @ (ClkVsRst or CLKDIGbuf or RSTfpga) begin
	if (ClkVsRst) begin
		ClkOrRst = CLKDIGbuf;
	end else begin
		ClkOrRst = RSTfpga;
	end
end
OBUFDS CLKdigCbuf (.O(DVI9_CLKDIGI_P), .OB(DVI9_CLKDIGI_N), .I(ClkOrRst));     
OBUFDS CLKdigDbuf (.O(DVI10_CLKDIGI_P), .OB(DVI10_CLKDIGI_N), .I(ClkOrRst));

//Send commands to the front-end boards
ODDR CMD1_REG (.Q(CMDreg1), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD1_BUFFER 		(.O(DVI1_CMD_P), .OB(DVI1_CMD_N), .I(CMDreg1));  
ODDR CMD2_REG (.Q(CMDreg2), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD2_BUFFER 		(.O(DVI2_CMD_P), .OB(DVI2_CMD_N), .I(CMDreg2));  
ODDR CMD3_REG (.Q(CMDreg3), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD3_BUFFER 		(.O(DVI3_CMD_P), .OB(DVI3_CMD_N), .I(CMDreg3));  
ODDR CMD4_REG (.Q(CMDreg4), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD4_BUFFER 		(.O(DVI4_CMD_P), .OB(DVI4_CMD_N), .I(CMDreg4));  
ODDR CMD5_REG (.Q(CMDreg5), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD5_BUFFER 		(.O(DVI5_CMD_P), .OB(DVI5_CMD_N), .I(CMDreg5));  
ODDR CMD6_REG (.Q(CMDreg6), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD6_BUFFER 		(.O(DVI6_CMD_P), .OB(DVI6_CMD_N), .I(CMDreg6));  
ODDR CMD7_REG (.Q(CMDreg7), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD7_BUFFER 		(.O(DVI7_CMD_P), .OB(DVI7_CMD_N), .I(CMDreg7));  
ODDR CMD8_REG (.Q(CMDreg8), .D1(CMD), .D2(CMD), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD8_BUFFER 		(.O(DVI8_CMD_P), .OB(DVI8_CMD_N), .I(CMDreg8));  
ODDR CMD9_REG (.Q(CMDreg9), .D1(CMDE), .D2(CMDE), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD9_BUFFER 		(.O(DVI9_CMD_P), .OB(DVI9_CMD_N), .I(CMDreg9));
ODDR CMD10_REG (.Q(CMDreg10), .D1(CMDE), .D2(CMDE), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS CMD10_BUFFER 		(.O(DVI10_CMD_P), .OB(DVI10_CMD_N), .I(CMDreg10));

//Send hard resets to the tracker front-end boards
ODDR RST1_REG (.Q(RSTreg1), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST1_BUFFER 		(.O(DVI1_RST_P), .OB(DVI1_RST_N), .I(RSTreg1));  
ODDR RST2_REG (.Q(RSTreg2), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST2_BUFFER 		(.O(DVI2_RST_P), .OB(DVI2_RST_N), .I(RSTreg2));  
ODDR RST3_REG (.Q(RSTreg3), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST3_BUFFER 		(.O(DVI3_RST_P), .OB(DVI3_RST_N), .I(RSTreg3));  
ODDR RST4_REG (.Q(RSTreg4), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST4_BUFFER 		(.O(DVI4_RST_P), .OB(DVI4_RST_N), .I(RSTreg4));  
ODDR RST5_REG (.Q(RSTreg5), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST5_BUFFER 		(.O(DVI5_RST_P), .OB(DVI5_RST_N), .I(RSTreg5));  
ODDR RST6_REG (.Q(RSTreg6), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST6_BUFFER 		(.O(DVI6_RST_P), .OB(DVI6_RST_N), .I(RSTreg6));  
ODDR RST7_REG (.Q(RSTreg7), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST7_BUFFER 		(.O(DVI7_RST_P), .OB(DVI7_RST_N), .I(RSTreg7));  
ODDR RST8_REG (.Q(RSTreg8), .D1(RSTfpga), .D2(RSTfpga), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS RST8_BUFFER 		(.O(DVI8_RST_P), .OB(DVI8_RST_N), .I(RSTreg8));  

//Send trigger acknowledge signals to the front-end boards
ODDR TRIGGER1_REG (.Q(TRIGGERreg1), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER1_BUFFER 		(.O(DVI1_TRIGGER_P), .OB(DVI1_TRIGGER_N), .I(TRIGGERreg1));  
ODDR TRIGGER2_REG (.Q(TRIGGERreg2), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER2_BUFFER 		(.O(DVI2_TRIGGER_P), .OB(DVI2_TRIGGER_N), .I(TRIGGERreg2));  
ODDR TRIGGER3_REG (.Q(TRIGGERreg3), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER3_BUFFER 		(.O(DVI3_TRIGGER_P), .OB(DVI3_TRIGGER_N), .I(TRIGGERreg3));  
ODDR TRIGGER4_REG (.Q(TRIGGERreg4), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER4_BUFFER 		(.O(DVI4_TRIGGER_P), .OB(DVI4_TRIGGER_N), .I(TRIGGERreg4));  
ODDR TRIGGER5_REG (.Q(TRIGGERreg5), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER5_BUFFER 		(.O(DVI5_TRIGGER_P), .OB(DVI5_TRIGGER_N), .I(TRIGGERreg5));  
ODDR TRIGGER6_REG (.Q(TRIGGERreg6), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER6_BUFFER 		(.O(DVI6_TRIGGER_P), .OB(DVI6_TRIGGER_N), .I(TRIGGERreg6));  
ODDR TRIGGER7_REG (.Q(TRIGGERreg7), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER7_BUFFER 		(.O(DVI7_TRIGGER_P), .OB(DVI7_TRIGGER_N), .I(TRIGGERreg7));  
ODDR TRIGGER8_REG (.Q(TRIGGERreg8), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER8_BUFFER 		(.O(DVI8_TRIGGER_P), .OB(DVI8_TRIGGER_N), .I(TRIGGERreg8));  
ODDR TRIGGER9_REG (.Q(TRIGGERreg9), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER9_BUFFER 		(.O(DVI9_TRIGGER_P), .OB(DVI9_TRIGGER_N), .I(TRIGGERreg9));
ODDR TRIGGER10_REG (.Q(TRIGGERreg10), .D1(TRIGGER), .D2(TRIGGER), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS TRIGGER10_BUFFER 		(.O(DVI10_TRIGGER_P), .OB(DVI10_TRIGGER_N), .I(TRIGGERreg10));

//Receive all the Fast-OR signals from the front-end boards
//Note:  depending on the buffer management mode, 4 of these signals may be used instead for buffer clear.
//       They are the ones for which we implement a controlled timing delay.
//The following is the ordering of the boards and their addresses:
//                                         FPGA       [FastOR]   DVI Cable
//  First tracker layer (V): 					h0			[h0]		1
//  Second tracker layer (T): 			h4 & h5		[h1]		2
//  Third tracker layer (V):					h1			[h2]		3
//	Fourth tracker layer (T):				h6 & h7		[h3]		4
//	Fifth tracker layer (T):				h8 & h9		[h4]		5
//	Sixth tracker layer (V):					h2			[h5]		6
//  Seventh tracker layer (T):			hA & hB		[h6]		7
//	Eighth tracker layer (V):					h3			[h7]		8
//  First energy digitizer (3 channels):	hC			[h8]		9
//	Second energy digitizer (2 channels):	hD			[h9]		10
wire [9:0] FastORreg;
IBUFDS FastOR1_BUFFER	(.I(DVI1_FASTOR_P), .IB(DVI1_FASTOR_N), .O(FastOR1));   //Bank 22
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	FastOR1_DELAY (.DATAOUT(FastORDly1), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(FastOR1), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR FastOR1_REG (.D(FastORDly1), .Q1(FastORreg[0]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR2_BUFFER	(.I(DVI2_FASTOR_P), .IB(DVI2_FASTOR_N), .O(FastOR2));  
IDDR FastOR2_REG (.D(FastOR2), .Q1(FastORreg[1]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR3_BUFFER	(.I(DVI3_FASTOR_P), .IB(DVI3_FASTOR_N), .O(FastOR3));   //Bank 13
(* IODELAY_GROUP = "Dly13" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	FastOR3_DELAY (.DATAOUT(FastORDly3), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(FastOR3), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR FastOR3_REG (.D(FastORDly3), .Q1(FastORreg[2]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR4_BUFFER	(.I(DVI4_FASTOR_P), .IB(DVI4_FASTOR_N), .O(FastOR4));  
IDDR FastOR4_REG (.D(FastOR4), .Q1(FastORreg[3]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR5_BUFFER	(.I(DVI5_FASTOR_P), .IB(DVI5_FASTOR_N), .O(FastOR5));     //Bank 14
(* IODELAY_GROUP = "Dly14" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	FastOR5_DELAY (.DATAOUT(FastORDly5), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(FastOR5), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc)); 
IDDR FastOR5_REG (.D(FastORDly5), .Q1(FastORreg[4]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR6_BUFFER	(.I(DVI6_FASTOR_P), .IB(DVI6_FASTOR_N), .O(FastOR6));  
(* IODELAY_GROUP = "Dly14" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	FastOR6_DELAY (.DATAOUT(FastORDly6), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(FastOR6), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR FastOR6_REG (.D(FastORDly6), .Q1(FastORreg[5]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR7_BUFFER	(.I(DVI7_FASTOR_P), .IB(DVI7_FASTOR_N), .O(FastOR7));  
IDDR FastOR7_REG (.D(FastOR7), .Q1(FastORreg[6]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR8_BUFFER	(.I(DVI8_FASTOR_P), .IB(DVI8_FASTOR_N), .O(FastOR8));  
IDDR FastOR8_REG (.D(FastOR8), .Q1(FastORreg[7]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR9_BUFFER	(.I(DVI9_FASTOR_P), .IB(DVI9_FASTOR_N), .O(FastOR9));  
IDDR FastOR9_REG (.D(FastOR9), .Q1(FastORreg[8]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS FastOR10_BUFFER	(.I(DVI10_FASTOR_P), .IB(DVI10_FASTOR_N), .O(FastOR10));  
IDDR FastOR10_REG (.D(FastOR10), .Q1(FastORreg[9]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));

//Receive all the data signals from the front-end boards
wire [15:0] DataReg;
parameter [4:0] DlyVal= 5'd3;   //Default input delay setting upon reset
IBUFDS Data0_BUFFER	(.I(DVI1_DATA_P), .IB(DVI1_DATA_N), .O(Data0));   //V board
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA0_DELAY (.DATAOUT(DataDly0), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data0), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data0_REG (.D(DataDly0), .Q1(DataReg[0]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data4_BUFFER	(.I(DVI2_DATA1_P), .IB(DVI2_DATA1_N), .O(Data4)); //T board, 1st FPGA
(* IODELAY_GROUP = "Dly13" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA4_DELAY (.DATAOUT(DataDly4), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data4), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data4_REG (.D(DataDly4), .Q1(DataReg[4]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data5_BUFFER	(.I(DVI2_DATA2_P), .IB(DVI2_DATA2_N), .O(Data5)); //T board, 2nd FPGA
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA5_DELAY (.DATAOUT(DataDly5), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data5), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data5_REG (.D(DataDly5), .Q1(DataReg[5]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data1_BUFFER	(.I(DVI3_DATA_P), .IB(DVI3_DATA_N), .O(Data1));   //V board
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA1_DELAY (.DATAOUT(DataDly1), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data1), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data1_REG (.D(DataDly1), .Q1(DataReg[1]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data6_BUFFER	(.I(DVI4_DATA1_P), .IB(DVI4_DATA1_N), .O(Data6)); //T board, 1st FPGA
(* IODELAY_GROUP = "Dly13" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA6_DELAY (.DATAOUT(DataDly6), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data6), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data6_REG (.D(DataDly6), .Q1(DataReg[6]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data7_BUFFER	(.I(DVI4_DATA2_P), .IB(DVI4_DATA2_N), .O(Data7)); //T board, 2nd FPGA
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA7_DELAY (.DATAOUT(DataDly7), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data7), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data7_REG (.D(DataDly7), .Q1(DataReg[7]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data8_BUFFER	(.I(DVI5_DATA1_P), .IB(DVI5_DATA1_N), .O(Data8)); //T board, 1st FPGA
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA8_DELAY (.DATAOUT(DataDly8), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data8), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data8_REG (.D(DataDly8), .Q1(DataReg[8]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data9_BUFFER	(.I(DVI5_DATA2_P), .IB(DVI5_DATA2_N), .O(Data9)); //T board, 2nd FPGA
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA9_DELAY (.DATAOUT(DataDly9), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data9), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data9_REG (.D(DataDly9), .Q1(DataReg[9]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data2_BUFFER	(.I(DVI6_DATA_P), .IB(DVI6_DATA_N), .O(Data2));   //V board
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA2_DELAY (.DATAOUT(DataDly2), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data2), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data2_REG (.D(DataDly2), .Q1(DataReg[2]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS DataA_BUFFER	(.I(DVI7_DATA1_P), .IB(DVI7_DATA1_N), .O(DataA)); //T board, 1st FPGA
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATAa_DELAY (.DATAOUT(DataDlyA), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(DataA), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR DataA_REG (.D(DataDlyA), .Q1(DataReg[10]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS DataB_BUFFER	(.I(DVI7_DATA2_P), .IB(DVI7_DATA2_N), .O(DataB)); //T board, 2nd FPGA
(* IODELAY_GROUP = "Dly12" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATAb_DELAY (.DATAOUT(DataDlyB), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(DataB), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR DataB_REG (.D(DataDlyB), .Q1(DataReg[11]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data3_BUFFER	(.I(DVI8_DATA_P), .IB(DVI8_DATA_N), .O(Data3));   //V board
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA3_DELAY (.DATAOUT(DataDly3), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data3), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data3_REG (.D(DataDly3), .Q1(DataReg[3]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS DataC_BUFFER	(.I(DVI9_DATA_P), .IB(DVI9_DATA_N), .O(DataC));   //Energy digitizer board
(* IODELAY_GROUP = "Dly12" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATAc_DELAY (.DATAOUT(DataDlyC), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(DataC), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR DataC_REG (.D(DataDlyC), .Q1(DataReg[12]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS DataD_BUFFER	(.I(DVI10_DATA_P), .IB(DVI10_DATA_N), .O(DataD));   //Energy digitizer board
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATAd_DELAY (.DATAOUT(DataDlyD), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(DataD), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR DataD_REG (.D(DataDlyD), .Q1(DataReg[14]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data2C_BUFFER	(.I(DVI9_DATA2_P), .IB(DVI9_DATA2_N), .O(Data2C));   //Energy digitizer board second data line
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA2c_DELAY (.DATAOUT(Data2DlyC), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data2C), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data2C_REG (.D(Data2DlyC), .Q1(DataReg[13]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS Data2D_BUFFER	(.I(DVI10_DATA2_P), .IB(DVI10_DATA2_N), .O(Data2D));   //Energy digitizer board second data line
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	DATA2d_DELAY (.DATAOUT(Data2DlyD), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(Data2D), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR Data2D_REG (.D(Data2DlyD), .Q1(DataReg[15]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));

(* IODELAY_GROUP = "Dly12" *) IDELAYCTRL DelayCtrl12(.REFCLK(CLK_PHASE), .RST(ResetDly), .RDY(Rdy12));  //Bank 12
(* IODELAY_GROUP = "Dly13" *) IDELAYCTRL DelayCtrl13(.REFCLK(CLK_PHASE), .RST(ResetDly), .RDY(Rdy13));  //Bank 13
(* IODELAY_GROUP = "Dly14" *) IDELAYCTRL DelayCtrl14(.REFCLK(CLK_PHASE), .RST(ResetDly), .RDY(Rdy14));  //Bank 14
(* IODELAY_GROUP = "Dly22" *) IDELAYCTRL DelayCtrl22(.REFCLK(CLK_PHASE), .RST(ResetDly), .RDY(Rdy22));  //Bank 22
(* IODELAY_GROUP = "Dly23" *) IDELAYCTRL DelayCtrl23(.REFCLK(CLK_PHASE), .RST(ResetDly), .RDY(Rdy23));  //Bank 23

//Receive the buffer clear signals from the V boards
wire [3:0] U1Reg;
IBUFDS U1_1_BUFFER	(.I(DVI1_U1_P), .IB(DVI1_U1_N), .O(U1_1));   //Bank 22
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	U1_1_DELAY (.DATAOUT(U1Dly1), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(U1_1), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR U1_1_REG (.D(U1Dly1), .Q1(U1Reg[0]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS U1_3_BUFFER	(.I(DVI3_U1_P), .IB(DVI3_U1_N), .O(U1_3));    //Bank 22
(* IODELAY_GROUP = "Dly22" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	U1_3_DELAY (.DATAOUT(U1Dly3), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(U1_3), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR U1_3_REG (.D(U1Dly3), .Q1(U1Reg[1]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS U1_6_BUFFER	(.I(DVI6_U1_P), .IB(DVI6_U1_N), .O(U1_6));     //Bank 14
(* IODELAY_GROUP = "Dly14" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	U1_6_DELAY (.DATAOUT(U1Dly6), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(U1_6), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR U1_6_REG (.D(U1Dly6), .Q1(U1Reg[2]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS U1_8_BUFFER	(.I(DVI8_U1_P), .IB(DVI8_U1_N), .O(U1_8));     //Bank 23
(* IODELAY_GROUP = "Dly23" *) IODELAYE1 #(.IDELAY_TYPE("VARIABLE"), .SIGNAL_PATTERN("DATA"), .IDELAY_VALUE(DlyVal))
	U1_8_DELAY (.DATAOUT(U1Dly8), .C(CLK_PHASE), .DATAIN(1'b0), .IDATAIN(U1_8), 
						.T(1'b1), .RST(ResetDly), .INC(IncDir), .CE(EnableInc));
IDDR U1_8_REG (.D(U1Dly8), .Q1(U1Reg[3]), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));

//Connect to all of the i2c lines
wire [9:0] SDAin, SDAout, SDAen, SCLout, SCLen;
//First, the SDA and SCL from the second USB port.  This is not used so far, so just keep the lines disabled
parameter SDAout2 = 1'b1;
parameter SCLout2 = 1'b1;
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA2 (.O(SDAin2), .IO(SDA), .I(SDAout2), .T(1'b1));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL2 (.O(SCLin2), .IO(SCL), .I(SCLout2), .T(1'b1));
//Second, the SDA and SCL lines for controlling the beam RF PLL chip.  So far we only control them from the
//USB interface, so keep these pins high impedance, too
parameter CLK_SDAout = 1'b1;
parameter CLK_SCLout = 1'b1;
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_CLK_SDA (.O(CLK_SDAin), .IO(CLK_SDA), .I(CLK_SDAout), .T(1'b1));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_CLK_SCL (.O(CLK_SCLin), .IO(CLK_SCL), .I(CLK_SCLout), .T(1'b1));
//Third, the SDA and SCL lines going to the 10 front-end boards
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI1 (.O(SDAin[0]), .IO(DVI1_I2C_DATA), .I(SDAout[0]), .T(SDAen[0]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI1 (.O(SCLinDum0), .IO(DVI1_I2C_CLK), .I(SCLout[0]), .T(SCLen[0]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI2 (.O(SDAin[1]), .IO(DVI2_I2C_DATA), .I(SDAout[1]), .T(SDAen[1]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI2 (.O(SCLinDum1), .IO(DVI2_I2C_CLK), .I(SCLout[1]), .T(SCLen[1]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI3 (.O(SDAin[2]), .IO(DVI3_I2C_DATA), .I(SDAout[2]), .T(SDAen[2]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI3 (.O(SCLinDum2), .IO(DVI3_I2C_CLK), .I(SCLout[2]), .T(SCLen[2]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI4 (.O(SDAin[3]), .IO(DVI4_I2C_DATA), .I(SDAout[3]), .T(SDAen[3]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI4 (.O(SCLinDum3), .IO(DVI4_I2C_CLK), .I(SCLout[3]), .T(SCLen[3]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI5 (.O(SDAin[4]), .IO(DVI5_I2C_DATA), .I(SDAout[4]), .T(SDAen[4]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI5 (.O(SCLinDum4), .IO(DVI5_I2C_CLK), .I(SCLout[4]), .T(SCLen[4]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI6 (.O(SDAin[5]), .IO(DVI6_I2C_DATA), .I(SDAout[5]), .T(SDAen[5]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI6 (.O(SCLinDum5), .IO(DVI6_I2C_CLK), .I(SCLout[5]), .T(SCLen[5]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI7 (.O(SDAin[6]), .IO(DVI7_I2C_DATA), .I(SDAout[6]), .T(SDAen[6]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI7 (.O(SCLinDum6), .IO(DVI7_I2C_CLK), .I(SCLout[6]), .T(SCLen[6]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI8 (.O(SDAin[7]), .IO(DVI8_I2C_DATA), .I(SDAout[7]), .T(SDAen[7]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI8 (.O(SCLinDum7), .IO(DVI8_I2C_CLK), .I(SCLout[7]), .T(SCLen[7]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI9 (.O(SDAin[8]), .IO(DVI9_I2C_DATA), .I(SDAout[8]), .T(SDAen[8]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI9 (.O(SCLinDum8), .IO(DVI9_I2C_CLK), .I(SCLout[8]), .T(SCLen[8]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA_DVI10 (.O(SDAin[9]), .IO(DVI10_I2C_DATA), .I(SDAout[9]), .T(SDAen[9]));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL_DVI10 (.O(SCLinDum9), .IO(DVI10_I2C_CLK), .I(SCLout[9]), .T(SCLen[9]));

//Ethernet interface (Zest-ET1) 
wire [15:0] EthDat;
//Incoming busy signal
IDDR EthBusy_REG (.D(EthBusy), .Q1(EthBusyReg), .C(CLK_PHASE), .CE(1'b1), .R(1'b0));
IBUFDS EthBusy_BUFFER	(.I(ETH_0_P), .IB(ETH_0_N), .O(EthBusy)); 
//Outgoing write enable
ODDR EthEn_REG (.Q(EthEnReg), .D1(EthEn), .D2(EthEn), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthEn_BUFFER (.O(ETH_1_P), .OB(ETH_1_N), .I(EthEnReg)); 
//Clock for the input to the FIFO in the Zest-ET1 FPGA 
OBUFDS EthCLK_BUFFER (.O(ETH_8_P), .OB(ETH_8_N), .I(CLK));
//Reset signal
ODDR EthRst_REG (.Q(EthRstReg), .D1(EthRst), .D2(EthRst), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthRst_BUFFER (.O(ETH_2_P), .OB(ETH_2_N), .I(EthRstReg));
//16-bit wide data word
ODDR EthDat0_REG (.Q(EthDat0Reg), .D1(EthDat[0]), .D2(EthDat[0]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat0_BUFFER (.O(ETH_3_P), .OB(ETH_3_N), .I(EthDat0Reg));
ODDR EthDat1_REG (.Q(EthDat1Reg), .D1(EthDat[1]), .D2(EthDat[1]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat1_BUFFER (.O(ETH_4_P), .OB(ETH_4_N), .I(EthDat1Reg));
ODDR EthDat2_REG (.Q(EthDat2Reg), .D1(EthDat[2]), .D2(EthDat[2]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat2_BUFFER (.O(ETH_5_P), .OB(ETH_5_N), .I(EthDat2Reg));
ODDR EthDat3_REG (.Q(EthDat3Reg), .D1(EthDat[3]), .D2(EthDat[3]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat3_BUFFER (.O(ETH_6_P), .OB(ETH_6_N), .I(EthDat3Reg));
ODDR EthDat4_REG (.Q(EthDat4Reg), .D1(EthDat[4]), .D2(EthDat[4]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat4_BUFFER (.O(ETH_7_P), .OB(ETH_7_N), .I(EthDat4Reg));
ODDR EthDat5_REG (.Q(EthDat5Reg), .D1(EthDat[5]), .D2(EthDat[5]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat5_BUFFER (.O(ETH_9_P), .OB(ETH_9_N), .I(EthDat5Reg));
ODDR EthDat6_REG (.Q(EthDat6Reg), .D1(EthDat[6]), .D2(EthDat[6]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat6_BUFFER (.O(ETH_10_P), .OB(ETH_10_N), .I(EthDat6Reg));
ODDR EthDat7_REG (.Q(EthDat7Reg), .D1(EthDat[7]), .D2(EthDat[7]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat7_BUFFER (.O(ETH_11_P), .OB(ETH_11_N), .I(EthDat7Reg));
ODDR EthDat8_REG (.Q(EthDat8Reg), .D1(EthDat[8]), .D2(EthDat[8]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat8_BUFFER (.O(ETH_12_P), .OB(ETH_12_N), .I(EthDat8Reg));
ODDR EthDat9_REG (.Q(EthDat9Reg), .D1(EthDat[9]), .D2(EthDat[9]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDat9_BUFFER (.O(ETH_13_P), .OB(ETH_13_N), .I(EthDat9Reg));
ODDR EthDatA_REG (.Q(EthDatAReg), .D1(EthDat[10]), .D2(EthDat[10]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatA_BUFFER (.O(ETH_14_P), .OB(ETH_14_N), .I(EthDatAReg));
ODDR EthDatB_REG (.Q(EthDatBReg), .D1(EthDat[11]), .D2(EthDat[11]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatB_BUFFER (.O(ETH_15_P), .OB(ETH_15_N), .I(EthDatBReg));
ODDR EthDatC_REG (.Q(EthDatCReg), .D1(EthDat[12]), .D2(EthDat[12]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatC_BUFFER (.O(ETH_16_P), .OB(ETH_16_N), .I(EthDatCReg));
ODDR EthDatD_REG (.Q(EthDatDReg), .D1(EthDat[13]), .D2(EthDat[13]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatD_BUFFER (.O(ETH_17_P), .OB(ETH_17_N), .I(EthDatDReg));
ODDR EthDatE_REG (.Q(EthDatEReg), .D1(EthDat[14]), .D2(EthDat[14]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatE_BUFFER (.O(ETH_18_P), .OB(ETH_18_N), .I(EthDatEReg));
ODDR EthDatF_REG (.Q(EthDatFReg), .D1(EthDat[15]), .D2(EthDat[15]), .C(CLK), .CE(1'b1), .R(1'b0));
OBUFDS EthDatF_BUFFER (.O(ETH_19_P), .OB(ETH_19_N), .I(EthDatFReg));

wire [7:0] TxD_data;  //UART outgoing data byte
wire [7:0] RxD_data;  //UART incoming data byte

reg [9:0] FastORmux;
reg [7:0] BufClrStrm;
always @ (BufMngrType or FastORreg or U1Reg) begin
	if (BufMngrType) begin
		FastORmux[0] = FastORreg[0];
		FastORmux[1] = 1'b0;
		FastORmux[2] = FastORreg[2];
		FastORmux[3] = 1'b0;
		FastORmux[4] = 1'b0;
		FastORmux[5] = FastORreg[5];
		FastORmux[6] = 1'b0;
		FastORmux[7] = FastORreg[7];
		FastORmux[8] = FastORreg[8];
		FastORmux[9] = FastORreg[9];
		BufClrStrm[0] = U1Reg[0];
		BufClrStrm[1] = FastORreg[1];
		BufClrStrm[2] = U1Reg[1];
		BufClrStrm[3] = FastORreg[3];
		BufClrStrm[4] = FastORreg[4];
		BufClrStrm[5] = U1Reg[2];
		BufClrStrm[6] = FastORreg[6];
		BufClrStrm[7] = U1Reg[3];
	end else begin
		FastORmux = FastORreg;
		BufClrStrm = 0;
	end
end

//Instantiate the event builder code here
assign TrgEx = 1'b0;
FPGA_EvtBuild pCT_DAQ(Diagnostic1,Diagnostic2,TrgInhibit,ClkVsRst,BufMngrType,ResetDly,IncDir,EnableInc,RunInProg,EthBusyReg,EthEn,EthDat,EthRst,
		TxD_start,TxD_data,TRIGGER,CMD,CMDE,RSTfpga,
		TxD_busy,RxD_data_ready,RxD_data,TrgEx,DataReg,CLK,FastORmux,ResetExt,
		SDAin,SDAout,SDAen,SCLout,SCLen,SDAin2,SCLin2,CLK_SDAin,CLK_SCLin,BufClrStrm);

//Generate a bounce-free reset from the manual reset button on the ML605 board
debounce ExternalResetDebounce(.clock(CLK), .buttonPress(RESET_SW), .pulse(ResetExt));

//Instantiate the UART receiver and transmitter
async_receiver pcUART_serial_rx(.clk(CLK), .RxD(UART_RX), .RxD_data_ready(RxD_data_ready), .RxD_data(RxD_data));

async_transmitter pcUART_serial_tx(.clk(CLK), .TxD(UART_TX), .TxD_start(TxD_start), .TxD_data(TxD_data), .TxD_busy(TxD_busy));

reg [7:0] receivedByte;

always @ (posedge CLK)
begin
if (RxD_data_ready)
begin
	receivedByte <= RxD_data;
end
end
assign GPIO_LED = receivedByte;   //Make the LEDs blink according to the incoming UART data byte

assign RESET_LED = RESET_SW;

//EnergyTrgGen EnergyTrgGen_U(Diagnostic1, CLKDIGbuf, ResetExt);

endmodule

//Divide down the digitizer clock to send a signal out to an external pulser, for test purposes only
//module EnergyTrgGen(
//    output Trigger,
//    input Clock,
//    input Reset
//    );
//
//reg Trigger;
//reg [18:0] Cnt;
//
//always @ (posedge Clock) begin
//	if (Reset) begin
//		Cnt <= 0;
//		Trigger <= 0;
//	end else begin
//		Cnt <= Cnt + 1;
//		if (Cnt == 19'b1111111111111111111) Trigger <= 1'b1;
//		else Trigger <= 1'b0;
//	end
//end
//
//endmodule