//////////////////////////////////////////////////////////////////////////////////
// Company:   UCSC
// Engineer:  R. Johnson
// 
// Create Date:    7/12/2013 
// Design Name:    pCT_DAQ_TrackerV
// Module Name:    topTrackerV 
// Project Name:   pCT 
// Target Devices: Spartan-6
//  
// Description: Top level module.  Runs on the pCT Tracker front-end board
// September 19, 2013: 	direct the hard reset signal from the even FPGA to the odd FPGA for T boards
// November 7, 2013:  	implement the buffer clear signal
// March 16, 2014: simplify the buffer clear mechanism
// August 22, 2014: major change in sending signals to ASICs.  Remove OLogic delays on CMD and instead
//                  shift the phase of the outgoing clock.  
//
//////////////////////////////////////////////////////////////////////////////////
module topTrackerV(CLKIN_P, CLKIN_N,
           CLKOUT1_P,  CLKOUT1_N,
           CLKOUT2_P,  CLKOUT2_N,
			  TReq0_P, TReq0_N,
			  TReq1_P, TReq1_N,
			  TReq2_P, TReq2_N,
			  TReq3_P, TReq3_N,
			  TReq4_P, TReq4_N,
			  TReq5_P, TReq5_N,
			  TReq6_P, TReq6_N,
			  TReq7_P, TReq7_N,
			  TReq8_P, TReq8_N,
			  TReq9_P, TReq9_N,
			  TReqA_P, TReqA_N,
			  TReqB_P, TReqB_N,
			  Data0_P, Data0_N,
			  Data1_P, Data1_N,
			  Data2_P, Data2_N,
			  Data3_P, Data3_N,
			  Data4_P, Data4_N,
			  Data5_P, Data5_N,
			  Data6_P, Data6_N,
			  Data7_P, Data7_N,
			  Data8_P, Data8_N,
			  Data9_P, Data9_N,
			  DataA_P, DataA_N,
			  DataB_P, DataB_N,
			  TrgIn_P, TrgIn_N,
			  CmdIn_P, CmdIn_N,
			  RST_P, RST_N,
			  Tack1_P, Tack1_N,
			  Tack2_P, Tack2_N,
			  CMD1_P, CMD1_N,
			  CMD2_P, CMD2_N,
			  RSTchp1_P, RSTchp1_N,
			  RSTchp2_P, RSTchp2_N,
			  FastOR_P, FastOR_N,
			  Dout_P, Dout_N,
			  U1_P, U1_N,
			  PWREN,
			  Address0,Address1,Address2,Address3,
			  Debug1, Debug2, Debug3, Debug4,
			  INTER1P, INTER1N, INTER2P, INTER2N
    );

output INTER1P, INTER1N;
input INTER2P, INTER2N;
output Debug1, Debug2, Debug3, Debug4;
input Address0, Address1, Address2, Address3;
output PWREN;
input CLKIN_P, CLKIN_N;
output CLKOUT1_P, CLKOUT1_N;
output CLKOUT2_P, CLKOUT2_N;
input TReq0_P, TReq0_N;
input TReq1_P, TReq1_N;
input TReq2_P, TReq2_N;
input TReq3_P, TReq3_N;
input TReq4_P, TReq4_N;
input TReq5_P, TReq5_N;
input TReq6_P, TReq6_N;
input TReq7_P, TReq7_N;
input TReq8_P, TReq8_N;
input TReq9_P, TReq9_N;
input TReqA_P, TReqA_N;
input TReqB_P, TReqB_N;
input Data0_P, Data0_N;
input Data1_P, Data1_N;
input Data2_P, Data2_N;
input Data3_P, Data3_N;
input Data4_P, Data4_N;
input Data5_P, Data5_N;
input Data6_P, Data6_N;
input Data7_P, Data7_N;
input Data8_P, Data8_N;
input Data9_P, Data9_N;
input DataA_P, DataA_N;
input DataB_P, DataB_N;
input TrgIn_P, TrgIn_N;
input CmdIn_P, CmdIn_N;
input RST_P, RST_N;
output Tack1_P, Tack1_N;
output CMD1_P, CMD1_N;
output Tack2_P, Tack2_N;
output CMD2_P, CMD2_N;
output FastOR_P, FastOR_N;
output Dout_P, Dout_N;
output U1_P, U1_N;
output RSTchp1_P, RSTchp1_N;
output RSTchp2_P, RSTchp2_N;

reg FastOR;
wire [3:0] Address;
assign Address[0] = Address0;
assign Address[1] = Address1;
assign Address[2] = Address2;
assign Address[3] = Address3;

//Digital clock manager, derives the local clocks from the 100 MHz clock that arrives from the event builder
DCM DCM_INSTANCE(.CLK_IN1_P(CLKIN_P),.CLK_IN1_N(CLKIN_N),.CLK_OUT1(CLK_DLY_CMD),.CLK_OUT2(CLK), .CLK_OUT3(CLK_DLY_CMD_bar), .RESET(1'b0),.LOCKED(LOCKED));
//Clocks 1 and 3 are used ONLY to send a phase shifted clock to the ASICs
//Best phases appear to be around 90 and 270 respectively (August 22, 2014), although this is not intuitively understood!
//Clock 2 is used everywhere else. . .
assign CLK_DLY = CLK;     //Clock used to receive signals from the ASICs
assign CLK_DLY2 = CLK;    //Clock used to receive signals from the event builder

//Output clocks going to ASICs
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_CLK1 (.Q(CLKOUT1), .C0(CLK_DLY_CMD), .C1(CLK_DLY_CMD_bar), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT1_BUFFER (.O(CLKOUT1_P), .OB(CLKOUT1_N), .I(CLKOUT1));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_CLK2 (.Q(CLKOUT2), .C0(CLK_DLY_CMD), .C1(CLK_DLY_CMD_bar), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT2_BUFFER (.O(CLKOUT2_P), .OB(CLKOUT2_N), .I(CLKOUT2));
//ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
//		ODDR2_CLK1 (.Q(CLKOUT1), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
//OBUFDS CLKOUT1_BUFFER (.O(CLKOUT1_P), .OB(CLKOUT1_N), .I(CLKOUT1));
//ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
//		ODDR2_CLK2 (.Q(CLKOUT2), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
//OBUFDS CLKOUT2_BUFFER (.O(CLKOUT2_P), .OB(CLKOUT2_N), .I(CLKOUT2));

wire [11:0] TReq, Data, DataNDLY, DataReg, TReqReg;

//LVDS buffers for the Fast-OR trigger request signals from the ASICs
IBUFDS TREQ0_BUFFER	(.I(TReq0_P), .IB(TReq0_N), .O(TReq[0]));
IBUFDS TREQ1_BUFFER	(.I(TReq1_P), .IB(TReq1_N), .O(TReq[1]));
IBUFDS TREQ2_BUFFER	(.I(TReq2_P), .IB(TReq2_N), .O(TReq[2]));
IBUFDS TREQ3_BUFFER	(.I(TReq3_P), .IB(TReq3_N), .O(TReq[3]));
IBUFDS TREQ4_BUFFER	(.I(TReq4_P), .IB(TReq4_N), .O(TReq[4]));
IBUFDS TREQ5_BUFFER	(.I(TReq5_P), .IB(TReq5_N), .O(TReq[5]));
IBUFDS TREQ6_BUFFER	(.I(TReq6_P), .IB(TReq6_N), .O(TReq[6]));
IBUFDS TREQ7_BUFFER	(.I(TReq7_P), .IB(TReq7_N), .O(TReq[7]));
IBUFDS TREQ8_BUFFER	(.I(TReq8_P), .IB(TReq8_N), .O(TReq[8]));
IBUFDS TREQ9_BUFFER	(.I(TReq9_P), .IB(TReq9_N), .O(TReq[9]));
IBUFDS TREQa_BUFFER	(.I(TReqA_P), .IB(TReqA_N), .O(TReq[10]));
IBUFDS TREQb_BUFFER	(.I(TReqB_P), .IB(TReqB_N), .O(TReq[11]));

//Register the TReq signals
IDDR2 Treq0_Reg (.D(TReq[0]), .Q0(TReqReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq1_Reg (.D(TReq[1]), .Q0(TReqReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq2_Reg (.D(TReq[2]), .Q0(TReqReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq3_Reg (.D(TReq[3]), .Q0(TReqReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq4_Reg (.D(TReq[4]), .Q0(TReqReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq5_Reg (.D(TReq[5]), .Q0(TReqReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq6_Reg (.D(TReq[6]), .Q0(TReqReg[6]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq7_Reg (.D(TReq[7]), .Q0(TReqReg[7]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq8_Reg (.D(TReq[8]), .Q0(TReqReg[8]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq9_Reg (.D(TReq[9]), .Q0(TReqReg[9]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treqa_Reg (.D(TReq[10]), .Q0(TReqReg[10]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treqb_Reg (.D(TReq[11]), .Q0(TReqReg[11]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

// LVDS buffers for data coming in from the ASICs
IBUFDS Data0_BUFFER	(.I(Data0_P), .IB(Data0_N), .O(Data[0]));
IBUFDS Data1_BUFFER	(.I(Data1_P), .IB(Data1_N), .O(Data[1]));
IBUFDS Data2_BUFFER	(.I(Data2_P), .IB(Data2_N), .O(Data[2]));
IBUFDS Data3_BUFFER	(.I(Data3_P), .IB(Data3_N), .O(Data[3]));
IBUFDS Data4_BUFFER	(.I(Data4_P), .IB(Data4_N), .O(Data[4]));
IBUFDS Data5_BUFFER	(.I(Data5_P), .IB(Data5_N), .O(Data[5]));
IBUFDS Data6_BUFFER	(.I(Data6_P), .IB(Data6_N), .O(Data[6]));
IBUFDS Data7_BUFFER	(.I(Data7_P), .IB(Data7_N), .O(Data[7]));
IBUFDS Data8_BUFFER	(.I(Data8_P), .IB(Data8_N), .O(Data[8]));
IBUFDS Data9_BUFFER	(.I(Data9_P), .IB(Data9_N), .O(Data[9]));
IBUFDS DataA_BUFFER	(.I(DataA_P), .IB(DataA_N), .O(Data[10]));
IBUFDS DataB_BUFFER	(.I(DataB_P), .IB(DataB_N), .O(Data[11]));

// Calibrated input delays for the data coming back from the ASICs
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data0_DELAY (.IDATAIN(Data[0]), .DATAOUT(DataNDLY[0]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN), .BUSY(CALBUSY0));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data1_DELAY (.IDATAIN(Data[1]), .DATAOUT(DataNDLY[1]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data2_DELAY (.IDATAIN(Data[2]), .DATAOUT(DataNDLY[2]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data3_DELAY (.IDATAIN(Data[3]), .DATAOUT(DataNDLY[3]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data4_DELAY (.IDATAIN(Data[4]), .DATAOUT(DataNDLY[4]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data5_DELAY (.IDATAIN(Data[5]), .DATAOUT(DataNDLY[5]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data6_DELAY (.IDATAIN(Data[6]), .DATAOUT(DataNDLY[6]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN), .BUSY(CALBUSY6));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data7_DELAY (.IDATAIN(Data[7]), .DATAOUT(DataNDLY[7]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data8_DELAY (.IDATAIN(Data[8]), .DATAOUT(DataNDLY[8]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	Data9_DELAY (.IDATAIN(Data[9]), .DATAOUT(DataNDLY[9]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	DataA_DELAY (.IDATAIN(Data[10]), .DATAOUT(DataNDLY[10]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));
IODELAY2 #(.IDELAY_TYPE("VARIABLE_FROM_HALF_MAX"),.DELAY_SRC("IDATAIN"), .DATA_RATE("DDR")) 
	DataB_DELAY (.IDATAIN(Data[11]), .DATAOUT(DataNDLY[11]), .CLK(CLK), .IOCLK0(CLK_DLY), .IOCLK1(~CLK_DLY), .CAL(CALIO), 
					.INC(CALINC), .RST(CALRST), .CE(CALEN));

// Register the data from the ASICs					
IDDR2 Data0_Reg (.D(DataNDLY[0]), .Q0(DataReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data1_Reg (.D(DataNDLY[1]), .Q0(DataReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data2_Reg (.D(DataNDLY[2]), .Q0(DataReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data3_Reg (.D(DataNDLY[3]), .Q0(DataReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data4_Reg (.D(DataNDLY[4]), .Q0(DataReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data5_Reg (.D(DataNDLY[5]), .Q0(DataReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data6_Reg (.D(DataNDLY[6]), .Q0(DataReg[6]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data7_Reg (.D(DataNDLY[7]), .Q0(DataReg[7]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data8_Reg (.D(DataNDLY[8]), .Q0(DataReg[8]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data9_Reg (.D(DataNDLY[9]), .Q0(DataReg[9]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 DataA_Reg (.D(DataNDLY[10]), .Q0(DataReg[10]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 DataB_Reg (.D(DataNDLY[11]), .Q0(DataReg[11]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

//Receive the LVDS trigger, command, and reset signals coming from the event builder
IBUFDS TRGIN_BUFFER (.I(TrgIn_P), .IB(TrgIn_N), .O(TrgIn));
IDDR2 TRGIN_Reg (.D(TrgIn), .Q0(TrgInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS CMDIN_BUFFER (.I(CmdIn_P), .IB(CmdIn_N), .O(CmdIn));
IDDR2 CMDIN_Reg (.D(CmdIn), .Q0(CmdInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS RST_BUFFER (.I(RST_P), .IB(RST_N), .O(Reset));
IDDR2 RST_Reg (.D(Reset), .Q0(ResetReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

//Trigger acknowledge going to ASICs
OBUFDS TACK1_BUFFER (.O(Tack1_P), .OB(Tack1_N), .I(TACK1LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_TACK1 (.Q(TACK1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Tack[0]), .D1(Tack[0]), .R(1'b0), .S(1'b0));
OBUFDS TACK2_BUFFER (.O(Tack2_P), .OB(Tack2_N), .I(TACK2LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_TACK2 (.Q(TACK2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Tack[1]), .D1(Tack[1]), .R(1'b0), .S(1'b0));

//Commands to ASICs
//BUFIO2 CMD1_IOCLK (.I(CLK), .IOCLK(CLKIO1));
OBUFDS CMD1_BUFFER (.O(CMD1_P), .OB(CMD1_N), .I(CMD1LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_CMD1 (.Q(CMD1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CMDFAB), .D1(CMDFAB), .R(1'b0), .S(1'b0));
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(192), .DELAY_SRC("ODATAIN")) CMD1_DELAY (.ODATAIN(CMD1LATCH), .DOUT(CMD1DLY));
//BUFIO2 CMD2_IOCLK (.I(CLK), .IOCLK(CLKIO2));
OBUFDS CMD2_BUFFER (.O(CMD2_P), .OB(CMD2_N), .I(CMD2LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_CMD2 (.Q(CMD2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CMDFAB), .D1(CMDFAB), .R(1'b0), .S(1'b0));
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(192), .DELAY_SRC("ODATAIN")) CMD2_DELAY (.ODATAIN(CMD2LATCH), .DOUT(CMD2DLY));
// Command delay = 98 should be about 5ns maximum 

//Reset signals to ASICs
OBUFDS RST1_BUFFER (.O(RSTchp1_P), .OB(RSTchp1_N), .I(Rst1LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_RST1 (.Q(Rst1LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(RSTchips), .D1(RSTchips), .R(1'b0), .S(1'b0));
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(248), .DELAY_SRC("ODATAIN")) RST1_DELAY (.ODATAIN(Rst1LATCH), .DOUT(RSTchp1DLY));
OBUFDS RST2_BUFFER (.O(RSTchp2_P), .OB(RSTchp2_N), .I(Rst2LATCH));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_RST2 (.Q(Rst2LATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(RSTchips), .D1(RSTchips), .R(1'b0), .S(1'b0));
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(248), .DELAY_SRC("ODATAIN")) RST2_DELAY (.ODATAIN(Rst2LATCH), .DOUT(RSTchp2DLY));

//Send the trigger signal out to the event builder, or in the case of T boards, the buffer clear signal, if that option is chosen
OBUFDS FASTOR_BUFFER (.O(FastOR_P), .OB(FastOR_N), .I(FastORReg));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_FASTOR (.Q(FastORReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(FastOR), .D1(FastOR), .R(1'b0), .S(1'b0));

//Send the output data stream.  On T boards the even and odd FPGAs route this to different wires on the DVI connector
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_Dout (.Q(DoutReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Dout), .D1(Dout), .R(1'b0), .S(1'b0));
OBUFDS DOUT_BUFFER (.O(Dout_P), .OB(Dout_N), .I(DoutReg));

//Buffer clear signals.  This net is connected only on the V boards!
OBUFDS U1_BUFFER (.O(U1_P), .OB(U1_N), .I(U1reg));  
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_TOT (.Q(U1reg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(BufClrStream), .D1(BufClrStream), .R(1'b0), .S(1'b0));

//Buffers and logic for the signals passed between the two FPGAs
reg Inter1Out, ResetLocal;
OBUFDS IntBuffer1 (.O(INTER1P), .OB(INTER1N), .I(Inter1Out));
IBUFDS IntBuffer2 (.I(INTER2P), .IB(INTER2N), .O(Inter2In));
IDDR2 Int2_Reg (.D(Inter2In), .Q0(Inter2InReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));

//Logic to implement differences between V board and T boards, and the two different FPGAs on T boards
reg PWREN;
reg BufClrOdd;
always @ (Address or PWREN_f or FastORlocal or ResetReg or Inter2InReg or BufClrStream or OutputUsage or BufClr) begin
	if (Address < 4) begin    //V boards
		PWREN = PWREN_f;
		FastOR = FastORlocal;
		Inter1Out = 1'b0;
		ResetLocal = ResetReg;
		BufClrOdd = 1'b0;
	end else begin			  //T boards
		if (!Address[0]) begin  						//T board even address FPGA (master)
			PWREN = PWREN_f;    						//Enable for all except the 3A 2.5V power supplies
			if (OutputUsage) begin
				FastOR = BufClrStream;					//Send the buffer clear signal instead of the trigger signal
				BufClrOdd = Inter2InReg;				//Use the buffer clear signal from the odd FPGA
			end else begin
				FastOR = FastORlocal | Inter2InReg;   	//Include the FastOR signal from the odd FPGA
				BufClrOdd = 1'b0;
			end
			Inter1Out = ResetReg;   					//Copy the hard reset to the odd FPGA
			ResetLocal = ResetReg;  					//Use the hard reset directly for the even FPGA
		end else begin         							//T board odd address FPGA (slave)
			PWREN = !PWREN_f;   						//3A, 2.5V power supplies have an inverted enable controlled by the odd FPGA
			if (OutputUsage) begin
				Inter1Out = BufClr;						//Send the buffer clear signal to the even FPGA
			end else begin
				Inter1Out = FastORlocal;    			//Send the FastOR to the even FPGA
			end
			FastOR = FastORlocal;						//Does nothing here, but needed to avoid a latch condition
			ResetLocal = Inter2InReg;   				//Receive the hard reset from the even FPGA
			BufClrOdd = 1'b0;
		end
	end
end

//Main program for the Tracker FPGAs
wire [1:0] Tack;
FPGA_DAQ_TKR12 FPGA_DAQ_TKR(Debug1,Debug2,Debug3,Debug4,OutputUsage,BufClr,RSTchips,ResetSoft,CALIO,CALEN,CALINC,CALRST,PWREN_f,
														Dout,FastORlocal,CMDFAB,Tack,CLK,ResetLocal,CmdInReg,TrgInReg,DataReg,TReqReg,Address);

//Program to multiplex the buffer clear signals from both FPGAs on T boards.  This does nothing on odd address T board FPGAs.
BufSgnl BufSgnlVT(BufClrStream, BufClr, BufClrOdd, ResetSoft, CLK, Address);

endmodule
