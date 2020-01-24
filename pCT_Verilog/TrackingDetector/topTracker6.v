`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:18:00 10/26/2012 
// Design Name: 
// Module Name:    topTestBoard 
// Project Name: pCT FPGA Chip Testing
// Target Devices: Spartan-6
// Tool versions: 
// Description: Runs on the test board to operate 6 front-end chips.
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module top(CLKIN_P, CLKIN_N,
           CLKOUT_P,  CLKOUT_N,
			  TReq0_P, TReq0_N,
			  TReq1_P, TReq1_N,
			  TReq2_P, TReq2_N,
			  TReq3_P, TReq3_N,
			  TReq4_P, TReq4_N,
			  TReq5_P, TReq5_N,
			  Data0_P, Data0_N,
			  Data1_P, Data1_N,
			  Data2_P, Data2_N,
			  Data3_P, Data3_N,
			  Data4_P, Data4_N,
			  Data5_P, Data5_N,
			  TrgIn_P, TrgIn_N,
			  CmdIn_P, CmdIn_N,
			  RST_P, RST_N,
			  Tack_P, Tack_N,
			  CMD_P, CMD_N,
			  FastOR_P, FastOR_N,
			  Dout_P, Dout_N,
			  U1P, U1N,
			  U2P, U2N,
			  U3P, U3N
    );

input CLKIN_P, CLKIN_N;
output CLKOUT_P, CLKOUT_N;
input TReq0_P, TReq0_N;
input TReq1_P, TReq1_N;
input TReq2_P, TReq2_N;
input TReq3_P, TReq3_N;
input TReq4_P, TReq4_N;
input TReq5_P, TReq5_N;
input Data0_P, Data0_N;
input Data1_P, Data1_N;
input Data2_P, Data2_N;
input Data3_P, Data3_N;
input Data4_P, Data4_N;
input Data5_P, Data5_N;
input TrgIn_P, TrgIn_N;
input CmdIn_P, CmdIn_N;
input RST_P, RST_N;
output Tack_P, Tack_N;
output CMD_P, CMD_N;
output FastOR_P, FastOR_N;
output Dout_P, Dout_N;
output U1P, U1N;
output U2P, U2N;
output U3P, U3N;

parameter [3:0] Address=4'b0001;    //Assign this address to the board

DCM DCM_INSTANCE(.CLK_IN1_P(CLKIN_P),.CLK_IN1_N(CLKIN_N),.CLK_OUT1(CLK_DLY),.CLK_OUT2(CLK), .CLK_OUT3(CLK_DLY2), .RESET(1'b0),.LOCKED(LOCKED));
//CLK_DLY phase delay = 60 degrees for 100 MHz

ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_INST (.Q(CLKOUT), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT_BUFFER (.O(CLKOUT_P), .OB(CLKOUT_N), .I(CLKOUT));

IBUFDS TREQ0_BUFFER	(.I(TReq0_P), .IB(TReq0_N), .O(TReq[0]));
IBUFDS TREQ1_BUFFER	(.I(TReq1_P), .IB(TReq1_N), .O(TReq[1]));
IBUFDS TREQ2_BUFFER	(.I(TReq2_P), .IB(TReq2_N), .O(TReq[2]));
IBUFDS TREQ3_BUFFER	(.I(TReq3_P), .IB(TReq3_N), .O(TReq[3]));
IBUFDS TREQ4_BUFFER	(.I(TReq4_P), .IB(TReq4_N), .O(TReq[4]));
IBUFDS TREQ5_BUFFER	(.I(TReq5_P), .IB(TReq5_N), .O(TReq[5]));

IDDR2 Treq0_Reg (.D(TReq[0]), .Q0(TReqReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq1_Reg (.D(TReq[1]), .Q0(TReqReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq2_Reg (.D(TReq[2]), .Q0(TReqReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq3_Reg (.D(TReq[3]), .Q0(TReqReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq4_Reg (.D(TReq[4]), .Q0(TReqReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Treq5_Reg (.D(TReq[5]), .Q0(TReqReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS Data0_BUFFER	(.I(Data0_P), .IB(Data0_N), .O(Data[0]));

IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data0_DELAY (.IDATAIN(Data[0]), .DATAOUT(DataNDLY[0]));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data1_DELAY (.IDATAIN(Data[1]), .DATAOUT(DataNDLY[1])); 
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data2_DELAY (.IDATAIN(Data[2]), .DATAOUT(DataNDLY[2]));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data3_DELAY (.IDATAIN(Data[3]), .DATAOUT(DataNDLY[3]));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data4_DELAY (.IDATAIN(Data[4]), .DATAOUT(DataNDLY[4]));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.IDELAY_VALUE(1), .DELAY_SRC("IDATAIN")) Data5_DELAY (.IDATAIN(Data[5]), .DATAOUT(DataNDLY[5]));

IDDR2 Data0_Reg (.D(DataNDLY[0]), .Q0(DataReg[0]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data1_Reg (.D(DataNDLY[1]), .Q0(DataReg[1]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data2_Reg (.D(DataNDLY[2]), .Q0(DataReg[2]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data3_Reg (.D(DataNDLY[3]), .Q0(DataReg[3]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data4_Reg (.D(DataNDLY[4]), .Q0(DataReg[4]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 Data5_Reg (.D(DataNDLY[5]), .Q0(DataReg[5]), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS Data1_BUFFER	(.I(Data1_P), .IB(Data1_N), .O(Data[1]));
IBUFDS Data2_BUFFER	(.I(Data2_P), .IB(Data2_N), .O(Data[2]));
IBUFDS Data3_BUFFER	(.I(Data3_P), .IB(Data3_N), .O(Data[3]));
IBUFDS Data4_BUFFER	(.I(Data4_P), .IB(Data4_N), .O(Data[4]));
IBUFDS Data5_BUFFER	(.I(Data5_P), .IB(Data5_N), .O(Data[5]));

IBUFDS TRGIN_BUFFER (.I(TrgIn_P), .IB(TrgIn_N), .O(TrgIn));
IDDR2 TRGIN_Reg (.D(TrgIn), .Q0(TrgInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS CMDIN_BUFFER (.I(CmdIn_P), .IB(CmdIn_N), .O(CmdIn));
IDDR2 CMDIN_Reg (.D(CmdIn), .Q0(CmdInReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS RST_BUFFER (.I(RST_P), .IB(RST_N), .O(Reset));
IDDR2 RST_Reg (.D(Reset), .Q0(ResetReg), .C0(CLK_DLY2), .C1(~CLK_DLY2), .CE(1'b1), .R(1'b0), .S(1'b0));

OBUFDS TACK_BUFFER (.O(Tack_P), .OB(Tack_N), .I(TACKDLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_TACK (.Q(TACKLATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Tack), .D1(Tack), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(37), .DELAY_SRC("ODATAIN")) TACK_DELAY (.ODATAIN(TACKLATCH), .DOUT(TACKDLY));

OBUFDS CMD_BUFFER (.O(CMD_P), .OB(CMD_N), .I(CMDDLY));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_CMD (.Q(CMDLATCH), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CMDFAB), .D1(CMDFAB), .R(1'b0), .S(1'b0));
IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(37), .DELAY_SRC("ODATAIN")) CMD_DELAY (.ODATAIN(CMDLATCH), .DOUT(CMDDLY));
// Command delay = 37 for 100 MHz

OBUFDS FASTOR_BUFFER (.O(FastOR_P), .OB(FastOR_N), .I(FastORReg));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_FASTOR (.Q(FastORReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(FastOR), .D1(FastOR), .R(1'b0), .S(1'b0));

ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_Dout (.Q(DoutReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Dout), .D1(Dout), .R(1'b0), .S(1'b0));
OBUFDS DOUT_BUFFER (.O(Dout_P), .OB(Dout_N), .I(DoutReg));

OBUFDS BufClr_BUFFER (.O(U1P), .OB(U1N), .I(DataReg[0]));  

OBUFDS U2_BUFFER (.O(U2P), .OB(U2N), .I(CMDFAB));   
//ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
//		ODDR2_U2 (.Q(U2Reg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(CmdInReg), .D1(CmdInReg), .R(1'b0), .S(1'b0));
//OBUFDS U2_BUFFER (.O(U2P), .OB(U2N), .I(U2DLY));  //Output for debug scope signals
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(37), .DELAY_SRC("ODATAIN")) U2_DELAY (.ODATAIN(U2Reg), .DOUT(U2DLY));

OBUFDS U3_BUFFER (.O(U3P), .OB(U3N), .I(TrgInReg));
//ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
//		ODDR2_U3 (.Q(U3Reg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(BufClr), .D1(BufClr), .R(1'b0), .S(1'b0));
//OBUFDS U3_BUFFER (.O(U3P), .OB(U3N), .I(U3DLY));  //Output for debug scope signals
//IODELAY2 #(.IDELAY_TYPE("FIXED"),.ODELAY_VALUE(37), .DELAY_SRC("ODATAIN")) U3_DELAY (.ODATAIN(U3Reg), .DOUT(U3DLY));

wire [5:0] TReq, Data, DataNDLY, DataReg, TReqReg;
            
FPGA_DAQ_TKR6 FPGA_DAQ_TKR6_I1(Dout,FastOR,CMDFAB,Tack, CLK,ResetReg,CmdInReg,TrgInReg,DataReg,TReqReg,Address);
endmodule
