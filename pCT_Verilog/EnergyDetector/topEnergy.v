//////////////////////////////////////////////////////////////////////////////////
// Company: UCSC/SCIPP
// Engineer: R. Johnson
// 
// Create Date:    1/10/2013 
// Design Name: 
// Module Name:    topEnergy 
// Project Name: pCT FPGA Energy Detector Digitizer Board
// Target Devices: Spartan-6
// Tool versions: 
// Description: Runs on the digitizer board FPGA to handle 3 scintillator channels
//
// Revision: 1    Eliminate redundant clock
//           2    Add second data line  2/27/14
//
//////////////////////////////////////////////////////////////////////////////////
module topEnergy(CLKIN_P, CLKIN_N,
	CmdIn_P, CmdIn_N,
	TrgIn_P, TrgIn_N,
	TREQOUT_P, TREQOUT_N,
	Dout_P, Dout_N,
	SDApin, SCLpin,
	TREQ0P, TREQ0N, TREQ1P, TREQ1N, TREQ2P, TREQ2N,
	DATA2_P, DATA2_N,
	CLK0P, CLK0N, CLK1P, CLK1N, CLK2P, CLK2N,
	CLKDIGI_P, CLKDIGI_N,
	D0DIG0, D1DIG0, D2DIG0, D3DIG0, D4DIG0, D5DIG0, D6DIG0, D7DIG0, D8DIG0, D9DIG0, D10DIG0, D11DIG0, D12DIG0, D13DIG0, OTRDIG0,
	D0DIG1, D1DIG1, D2DIG1, D3DIG1, D4DIG1, D5DIG1, D6DIG1, D7DIG1, D8DIG1, D9DIG1, D10DIG1, D11DIG1, D12DIG1, D13DIG1, OTRDIG1,
	D0DIG2, D1DIG2, D2DIG2, D3DIG2, D4DIG2, D5DIG2, D6DIG2, D7DIG2, D8DIG2, D9DIG2, D10DIG2, D11DIG2, D12DIG2, D13DIG2, OTRDIG2,
	Address0,Address1,Address2,Address3
    );
	input CLKIN_P, CLKIN_N;
	input	CmdIn_P, CmdIn_N;
	input	TrgIn_P, TrgIn_N;
	output 	TREQOUT_P, TREQOUT_N;
	output	Dout_P, Dout_N;
	inout SDApin, SCLpin;
	input TREQ0P, TREQ0N, TREQ1P, TREQ1N, TREQ2P, TREQ2N;
	output DATA2_P, DATA2_N;
	output CLK0P, CLK0N, CLK1P, CLK1N, CLK2P, CLK2N;
	input CLKDIGI_P, CLKDIGI_N;
	input D0DIG0, D1DIG0, D2DIG0, D3DIG0, D4DIG0, D5DIG0, D6DIG0, D7DIG0, D8DIG0, D9DIG0, D10DIG0, D11DIG0, D12DIG0, D13DIG0, OTRDIG0;
	input D0DIG1, D1DIG1, D2DIG1, D3DIG1, D4DIG1, D5DIG1, D6DIG1, D7DIG1, D8DIG1, D9DIG1, D10DIG1, D11DIG1, D12DIG1, D13DIG1, OTRDIG1;
	input D0DIG2, D1DIG2, D2DIG2, D3DIG2, D4DIG2, D5DIG2, D6DIG2, D7DIG2, D8DIG2, D9DIG2, D10DIG2, D11DIG2, D12DIG2, D13DIG2, OTRDIG2;
	input Address0, Address1, Address2, Address3;

//Dummy values for the i2c ports, to keep them inactive.  Probably they won't be used on the digitizer board.
parameter SDAenable=1'b1;
parameter SCLenable=1'b1;	
parameter SDAout=1'b1;
parameter SCLout=1'b1;
	
wire [2:0] TREQReg;    //Trigger bits from comparators
wire [14:0] Dig0,Dig1,Dig2;
wire [3:0] Address;

assign Address[0] = Address0;
assign Address[1] = Address1;
assign Address[2] = Address2;
assign Address[3] = Address3;

//100 MHz differential clock input, from the event builder
clk_wiz_v3_6 DCM_INSTANCE
   (// Clock in ports
    .CLK_IN1_P(CLKIN_P),    // IN
    .CLK_IN1_N(CLKIN_N),    // IN
    // Clock out ports
    .CLK1_100(CLK),     // OUT
    .CLK2_100_DLY(CLK_DLY_dum),     // OUT
	 .CLK3DIGI(CLKDIG_int),  // OUT
    // Status and control signals
    .RESET(1'b0),// IN
    .LOCKED(LOCKED)); 
	 
//CLK 100 MHz single-ended clock
//CLK_DLY 100 MHz clock with phase delay, to capture data from the event builder
assign CLK_DLY = CLK;   //Eliminate one clock net for improved timing performance, since both were the same phase anyway

ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_INST0 (.Q(CLKOUT0), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(ClkGate), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT0_BUFFER (.O(CLK0P), .OB(CLK0N), .I(CLKOUT0));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_INST1 (.Q(CLKOUT1), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(ClkGate), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT1_BUFFER (.O(CLK1P), .OB(CLK1N), .I(CLKOUT1));
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_INST2 (.Q(CLKOUT2), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(ClkGate), .D0(1'b1), .D1(1'b0), .R(1'b0), .S(1'b0));
OBUFDS CLKOUT2_BUFFER (.O(CLK2P), .OB(CLK2N), .I(CLKOUT2));

//Select the clock source
IBUFGDS DIGICLK_BUFFER (.I(CLKDIGI_P), .IB(CLKDIGI_N), .O(CLKDIG_ext)); 
BUFGMUX DIGICLK_MUX(.I0(CLKDIG_int), .I1(CLKDIG_ext), .O(CLKDIGI), .S(CLKSEL));

//If the clock source is internal, then assume that the external clock line is used for a hard reset signal
reg RSTinReg;
always @ (CLKSEL or CLKDIG_ext) begin
	if (CLKSEL) begin
		RSTinReg = 1'b0;
	end else begin
		RSTinReg = CLKDIG_ext;
	end
end

// 3-state enable input, high=input, low=output.  These are no longer used, but the pins must be held high impedance!
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SDA (.O(SDAin), .IO(SDApin), .I(SDAout), .T(SDAenable));
IOBUF #(.DRIVE(12), .IOSTANDARD("DEFAULT"), .SLEW("SLOW") ) IOBUF_SCL (.O(SCLin), .IO(SCLpin), .I(SCLout), .T(SCLenable));
//assign SDAenable = ~SDAen;
//assign SCLenable = ~SCLen;

IBUFDS TRGIN_BUFFER (.I(TrgIn_P), .IB(TrgIn_N), .O(TrgIn));
IDDR2 TRGIN_Reg (.D(TrgIn), .Q0(TrgInReg), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

IBUFDS CMDIN_BUFFER (.I(CmdIn_P), .IB(CmdIn_N), .O(CmdIn));
IDDR2 CMDIN_Reg (.D(CmdIn), .Q0(CmdInReg), .C0(CLK_DLY), .C1(~CLK_DLY), .CE(1'b1), .R(1'b0), .S(1'b0));

OBUFDS TREQOUT_BUFFER (.O(TREQOUT_P), .OB(TREQOUT_N), .I(TREQOUTreg));  
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_TREQ (.Q(TREQOUTreg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(TReqOut), .D1(TReqOut), .R(1'b0), .S(1'b0));

OBUFDS Dout_BUFFER (.O(Dout_P), .OB(Dout_N), .I(DoutReg));  
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_Dout (.Q(DoutReg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Dout), .D1(Dout), .R(1'b0), .S(1'b0));

OBUFDS Dout2_BUFFER (.O(DATA2_P), .OB(DATA2_N), .I(Dout2Reg));  
ODDR2 #(.DDR_ALIGNMENT("NONE"), .INIT(1'b0), .SRTYPE("SYNC")) 
		ODDR2_Dout2 (.Q(Dout2Reg), .C0(CLK), .C1(~CLK), .CE(1'b1), .D0(Dout2), .D1(Dout2), .R(1'b0), .S(1'b0));
		
IBUFDS TREQ0_BUFFER (.I(TREQ0P), .IB(TREQ0N), .O(TREQ0));
IDDR2 TREQ0_Reg (.D(TREQ0), .Q0(TREQReg[0]), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));
IBUFDS TREQ1_BUFFER (.I(TREQ1P), .IB(TREQ1N), .O(TREQ1));
IDDR2 TREQ1_Reg (.D(TREQ1), .Q0(TREQReg[1]), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));
IBUFDS TREQ2_BUFFER (.I(TREQ2P), .IB(TREQ2N), .O(TREQ2));
IDDR2 TREQ2_Reg (.D(TREQ2), .Q0(TREQReg[2]), .C0(CLK), .C1(~CLK), .CE(1'b1), .R(1'b0), .S(1'b0));

//Register all the incoming samples, using the 65 MHz delayed clock
IDDR2 D0Dig0_Reg (.D(D0DIG0), .Q0(Dig0[0]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D1Dig0_Reg (.D(D1DIG0), .Q0(Dig0[1]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D2Dig0_Reg (.D(D2DIG0), .Q0(Dig0[2]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D3Dig0_Reg (.D(D3DIG0), .Q0(Dig0[3]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D4Dig0_Reg (.D(D4DIG0), .Q0(Dig0[4]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D5Dig0_Reg (.D(D5DIG0), .Q0(Dig0[5]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D6Dig0_Reg (.D(D6DIG0), .Q0(Dig0[6]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D7Dig0_Reg (.D(D7DIG0), .Q0(Dig0[7]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D8Dig0_Reg (.D(D8DIG0), .Q0(Dig0[8]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D9Dig0_Reg (.D(D9DIG0), .Q0(Dig0[9]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D10Dig0_Reg (.D(D10DIG0), .Q0(Dig0[10]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D11Dig0_Reg (.D(D11DIG0), .Q0(Dig0[11]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D12Dig0_Reg (.D(D12DIG0), .Q0(Dig0[12]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D13Dig0_Reg (.D(D13DIG0), .Q0(Dig0[13]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 OTRDig0_Reg (.D(OTRDIG0), .Q0(Dig0[14]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));

IDDR2 D0Dig1_Reg (.D(D0DIG1), .Q0(Dig1[0]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D1Dig1_Reg (.D(D1DIG1), .Q0(Dig1[1]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D2Dig1_Reg (.D(D2DIG1), .Q0(Dig1[2]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D3Dig1_Reg (.D(D3DIG1), .Q0(Dig1[3]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D4Dig1_Reg (.D(D4DIG1), .Q0(Dig1[4]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D5Dig1_Reg (.D(D5DIG1), .Q0(Dig1[5]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D6Dig1_Reg (.D(D6DIG1), .Q0(Dig1[6]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D7Dig1_Reg (.D(D7DIG1), .Q0(Dig1[7]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D8Dig1_Reg (.D(D8DIG1), .Q0(Dig1[8]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D9Dig1_Reg (.D(D9DIG1), .Q0(Dig1[9]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D10Dig1_Reg (.D(D10DIG1), .Q0(Dig1[10]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D11Dig1_Reg (.D(D11DIG1), .Q0(Dig1[11]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D12Dig1_Reg (.D(D12DIG1), .Q0(Dig1[12]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D13Dig1_Reg (.D(D13DIG1), .Q0(Dig1[13]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 OTRDig1_Reg (.D(OTRDIG1), .Q0(Dig1[14]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));

IDDR2 D0Dig2_Reg (.D(D0DIG2), .Q0(Dig2[0]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D1Dig2_Reg (.D(D1DIG2), .Q0(Dig2[1]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D2Dig2_Reg (.D(D2DIG2), .Q0(Dig2[2]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D3Dig2_Reg (.D(D3DIG2), .Q0(Dig2[3]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D4Dig2_Reg (.D(D4DIG2), .Q0(Dig2[4]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D5Dig2_Reg (.D(D5DIG2), .Q0(Dig2[5]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D6Dig2_Reg (.D(D6DIG2), .Q0(Dig2[6]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D7Dig2_Reg (.D(D7DIG2), .Q0(Dig2[7]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D8Dig2_Reg (.D(D8DIG2), .Q0(Dig2[8]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D9Dig2_Reg (.D(D9DIG2), .Q0(Dig2[9]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D10Dig2_Reg (.D(D10DIG2), .Q0(Dig2[10]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D11Dig2_Reg (.D(D11DIG2), .Q0(Dig2[11]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D12Dig2_Reg (.D(D12DIG2), .Q0(Dig2[12]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 D13Dig2_Reg (.D(D13DIG2), .Q0(Dig2[13]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));
IDDR2 OTRDig2_Reg (.D(OTRDIG2), .Q0(Dig2[14]), .C0(CLKDIGI), .C1(~CLKDIGI), .CE(1'b1), .R(1'b0), .S(1'b0));

EnergyDAQ EnergyDAQ_U(SDAin,SCLin,CLKSEL,ClkGate,Dout,Dout2,TReqOut,CLK,CLKDIGI,RSTinReg,CmdInReg,TrgInReg,TREQReg,Dig0,Dig1,Dig2,Address);

endmodule
