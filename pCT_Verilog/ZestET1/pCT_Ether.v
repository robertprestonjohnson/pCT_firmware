/*
   pCT Ethernet data transfer from the ML605 event builder to the PC
	R. Johnson   July 25, 2013

   ZestET1 Example 6
   File name: Example6.v
   Version: 1.00
   Date: 27/5/2011

   ZestET1 Example 6 - TCP test code using wrapper IP.
   Copies data to and from the Ethernet interface

   Copyright (C) 2011 Orange Tree Technologies Ltd. All rights reserved.
   Orange Tree Technologies grants the purchaser of a ZestET1 the right to use and 
   modify this logic core in any form such as Verilog source code or EDIF netlist in 
   FPGA designs that target the ZestET1.
   Orange Tree Technologies prohibits the use of this logic core in any form such 
   as Verilog source code or EDIF netlist in FPGA designs that target any other
   hardware unless the purchaser of the ZestET1 has purchased the appropriate 
   licence from Orange Tree Technologies. Contact Orange Tree Technologies if you 
   want to purchase such a licence.

  *****************************************************************************************
  **
  **  Disclaimer: LIMITED WARRANTY AND DISCLAIMER. These designs are
  **              provided to you "as is". Orange Tree Technologies and its licensors 
  **              make and you receive no warranties or conditions, express, implied, 
  **              statutory or otherwise, and Orange Tree Technologies specifically 
  **              disclaims any implied warranties of merchantability, non-infringement,
  **              or fitness for a particular purpose. Orange Tree Technologies does not
  **              warrant that the functions contained in these designs will meet your 
  **              requirements, or that the operation of these designs will be 
  **              uninterrupted or error free, or that defects in the Designs will be 
  **              corrected. Furthermore, Orange Tree Technologies does not warrant or 
  **              make any representations regarding use or the results of the use of the 
  **              designs in terms of correctness, accuracy, reliability, or otherwise.                                               
  **
  **              LIMITATION OF LIABILITY. In no event will Orange Tree Technologies 
  **              or its licensors be liable for any loss of data, lost profits, cost or 
  **              procurement of substitute goods or services, or for any special, 
  **              incidental, consequential, or indirect damages arising from the use or 
  **              operation of the designs or accompanying documentation, however caused 
  **              and on any theory of liability. This limitation will apply even if 
  **              Orange Tree Technologies has been advised of the possibility of such 
  **              damage. This limitation shall apply notwithstanding the failure of the 
  **              essential purpose of any limited remedies herein.
  **
  *****************************************************************************************
*/

`timescale 1ns / 1ps

module pCT_Ether
(
    // Programmable Clock
    input CLK,
    
    // Flash Interface
    output Flash_CSOn,
    output Flash_CLK,
    output Flash_MOSI,
    input Flash_MISO,

    // IO connector
    input IO0_In,
    inout [39:0] IO0,
    input [2:0] IO3_In,
    inout [35:0] IO3,

    // Ethernet Interface
    output Eth_Clk,
    output Eth_CSn,
    output Eth_WEn,
    output [4:0] Eth_A,
    inout [15:0] Eth_D,
    output [1:0] Eth_BE,
    input Eth_Intn,

    // DDR SDRAM interface
    output DS_CLK_P,
    output DS_CLK_N,
    output DS_CKE,
    output [12:0] DS_A,
    output [1:0] DS_BA,
    output DS_CAS_N,
    output DS_RAS_N,
    output DS_WE_N,
    inout [1:0] DS_DQS,
    output [1:0] DS_DM,
    inout [15:0] DS_DQ
);

    //////////////////////////////////////////////////////////////////////////
    // Declare constants
    
    // Test transfer length
    localparam WRITE_LENGTH = 500*1024*1024;
    
    //////////////////////////////////////////////////////////////////////////
    // Declare signals
    
    // Clocks and reset
    wire RST;
    wire RAMRST;
    wire RAMClk;
    wire RAMClk90;
    wire EthClk;

    // Link status variables
    wire [31:0] LocalIP;
    wire LinkStatus;
    wire LinkDuplex;
    wire [2:0] LinkSpeed;
    wire [31:0] RemoteIPAddr;
    wire [15:0] RemotePort;
    
    // Connection state signals
    reg EnableConnection;
    wire Connected;
    reg LastConnected;

    // Test data signals
    wire [7:0] SendData;
    reg [31:0] WriteCount;
    wire TxEnable;
    wire TxBusy;
    wire TxBufferEmpty;

    wire RxEnable;
    wire [7:0] RxData;
    wire RxBusy;

    //////////////////////////////////////////////////////////////////////////
    // Tie unused signals
    assign Flash_CSOn = 1;
    assign Flash_CLK = 1;
    assign Flash_MOSI = 1;
    assign IO0 = 40'hZ;
    assign IO3 = 36'hZ;

    //////////////////////////////////////////////////////////////////////////
    // Main code
       
	 wire ClkDatIn,WriteEnableIn;
	 wire FullFIFO,FIFOempty,HalfFullFIFO;
	 reg ReadEnableFIFO;
	 
	 //100 MHz clock from the ML605
	 IBUFGDS  CLKDigBuf (.I(IO0[16]), .IB(IO0[17]), .O(ClkDatIn));
	 
	 //Data input strobe from the ML605
	 IBUFDS  EnableBuf (.I(IO0[2]), .IB(IO0[3]), .O(WriteEnableIn));
	 
	 //Reset signal from the ML605
	 IBUFDS  ResetBuf (.I(IO0[4]), .IB(IO0[5]), .O(ResetIn));
	 
	 //Signal back to the ML605 to hold the trigger
	 OBUFDS  FullBuf (.I(HalfFullFIFO), .O(IO0[0]), .OB(IO0[1]));
	 
	 //16-bit wide data word from the ML605 board
	 wire [15:0] BytesIn;
	 IBUFDS  DatBuf0 (.I(IO0[6]), .IB(IO0[7]), .O(BytesIn[0]));
	 IBUFDS  DatBuf1 (.I(IO0[8]), .IB(IO0[9]), .O(BytesIn[1]));
	 IBUFDS  DatBuf2 (.I(IO0[10]), .IB(IO0[11]), .O(BytesIn[2]));
	 IBUFDS  DatBuf3 (.I(IO0[12]), .IB(IO0[13]), .O(BytesIn[3]));
	 IBUFDS  DatBuf4 (.I(IO0[14]), .IB(IO0[15]), .O(BytesIn[4]));
	 IBUFDS  DatBuf5 (.I(IO0[18]), .IB(IO0[19]), .O(BytesIn[5]));
	 IBUFDS  DatBuf6 (.I(IO0[20]), .IB(IO0[21]), .O(BytesIn[6]));
	 IBUFDS  DatBuf7 (.I(IO0[22]), .IB(IO0[23]), .O(BytesIn[7]));
	 IBUFDS  DatBuf8 (.I(IO0[24]), .IB(IO0[25]), .O(BytesIn[8]));
	 IBUFDS  DatBuf9 (.I(IO0[26]), .IB(IO0[27]), .O(BytesIn[9]));
	 IBUFDS  DatBufA (.I(IO0[28]), .IB(IO0[29]), .O(BytesIn[10]));
	 IBUFDS  DatBufB (.I(IO0[30]), .IB(IO0[31]), .O(BytesIn[11]));
	 IBUFDS  DatBufC (.I(IO0[32]), .IB(IO0[33]), .O(BytesIn[12]));
	 IBUFDS  DatBufD (.I(IO0[34]), .IB(IO0[35]), .O(BytesIn[13]));
	 IBUFDS  DatBufE (.I(IO0[36]), .IB(IO0[37]), .O(BytesIn[14]));
	 IBUFDS  DatBufF (.I(IO0[38]), .IB(IO0[39]), .O(BytesIn[15]));
 
    //Xilinx core for a FIFO 16-wide in, 8-wide out
	 fifo_generator_v9_3 FIFOin(.rst(ResetIn),
								  .wr_clk(ClkDatIn),
								  .rd_clk(EthClk),
								  .din(BytesIn),
								  .wr_en(WriteEnableIn),
								  .rd_en(TxEnable),
								  .dout(SendData),
								  .full(FullFIFO),
								  .empty(FIFOempty),
								  .prog_full(HalfFullFIFO)
								);

    // Connection management
    // Open a connection whenever possible
    // If the host closes the connection then we immediately re-open it
    // NB: If the connection isn't closed (e.g. host application crashes)
    // then we would need to close it from this end before we could re-open it
	 // Question: is the asynchronous reset really needed?
    always @ (posedge ResetIn or posedge EthClk) begin
        if (ResetIn) begin
            EnableConnection <= 0;
            LastConnected <= 0;
        end else begin
            LastConnected <= Connected;
            if (!EnableConnection) begin
                // Rising edge will start listening on connection
                EnableConnection <= 1;
            end else begin
                if (LastConnected & !Connected) begin
                    // Falling edge indicates connection was closed
                    // We need to force a rising edge on EnableConnection
                    // to start listening again
                    EnableConnection <= 0;
                end
            end
        end
    end
	 
    always @ (posedge ResetIn or posedge EthClk) begin
        if (ResetIn) begin
            WriteCount <= 0;
        end else begin
            if (!Connected) begin
                WriteCount <= 0;
            end else if (TxEnable) begin
                WriteCount <= WriteCount+1;
            end
        end
    end
    assign TxEnable = Connected & !FIFOempty & !TxBusy;
    
    // NB: Data from the interface appears on these Rx signals:
    // if (RxEnable==1) begin
    //     ??? <= RxData;
    // end
    assign RxBusy = 0;

    //////////////////////////////////////////////////////////////////////////
    // Instantiate clocks
    // (Assumes default 125MHz reference clock)
    ZestET1_Clocks ClockInst (
        .RST(RST),
        .RefClk(CLK),
        .EthClk(EthClk),
        .RAMRST(RAMRST),
        .RAMClk(RAMClk),
        .RAMClk90(RAMClk90)
    );

    //////////////////////////////////////////////////////////////////////////
    // Ethernet interface wrapper
    ZestET1_EthernetWrapper #(.CLOCK_RATE(125000000))
        EthernetInst (
            .User_CLK(EthClk),
            .User_RST(ResetIn),
            .User_LocalIPAddr(LocalIP),
            .User_LinkStatus(LinkStatus),
            .User_LinkDuplex(LinkDuplex),
            .User_LinkSpeed(LinkSpeed),

            // User interface - only one connection used
            // Control ports
            .User_Enable0(EnableConnection),
            .User_Server0(1'b1),
            .User_TCP0(1'b1),
            .User_TxPause0(1'b0),
            //.User_TxPaused0(),
            .User_Connected0(Connected),
            .User_MTU0(8'h80),
            .User_TTL0(8'h80),
            .User_LocalPort0(16'h5002),
            .User_WriteRemotePort0(16'h0000),
            .User_WriteRemoteIPAddr0(32'h00000000),
            .User_ReadRemotePort0(RemotePort),
            .User_ReadRemoteIPAddr0(RemoteIPAddr),
            
            // Transmit and receive ports
            .User_TxData0(SendData),
            .User_TxEnable0(TxEnable),
            .User_TxBusy0(TxBusy),
            .User_TxBufferEmpty0(TxBufferEmpty),
            .User_RxData0(RxData),
            .User_RxEnable0(RxEnable),
            .User_RxBusy0(RxBusy),
            
            // Disable all other connections
            .User_Enable1(1'b0),
            .User_Enable2(1'b0),
            .User_Enable3(1'b0),
            .User_Enable4(1'b0),
            .User_Enable5(1'b0),
            .User_Enable6(1'b0),
            .User_Enable7(1'b0),
            .User_Enable8(1'b0),
            .User_Enable9(1'b0),
            .User_Enable10(1'b0),
            .User_Enable11(1'b0),
            .User_Enable12(1'b0),
            .User_Enable13(1'b0),
            .User_Enable14(1'b0),
            .User_Enable15(1'b0),
            
            // Interface to GigExpedite
            .Eth_Clk(Eth_Clk),
            .Eth_CSn(Eth_CSn),
            .Eth_WEn(Eth_WEn),
            .Eth_A(Eth_A),
            .Eth_D(Eth_D),
            .Eth_BE(Eth_BE),
            .Eth_Intn(Eth_Intn)
        );

    //////////////////////////////////////////////////////////////////////////
    // SDRAM Buffer
    // Place holder - not used in the example but ties the pins correctly
    ZestET1_SDRAM  #(.CLOCK_RATE(166666666))
        SDRAMInst (
            .User_CLK(RAMClk),
            .User_CLK90(RAMClk90),
            .User_RST(RAMRST),

            .User_A(0),
            .User_RE(0),
            .User_WE(0),
            .User_Owner(8'h00),
            .User_BE(0),
            .User_DW(0),
            //.User_DR(),
            //.User_DR_Valid(),
            //.User_ValidOwner(),
            //.User_Busy(),
            //.User_InitDone(),

            .DS_CLK_P(DS_CLK_P),
            .DS_CLK_N(DS_CLK_N),
            .DS_A(DS_A),
            .DS_BA(DS_BA),
            .DS_DQ(DS_DQ),
            .DS_DQS(DS_DQS),
            .DS_DM(DS_DM),
            .DS_CAS_N(DS_CAS_N),
            .DS_RAS_N(DS_RAS_N),
            .DS_CKE(DS_CKE),
            .DS_WE_N(DS_WE_N)
        );
    
endmodule
