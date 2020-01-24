//DAQ program to run on the ML605 Virtex-6 FPGA and build events from data in serial streams from
//12 Tracker FPGAs and 2 Energy Detector FPGAs.  
//
// R. Johnson, June 28, 2013
// July 11, 2013:  changed RAM storage control from toggle to set/reset.  Added the Ethernet interface controls.
// July 13, 2013:  added the version number, set to 10.
// July 24, 2013:  made the Ethernet output into a 16-bit, instead of 12-bit, word.  V17.
// July 28, 2013:  fixed ethernet output and changed end-of-run handling.  V18
// July 29, 2013:  fixed control of the RAM buffer.  V19
// July 30, 2013:  under SVN revision control on DAQ computer.  V20
// August 1, 2013: fixed trigger mask loading and added readback; put trigger delay in.  V23
// August 2, 2013: new trigger for tracker/cal coincidence.  V24
// August 7, 2013: finally got i2c working on all DVI cables.  V27
// August 8, 2013: added some error counters
// August 9, 2013: add random trigger command for occupancy scans.  V28
// August 16, 2013: fixed parity calculations for outgoing commands.  V30
// August 17, 2013: add parity bit to trigger stream.  V31
// August 18, 2013: added ethernet test command.  V32
// August 27, 2013: simplified what happens when there is an ASIC error detected and modifed reset scheme.  V33
// September 5, 2013: added adjustable input delays for signals from front-end boards.  V34
// September 12, 2013: added dump of trigger counts.  V35
// September 19, 2013: added monitoring of run time and the times of hold errors.  V36
// September 23, 2013: added monitoring of live time.  V37
// September 28, 2013: Increase range of busy counters to 32 bits. V38
// October 3, 2013: add counter for triggers skipped due to Ethernet hold.  V39
// October 16, 2013: added option to receive clear buffer signals from the tracker boards.  V40
// November 7, 2013: send buffer management type back to the top level program.  V41
// November 13, 2013: implement a count of the buffer clear signals, for debugging.  V42
// December 5, 2013: fixed the random trigger command, now 8'h16, which had gotten mixed up with Ethernet test command.  V43
// January 13, 2014: removed some delay between triggers; added run header.  V44
// February 26, 2014: use one line to each digi board for clk or reset, to free up a second data line.  V45
// February 3, 2014: make use of the two data lines from each energy detector board.  V46
// March 16, 2014: change range of local buffers to monitor.  V47
// April 3, 2014: add feature to test data links.  V48
// April 16, 2014: count and output number of data words. V49
// April 18, 2014: hold trigger when the Ethernet is busy.  V50
// April 19, 2014: remove hold on trigger from Ethernet, as bug in TkrDataMerge is fixed.  V51
// April 25, 2014: attempt to improve timing of error calculation. V52
// May 22, 2014: add facility to write the stage angle to the run header. V53
// July 9, 2014: fix DAQ reset logic.  V54
// July 10, 2014: hold the trigger if the ethernet is backed up, instead of letting the buffers fill.  V55
// August 18, 2014: change time tag to be the trigger time, not the time that the event gets sent out.  V56
// September 18, 2014: connect up the external trigger, and use the E-det delay and stretch for it.  V57
// October 9, 2014: add global trigger delay.  V58
// November 3, 2014: reduced the range of the global trigger delay, as it was causing unrelated errors in the i2c circuits.  V59
// November 12, 2014: incorporate special trigger mode for operation in conjunction with the Timepix detector.  V60
// November 19, 2014: fix i2c again.  V61
// February 5, 2015: add option to synch the end of run with the end of spill.  V62
// February 11, 2015: try to fix the i2c yet again. V63
// April 14, 2016: receive detailed trigger information from the energy boards and write it out V64
// May 10, 2017: Increase the FIFO depth for storing the time stamps. V65

//Command format, 1 byte to 12 bytes sent sequentially by a UART on the PC:
//  8-bit command code
//  8-bits   7-4: Target FPGA address, or F for all FPGAs (for the i2c commands it is the board address, instead),   3-0:  number of data bytes
//  Up to 10 data bytes
//  For ASIC commands 10 bytes give an 80 bit data string (75 MSBs are a command string to be sent to the ASICs, including the start bit 
//  for the ASIC.  That last 5 LSBs are padded with zeros.)
//
//FPGA and [Board] Addresses.  The board addresses are used for i2c communication.  Otherwise the command communication is with the FPGA. The data lines are
//	numbered by the FPGA address, but the TReq lines and i2c lines are numbered by the Board address = DVI cable number minus 1
//	Note that the tracker T boards each have two FPGAs, which must differ in address by only the LSB.
//
//                                         		FPGA       [FastOR]   DVI Cable
// First tracker layer (V): 					h0			[h0]			1  
// Second tracker layer (T): 					h4 & h5		[h1]			2  
// Third tracker layer (V):						h1			[h2]			3  
// Fourth tracker layer (T):					h6 & h7		[h3]			4  
// Fifth tracker layer (T):						h8 & h9		[h4]			5
// Sixth tracker layer (V):						h2			[h5]			6
// Seventh tracker layer (T):					hA & hB		[h6]			7
// Eighth tracker layer (V):					h3			[h7]			8
// First energy digitizer (3 channels):			hC			[h8]			9   
// Second energy digitizer (2 channels):		hD			[h9]			10  
//
//Command definitions (8-bit commands to this FPGA program).
//All serial communication is done MSB-first.
// 8'h00: Null.  
// 8'h01: Send a command to the pCTFE64 chips of board address x.  The data field is the complete command string for the chips,
//        including the start bit.  Therefore, it can be up to 75 bits in length, padded at the LSB end with
//        five zeros.  The start bit must be included as the MSB (bit 79) or the command will be ignored.
//		  Calibration strobes will produce an extra entry into the RAM that can then be dumped with the 8'h06
//		  command.  There will be a 12-bit header 100001010010 followed by a 12-bit counter of the time
//		  until the TREQ is seen, followed by the 12-bit counter of the TOT of the TREQ.  The latter is
//		  zero if the TREQ never arrives.
// 8'h02: Start a run.  During the run, all commands are disabled except for end-run. 
//        Note that this means that the READ command will also be disabled during a run.  
//		  Instead, the READ commands are sent to the ASICs automatically for each trigger.  The READ command
//        should be issued manually only in the case of a calibration strobe event (or perhaps if one is 
//        trying to measure noise occupancy directly).  First three bytes of the data field is the run number; the
//        next 4 bytes give the run start time.
// 8'h03: End a run.
// 8'h04: Issue a trigger acknowledge to the chips, where data byte gives the trigger tag (4'h0, 4'h1, 4'h2, or 4'h3)
// 8'h05: Set the mask indicating which front-end boards are connected.  2 data bytes give the mask in the 14 LSB.
// 8'h06: Dump the data buffer back across the UART.  The first two bytes will be the length of the buffer being
//        dumped.  
// 8'h07: Send a non-ASIC command to one or more tracker boards.  One data byte gives the command in the 3 LSBs, or
//        for an extended command send 2 data bytes.
// 8'h08: Set the output data path:  1= RAM buffer, 2= Ethernet, 3= Both;  default is Ethernet only
// 8'h09: Set up the internal trigger, 3 data bytes:
//			  First Byte:
//				 0:  AND logic; set trigger mask bit to "1" to force a layer ON
//				 1:  OR logic, except that the two layers within a VT pair form an AND;  set trigger mask bit to "0" to force layer OFF
//			  Second Byte:    8-bit trigger mask for the 8 Tracker layers; careful: do not enable T-board triggers if using buffer clear signals
//			  Third Byte:		2-bit trigger mask for the 2 energy detector layers
// 8'h0A: Set on/off filling of the RAM storage for the UART interface.  LSB of datafield: 1=on, 0=off.
// 8'h0B: Send a hard reset pulse to the FPGAs on the test boards.  This will only reset the command decoders and configuration.
// 8'h0C: Write a test string into the RAM buffer, including the version number
// 8'h0D: Send a command to the energy digitizer board with address specified in the command.  The two data bytes are the command.
// 8'h0E: Reset the RAM buffer
// 8'h0F: Set the trigger configuration, 1 data byte:
//          x= 0000:  No trigger enabled (necessary for calibration runs)
//			x= 0001:  Internal trigger
//          x= 0010:  External trigger
// 8'h10: Dump local DAQ counters for triggers and events
// 8'h11: Abort on first error if DataField = 1
// 8'h12: Send a test sequence to the Ethernet
// 8'h13: Increment (data=1) or decrement (data=0) the input delay by 1 tap
// 8'h14: Reset the input delay to the default tap value
// 8'h15: Read back the layer mask
// 8'h16: Generate an internal "random" trigger
// 8'h17: Select whether to output time tags in the event header
// 8'h18: Choose between hard reset versus external clock for one of the digi board DVI pairs (0=Reset, 1=Ext Clk)
// 8'h19: Read the internal trigger setup
// 8'h20: Local reset of this code and its subroutines; does not reset the configuration
// 8'h21: Reset the Ethernet interface (Zest-ET1) FPGA code
// 8'h22: Load the tracker trigger timing, 2 data bytes.  Upper byte=delay, lower byte=length
// 8'h23: Read the tracker trigger timing
// 8'h24: Load the energy detector trigger timing, 2 data byte.  Upper byte=delay, lower byte=length
// 8'h25: Read the energy detector trigger timing
// 8'h26: Set the buffer management mode: 0=simple, 1=use buffer clear signals from the tracker boards
// 8'h27: Run a data transmission test on the link from the target FPGA
// 8'h28: Dump out the results of the most recent data transmission test (which should already be completed)
// 8'h29: Set the two-byte stage angle to write into the run header
// 8'h30: Reset the configuration
// 8'h31: Set the global clock delay to the last 3 bits of the data byte
// 8'h32: Set TimePix mode:  0= pCT only; 1= pCT and Timepix
// 8'h33: Set 16 bit TimePix frame length in units of 40 ns (for 100 MHz clock)
// 8'h34: Synchronize end of run with end of spill (0=don't; 1=do)
// 8'h35: Set to 1 to output trigger bits from the energy detector
// 8'h40: Dump debug counters and error flags
// 8'h50: Dump buffer occupancies for FE boards 
// 8'h51: Dump buffer occupances for ASICs
// 8'h60: Dump trigger diagnostic counters
// 8'h61: Dump trigger counts, energy and tracker
// 8'h62: Dump trigger counts, external and output
// 8'h63: Dump times of the first and last hold errors
// 8'h64: Dump of busy and hold triggers
// 8'h66: Dump buffer manager errors
// 8'h80 and 81: Dump local buffer occupancies
// 8'h90: Dump number of send and read commands that went out
// 8'h91: Dump number of triggers going out and total run time
// 8'h92: Dump trigger live time
// 8'h93: Dump the trigger-vs-time information (2 bytes--># words, followed by that many 24-bit words)
// 8'h95: Dump TimePix external trigger counts
// 8'h96: Dump some configuration constants
// 8'h97: Dump more configuration constants
// 8'hE0: Send an I2C read command to the specified board.  The data field is one byte, with bits 6:0 being the i2c address.
//        The board address (0 to 9) sets the multiplexer, which must not change until the (rather slow) i2c operation is completed.  
//        So i2c commands must not be extremely close together in time.
// 8'hF0: Send an I2C load command to the specified board.  The data field is the set of 3 or 4 bytes to go to I2C, with bits 6:0 of the first byte 
//		  being the i2c address. The board address (0 to 9) sets the multiplexer, which must not change until the (rather slow) i2c operation is completed.  
//        So i2c commands must not be extremely close together in time.
// All other codes:  Null

module FPGA_EvtBuild(Diagnostic1,Diagnostic2,TrgInhibit,ClkVsRst,BufMngrType,ResetDly,IncDir,EnableInc,RunInProg,EthBusy,EthEn,EthDat,EthRst,TxD_start,TxD_data,TrgOut,CMD,CmdE,RSTfpga,TxD_busy,RxD_data_ready,RxD_data,TrgEx,Data,Clock,FastOR,ResetExt,SDAin,SDAout,SDAen,SCL,SCLen,SDAin2,SCLin2,CLK_SDAin,CLK_SCLin,BufClrStrm);

output Diagnostic1;      	//Selected signal to send out to the scope probe (SMA output on the ML605)
output Diagnostic2;      	//Selected signal to send out to the scope probe (SMA output on the ML605)
output ClkVsRst;			//Select usage of one of the DVI pairs for the energy digitizer boards
output BufMngrType;			//Select whether to use trigger signals or buffer clear signals from T boards
output ResetDly;			//Reset the input delay to the default tap value
output IncDir;				//1=increment delay, 0=decrement delay
output EnableInc;			//Increment or decrement input delay each clock cycle
input EthBusy;				//Signal that the input FIFO of the Zest-ET1 Ethernet FPGA is getting too full to accept more data
output EthEn;				//Signal that a valid 16-bit word is ready for the Zest-ET1 Ethernet FPGA input FIFO
output [15:0] EthDat;		//Sixteen bit word to go to the Zest-ET1 Ethernet FPGA input FIFO
output EthRst;				//Signal to reset the Zest-ET1 Ethernet FPGA code
output TxD_start;			//Signal that a byte of data is ready in TxD_data
output [7:0] TxD_data;  	//Byte of data to send by UART
input TxD_busy;				//Signal from the UART transmitter to wait to go low before sending another byte
input RxD_data_ready;   	//Signal that a byte of data is available in RxD_data from the UART receiver
input [7:0] RxD_data;   	//Byte of incoming data from the UART
input Clock;          		//Input system clock 
input ResetExt;       		//System external reset (button on the FPGA board!)
input [15:0] Data;     		//Serial data streams from the 14 front-end board FPGAs, including 2 streams from each energy detector board
input TrgEx;     	  		//External trigger input
input [9:0] FastOR;	  		//Trigger ORs from the front-end boards.  Enumerated by the Board address.
output TrgOut;        		//Trigger Acknowledge going to the front-end boards 
output CMD;           		//Commands going to the tracker front-end boards 
output CmdE;		  		//Commands going to the energy digitizer boards
output RSTfpga;		  		//Reset going to the front-end board FPGA
output [9:0] SCL, SCLen;	//3-state i2c clock, with enable, with a separate line for each front-end board; enable active low
output [9:0] SDAout, SDAen; //3-state i2c data, with enable; active low
input [9:0] SDAin;	  		//i2c data input, enumerated by the Board address.
output RunInProg;			//True when a run is in progress
input SDAin2,SCLin2,CLK_SDAin,CLK_SCLin;  //Thus far unused i2c signals
input [7:0] BufClrStrm;		//Buffer clear serial signals from the tracker boards
input TrgInhibit;        	//Set HIGH to inhibit the trigger when running in TimePix mode

parameter [7:0] Version = 8'd65;

reg TagOut;
reg IncDir, EnableInc, ResetDly;
reg Ethernet, RAMbuf;		//Select data output paths
reg [15:0] EthDat;
reg EthEn, EthRst;
reg [15:0] MaskLyr;		//Each bit specifies whether the corresponding FPGA is present in the readout
wire [7:0] MaskTkr;		//Each bit specifies whether the corresponding tracker board is present in the readout
wire DMPDone;
reg TxD_start;
reg CmdE;
reg [9:0] SDAout,SCL;
reg [9:0] SDAen,SCLen;
reg [7:0] TxD_data;
reg [7:0] InByte;
reg RunInProg;
reg [15:0] EventData;	//Data streams flowing into the event builder module
reg CMD;
reg RSTfpga;
reg [77:0] OutputString;   //Used for readback of the test string, and also for DAQ counters
reg DataStore;
reg TrgOut;
reg HoldExt, ResetRs, AckRs;
reg [31:0] NTrgHld, NEthHld;
reg [7:0] NResetRs;
wire [11:0] DataOut;		//Data words in the combined event stream
reg [5:0] NeventLst;
reg [3:0] I2Cmux;			//Multiplexer board address for i2c I/O
reg ReadDAC, ReadIna;
wire [15:0] RegDAC, RegIna;
wire [1:0] DACerr;
wire [2:0] InaErr;    //Error codes from the i2c communication
wire [31:0] NRead;
reg [31:0] NSend;
reg [31:0] NTrgOut;
reg TrgRndm, AbortOnError, Abort;

//Encoding of states for the command interpretor state machine:
parameter [23:0] Wait = 24'b000000000000000000000001;  //Wait for a UART command byte
parameter [23:0] GtNB = 24'b000000000000000000000010;  //Get 2nd byte, telling number of bytes and board address
parameter [23:0] GtBs = 24'b000000000000000000000100;  //Get the rest of the Bytes
parameter [23:0] Deco = 24'b000000000000000000001000;  //Decode the command
parameter [23:0] FeCd = 24'b000000000000000000010000;  //Check the ASIC command
parameter [23:0] Ct66 = 24'b000000000000000000100000;  //Shift the ASIC command out
parameter [23:0] Stck = 24'b000000000000000001000000;  //Send a trigger signal to the test board
parameter [23:0] DpWt = 24'b000000000000010010000000;  //Wait for the data dump to finish
parameter [23:0] ShCd = 24'b000000000000000100000000;  //Shift out an 11-bit command to the tracker boards
parameter [23:0] Test = 24'b000000000000001000000000;  //Shift a test string or register contents into the RAM buffer
parameter [23:0] ECmd = 24'b000000000000010000000000;  //Wrap up short and long commands
parameter [23:0] ErgC = 24'b000000000000100000000000;  //Shift out an 18-bit command to the energy detector boards
parameter [23:0] I2Cr = 24'b000000000001000000000000;  //Read from an i2c register
parameter [23:0] I2Cw = 24'b000000000010000000000000;  //Write to an i2c register
parameter [23:0] LgCd = 24'b000000000100000000000000;  //Shift an extended command to the tracker boards
parameter [23:0] EndR = 24'b000000001000000000000000;  //Ending run, wait for buffers to flush out
parameter [23:0] EthT = 24'b000000010000000000000000;  //Send test stream to the Ethernet
parameter [23:0] RsDl = 24'b000000100000000000000000;  //Reset signal for input delays
parameter [23:0] DpRt = 24'b000001000000000000000000;  //Dumping trigger-rate-vs-time information
parameter [23:0] RnHd = 24'b000010000000000000000000;  //Send out the run header to the Ethernet interface
parameter [23:0] Dtst = 24'b000100000000000000000000;  //Run data transmission test
parameter [23:0] Abrt = 24'b001000000000000000000000;  //Abort the run upon detecting an error
parameter [23:0] Sync = 24'b010000000000000000000000;  //Wait for the end of spill
parameter [23:0] EndS = 24'b100000000000000000000000;  //At run end wait for the UART output not to be busy

reg [23:0] State, NextState;
reg [79:0] DataField;
reg [85:0] CmdOut;
reg [9:0] ShrtCmd; 
reg [25:0] LongCmd;
reg [17:0] EnrgCmd;
reg [5:0] CntHdr;
reg [6:0] Cnt;     	//Seven-bit counter
reg [4:0] Cnt4;     //Five-bit counter
wire [3:0] NBufFull;
wire [4:0] NlclFull;
reg DMPStart;
reg [65:0] CounterOut;
reg [3:0] Error;		//Error flags:  bit 0: wrong header from tracker board
						//              bit 1: CRC error in at least one event
reg RAMenable;
reg [27:0] CntTime;    //Time-out counter for the data dump and UART receive
reg RstRam;
reg [3:0] TackOut;
reg [3:0] TrgCnfg;
reg [7:0] TkrTrgMsk;
reg [1:0] CalTrgMsk;
wire [31:0] Nevent;    //Note that a run with much more than 4 billion events will cause problems!
reg [31:0] NWordOut;
reg [31:0] Nbusy, Nhold, NsndHold;
reg [31:0] Nful1, Nful2, Nful3, Nful4;
reg [31:0] Nful5, Nful6, Nful7, Nful8;
reg [15:0] Nbfl1, Nbfl2, Nbfl3, Nbfl4;
reg [15:0] Nlcl1, Nlcl2, Nlcl3, Nlcl4;
reg [15:0] Nlcl13, Nlcl14, Nlcl15, Nlcl16;
reg Reset;   //Local reset pulse activated only by command (Command state machine resets by ResetExt)
reg resetConfig;
reg [3:0] Address, NBytes;
wire [15:0] AddErr;
reg SendStream;
reg PrtyE;
reg StrobeDAC, StrobeIna;
reg [6:0] i2cAddr;
reg PrtyDat;
reg HoldEndR;
reg endRun;
reg BufMngrType;
reg [7:0] TkrTrgLen, TkrTrgDly, CalTrgLen, CalTrgDly;  //Trigger timing control
reg [11:0] RunHeader [0:8];
reg StrobeHdr;
reg [11:0] DataHdr;
reg [35:0] RunTime, LiveTime;   //RunTime is used as a time stamp, as well as to track dead time
reg [35:0] TimeTagOut;          //Trigger time of the event being sent out
reg [35:0] TimeTags[0:15];       //Queue the trigger times
reg [5:0] TrgWdOut;             //Trigger word of the event being sent out
reg [5:0] TrgWds[0:15];          //Queue the trigger words
reg [11:0] DeltaT, DeltaTlast, DeltaTout;   //Clock cycles from the last trigger input
reg [11:0] DeltaTbuf[0:15];					//FIFO for queueing the delta-T values
reg [3:0] Twrite, Tread;					//Read and write pointers for the delta-T FIFO queue

//Mapping between tracker trigger bits and FPGAs
assign MaskTkr[0] = MaskLyr[0];
assign MaskTkr[1] = MaskLyr[4] | MaskLyr[5];
assign MaskTkr[2] = MaskLyr[1];
assign MaskTkr[3] = MaskLyr[6] | MaskLyr[7];
assign MaskTkr[4] = MaskLyr[8] | MaskLyr[9];
assign MaskTkr[5] = MaskLyr[2];
assign MaskTkr[6] = MaskLyr[10] | MaskLyr[11];
assign MaskTkr[7] = MaskLyr[3];

//Some debugging counters:
reg [31:0] FirstHold;
reg [11:0] NCRCerr;
reg [15:0] NTagErr, NHoldErr, NDAQErr;
reg [15:0] NAddressErr;
reg [15:0] BufHoldStat, ReadyStat;
reg [11:0] FEbufStat;
always @ (posedge Clock) begin
	if (Reset | ResetExt) begin
		NCRCerr <= 0;
		Error <= 4'b0000;
		NHoldErr <= 0;
		NDAQErr <= 0;
		NTagErr <= 0;
		FirstHold <= 0;
		NAddressErr <= 0;
		NsndHold <= 0;
		NSend <= 0;
		BufHoldStat <= 0;
		FEbufStat <= 0;
		ReadyStat <= 0;
		Abort <= 1'b0;
	end else begin
		if (RunInProg) begin
			if (SendPls) NSend <= NSend + 1;
			if (SendHold) NsndHold <= NsndHold + 1;
			if (ErrSgnl) begin
				NDAQErr <= NDAQErr + 1;
				if (AbortOnError) Abort <= 1'b1;
				$display("%g\t FPGA_EvtBuild: ASIC DAQ error detected.  NDAQErr=%d",$time,NDAQErr+1);
			end				
			if (TagErr) begin
				NTagErr <= NTagErr + 1;
				if (AbortOnError) Abort <= 1'b1;
				$display("%g\t FPGA_EvtBuild: tag error detected.  NTagErr=%d",$time,NTagErr+1);
			end
			if (HoldError) begin
				NHoldErr <= NHoldErr + 1;   //So far nothing is done when a HoldError arises.  Can we reset the whole DAQ?
				if (NHoldErr == 0 ) begin
					FirstHold <= RunTime[35:4];
					BufHoldStat <= Bufree;
					ReadyStat <= BufRdy;
					FEbufStat <= BufClr;
					if (AbortOnError) Abort <= 1'b1;
				end
				$display("%g\t FPGA_EvtBuild: Hold error detected.  NHoldErr=%d",$time,NHoldErr+1);
			end
			if (AddErr != 0) begin
				NAddressErr <= NAddressErr + 1;
				Error[0] <= 1'b1;
				if (AbortOnError) Abort <= 1'b1;
				$display("%g\t FPGA_EvtBuild: Address error detected.  NAddressErr=%d",$time,NAddressErr+1);
			end
			if (CRCerr) begin 
				Error[1] <= 1'b1; 
				NCRCerr <= NCRCerr + 1; 
				if (AbortOnError) Abort <= 1'b1;
				$display("%g\t FPGA_EvtBuild: CRC error detected.  NCRCerr=%d",$time,NCRCerr+1);
			end
		end else Abort <= 1'b0;
	end
end

//initial begin
//   $display("Time Data CMD TReqIn State RxDdataRdy RxD_data Cnt RnInPrg DMPStrt DMPDone TxDstart TxDbusy TxDdata TrgOut CmdTrig Data DataSt BufSel");
//end

//always @ (posedge Clock) begin
//   $display("%g\t  %b  %b  %b   %h     %b %b %h   %b   %b   %b   %b   %b %b   %b %b %b    %b %h",$time,Data,CMD,TReqIn,State,RxD_data_ready,RxD_data,Cnt,RunInProg,DMPStart,DMPDone,TxD_start,TxD_busy,TxD_data,TrgOut,CmdTrig,Data,DataStore,BufSel);
//end

//Calculate the parity of the FPGA address
assign PrtyAddr = Address[3]^Address[2]^Address[1]^Address[0];

always @ (State or SpillFlag or SpillSynch or ResetRs or Abort or DoneTest or RxD_data or i2cDatRdy or RxD_data_ready or CntTime or InByte or Cnt or Cnt4 or DataField or DMPDone or RunInProg or CounterOut or CntHdr or NBytes or DmpTrgDone or TxD_busy)
begin:CombinatorialLogic
  case(State)
	Wait: 	begin
				if (RxD_data_ready) 
					NextState=GtNB;
				else if (Abort & RunInProg) 
					NextState=Abrt;
				else if (ResetRs & RunInProg)
				    NextState=ShCd;
				else
					NextState=Wait;
			end
	GtNB:	begin
				if (CntTime==28'b1111111111111111111111111111) NextState=Wait;    //Time-out condition
				else if (RxD_data_ready)
					if (RxD_data[3:0]==0) NextState=Deco;
					else NextState=GtBs;
				else
					NextState=GtNB;
			end
	GtBs:	begin
				if (CntTime==28'b1111111111111111111111111111) NextState=Wait;    //Ignore the command if user fails to send all the bytes
				else if (RxD_data_ready)
					if (Cnt[3:0]==NBytes) NextState=Deco;
					else NextState=GtBs;
				else NextState=GtBs;
			end
	Abrt:	  begin
					NextState = Deco;
			  end
	Deco:   begin
				case (InByte)
					8'h01:	begin					//Check the ASIC command string
								if (DataField[79]==1'b1 && (!RunInProg || DataField[73:70]==4'hf || DataField[73:70]==4'h2)) begin
									NextState=Ct66;  
								end else begin 
									NextState=Wait;		//Ignore ASIC commands that lack start bits
								end
							end
					8'h02:	NextState = ShCd;
					8'h03:	begin
								if (SpillSynch & SpillFlag) NextState = Sync;
								else NextState = EndS;
							end
					8'h04:	NextState = Stck;
					8'h06:	begin
								if (RunInProg) NextState=Wait;
								else NextState=DpWt;
							end
					8'h07:	if (NBytes == 1) NextState = ShCd; else if (NBytes == 2) NextState = LgCd; else NextState = Wait;
					8'h0C:	NextState = Test;
					8'h0D:	NextState = ErgC;
					8'h10:	NextState = Test;
					8'h12:	NextState = EthT;
					8'h14:	NextState = RsDl;
					8'h15:	NextState = Test;
					8'h19:	NextState = Test;
					8'h23:	NextState = Test;
					8'h25:	NextState = Test;
					8'h27:  NextState = LgCd;
					8'h28:	NextState = Test;
					8'h40:	NextState = Test;
					8'h50:	NextState = Test;
					8'h51:	NextState = Test;
					8'h52:	NextState = Test;
					8'h53:  NextState = Test;
					8'h54:  NextState = Test;
					8'h55:  NextState = Test;
					8'h60:	NextState = Test;
					8'h61:	NextState = Test;
					8'h62:	NextState = Test;
					8'h63:	NextState = Test;
					8'h64:	NextState = Test;
					8'h65:	NextState = Test;
					8'h66:	NextState = Test;
					8'h67:	NextState = Test;
					8'h80:	NextState = Test;
					8'h81:	NextState = Test;
					8'h82:	NextState = Test;
					8'h90:	NextState = Test;
					8'h91:	NextState = Test;
					8'h92:	NextState = Test;
					8'h93:	NextState = DpRt;
					8'h94:	NextState = Test;
					8'h95:  NextState = Test;
					8'h96:	NextState = Test;
					8'h97:	NextState = Test;
					8'h98:	NextState = Test;
					8'h99:	NextState = Test;
					8'hE0:	if (NBytes==1) NextState = I2Cr; else NextState = Wait;
					8'hF0:	case (NBytes)
								4'h3:	NextState = I2Cw;
								4'h4:	NextState = I2Cw;
								default: NextState = Wait;
							endcase
					default:NextState=Wait;
				endcase
			end
	RnHd:	begin
				if (Cnt == 9) begin
					NextState = Wait;    
				end else begin
					NextState = RnHd;    //Sending the run header to the Ethernet
				end
			end
	I2Cr:	begin
				if (i2cDatRdy || CntTime==28'b1111111111111111111111111111) NextState = Test; 
				else NextState = I2Cr;
			end
	I2Cw:	begin
				if (CntTime == 65536) NextState = Wait;   //Kill time while the i2c load executes
				else NextState = I2Cw;
			end
	Ct66:	begin								//Shift out the bits of the ASIC command
				if (Cnt==83) NextState=Wait;
				else NextState=Ct66;
			end
	Stck:	begin
				if (Cnt4 == 3) 
					NextState = Wait;
				else
					NextState=Stck;
			end
	DpWt:	begin
				if (DMPDone || CntTime==28'b1111111111111111111111111111) NextState=Wait;
				else NextState=DpWt;
			end
	ShCd:	begin
				if (Cnt4==9) NextState = ECmd;
				else NextState = ShCd;
			end
	LgCd:	begin
				if (Cnt4==25) begin
					if (InByte == 8'h27) NextState = Dtst;
					else NextState = ECmd;
				end else NextState = LgCd;
			end
	Dtst:	begin
				if (DoneTest) NextState = ECmd;
				else NextState = Dtst;
			end
	Test:	begin
				if (Cnt==77) NextState = Wait;
				else NextState = Test;
			end
	ECmd:   begin
				if (InByte==8'h02) NextState = RnHd;
				else NextState = Wait;
			end
	ErgC:	begin
				if (Cnt == 17) NextState = Wait;
				else NextState = ErgC;
			end
	Sync:   begin
				if (!SpillFlag) NextState = EndS;
				else NextState = Sync;
			end
	EndS:	begin
				if (TxD_busy) NextState = EndS;
				else NextState = EndR;
			end
	EndR:	begin
				if (CntTime == 2043) NextState = ShCd;
				else NextState = EndR;
			end
	EthT:	begin
				if (CntTime == 28'b1111111111111111111111111111) NextState = Wait;
				else NextState = EthT;
			end
	RsDl:	begin
				if (Cnt == 10) NextState = Wait;
				else NextState = RsDl;
			end
	DpRt:	begin
				if (CntTime == 28'b1111111111111111111111111111) NextState = Wait;
				else begin
					if (DmpTrgDone) NextState = Wait;
					else NextState = DpRt;
				end
			end
	default: begin
				NextState=Wait;
			 end
  endcase
end

reg ClkVsRst;
reg [11:0] StageAngle;
reg [1:0] GlbTrgDelay;
reg TimePix;
reg [15:0] TimePixFrame;    
reg SpillSynch;             //Set to 1 to synch end of run with end of spill
reg TrgBitsOut;             //Set to 1 so as not to test energy detector tags, in the case that they are replaced by trigger bits
always @ (posedge Clock)
begin:SequentialLogic
  if (Reset) begin
	CmdOut <= 0;
	DMPStart <= 1'b0;
	DmpTrgMon <= 1'b0;
	StrobeDAC <= 1'b0;
	StrobeIna <= 1'b0;
	ReadDAC <= 1'b0;
	ReadIna <= 1'b0;
	I2Cmux <= 4'hf;
	HoldEndR <= 1'b0;
	endRun <= 1'b0;
	SendStream <= 1'b0;
	EthRst <= 1'b0;
	Reset <= 1'b0;
	TxD_start2 <= 1'b0;
	TxD_data2 <= 8'b10011001;    //Signal to the DAQ that the run really is ending
	$display("%g\t FPGA_EvtBuild:  reset.",$time);
  end else if (resetConfig) begin
	Reset <= 1'b1;
	TagOut <= 1'b1;
	BufMngrType <= 1'b0;
	resetConfig <= 1'b0;
    RunInProg <= 1'b0;
	RAMenable <= 1'b1;
	TrgCnfg <= 4'b0001;
	TrigType <= 4'h1;
	TkrTrgMsk <= 8'b00000011;
	CalTrgMsk <= 2'b00;
	MaskLyr <= 16'b1111111111111111;
	Ethernet <= 1'b1;
	RAMbuf <= 1'b0;
	TkrTrgLen <= 8'h0a;
	TkrTrgDly <= 8'h00;
	CalTrgLen <= 8'h02;
	CalTrgDly <= 8'h0f;
	ClkVsRst <= 1'b0;
	AbortOnError <= 1'b1;
	GlbTrgDelay <= 0;
	TimePix <= 1'b0;
	TimePixFrame <= 16'd25001;
	SpillSynch <= 1'b1;
	TrgBitsOut <= 1'b1;
	$display("%g\t FPGA_EvtBuild:  resetting configuration.",$time);
  end else if (ResetExt) begin
	resetConfig <= 1'b1;
	State <= Wait;
	$display("%g\t FPGA_EvtBuild:  external reset.",$time);
  end else begin
    State <= NextState;
	if (State == Deco && InByte == 8'h02) begin
		RunTime <= 0;  //Zero the time stamp at the start of run
		LiveTime <= 0;
	end else begin
		RunTime <= RunTime + 1;
		if (!BusyHold) LiveTime <= LiveTime + 1;
	end
	case(State)
		Wait: 	begin								//Wait for a command from the UART
					EnableInc <= 1'b0;
					StrobeHdr <= 1'b0;
					ResetDly <= 1'b0;
					InByte <= RxD_data;
					RSTfpga <= 1'b0;
					CntTime <= 0;
					RstRam <= 1'b0;
					PrtyDat <= 1'b0;
					EthRst <= 1'b0;
					SendStream <= 1'b0;
					TrgRndm <= 1'b0;
					if (ResetRs) begin
						ShrtCmd <= 10'b1111101100;   //Command to reset the ASICs without altering their registers
						AckRs <= 1'b1;
					end else AckRs <= 1'b0;
//					if (RxD_data_ready) $display("%g\t FPGA_EvtBuild:  Command code=%b, %h.",$time,RxD_data,RxD_data);
				end
		GtNB:	begin								//Get the byte containing the FPGA board address and number of bytes
					Address <= RxD_data[7:4];
					NBytes <= RxD_data[3:0];
					Cnt <= 4'b0001;
					CntTime <= CntTime + 1;
//					if (RxD_data_ready) $display("%g\t FPGA_EvtBuild: Command address=%h, Number of Bytes=%d.",$time,RxD_data[7:4],RxD_data[3:0]);
				end
		GtBs:	begin								//Get all of the remaining bytes in the command
					CntTime <= CntTime + 1;
					if (RxD_data_ready) begin
//						$display("%g\t FPGA_EvtBuild: data field Byte=%b, %h.  PrtyDat=%b",$time,RxD_data,RxD_data,PrtyDat);
						PrtyDat <= PrtyDat^(RxD_data[7]^RxD_data[6]^RxD_data[5]^RxD_data[4]^RxD_data[3]^RxD_data[2]^RxD_data[1]^RxD_data[0]);
						Cnt <= Cnt + 1;
						DataField <= {DataField[71:0],RxD_data};
					end
				end
		Abrt: begin
					InByte <= 8'h03;
					NBytes <= 0;
					Address <= 0;
					$display("%g\t Aborting run due to data acquisition error.  Error flags=%b.",$time,Error);
				end
		Deco:	begin								//Decode the command and take the requested action
					Cnt <= 0;
					Cnt4 <= 0;
					CntHdr <= 6'b110001;
					CntTime <= 0;
//					$display("%g\t FPGA_EvtBuild:  Deco, byte received=%b  %h, Address=%h",$time,InByte,InByte,Address);
//					if (NBytes>0) $display("%g\t FPGA_EvtBuild:  data field=%b, %h, PrtyDat=%b",$time,DataField[79:0],DataField[79:0],PrtyDat);
					case (InByte) 
						8'h01: 	begin       //Prepare to send a command to the ASICs
									Cnt <= 0;               
									CmdOut <= {1'b1,Address,1'b1,DataField};    
								end
						8'h02:	begin
									ShrtCmd <= 10'b1111100101;    //Start run command, always sent to all boards
									RunHeader[0] <= 12'b110100100101;
									RunHeader[1] <= 12'b010101001110;
									RunHeader[2] <= DataField[55:44];    //Run number
									RunHeader[3] <= DataField[43:32];
									RunHeader[4] <= DataField[31:20];         //Start time
									RunHeader[5] <= DataField[19:8];
									RunHeader[6] <= {DataField[7:0],4'b0000};
									RunHeader[7] <= {3'b000,TagOut,Version};
									RunHeader[8] <= StageAngle;
								end
						8'h03:	begin
									ShrtCmd <= 10'b1111100110;    					//End run command, always sent to all boards
									$display("%g\t FPGA_EvtBuild: run end command received.",$time);
								end
						8'h04: 	begin
									TackOut <= {1'b1,DataField[1:0],~(DataField[0]^DataField[1])};
								end
						8'h05:	begin
									MaskLyr <= DataField[15:0];
									$display("%g\t FPGA_EvtBuild:  FPGA layer mask set to %b",$time,DataField[15:0]);
								end
						8'h06: 	begin
									if (!RunInProg) begin
										DMPStart <= 1'b1;
									end
									CntTime<=0;									
								end
						8'h07: 	begin
									if (NBytes == 1) begin
										ShrtCmd <= {1'b1,Address,1'b0,DataField[2:0],PrtyAddr^PrtyDat};  //Tracker board command
									end else if (NBytes == 2) begin
//										$display("%g\t FPGA_EvtBuild:  sending long command.  Address=%b, DataField=%b,  PrtyAddr=%b, PrtyDat=%b",$time,Address,DataField[15:0],PrtyAddr,PrtyDat);
										LongCmd <= {1'b1,Address,4'b0001,DataField[15:0],~(PrtyAddr^PrtyDat)};   //Tracker board extended command
									end
								end
						8'h08:	begin
									Ethernet <= DataField[1];
									RAMbuf <= DataField[0];
									$display("%g\t FPGA_EvtBuild:  output path selection = %h",$time,DataField[1:0]);
								end
						8'h09: 	begin
									TrigType <= DataField[19:16];
									TkrTrgMsk <= DataField[15:8];
									CalTrgMsk <= DataField[1:0];
									$display("%g\t FPGA_EvtBuild trigger setup:  TrigType=%b, TkrTrgMsk=%b, CalTrgMsk=%b.",$time,DataField[19:16],DataField[15:8],DataField[1:0]);  
								end
						8'h0A: 	begin
									RAMenable <= DataField[0];
									$display("%g\t FPGA_EvtBuild:  UART RAM enable set to %b.",$time,DataField[0]);
								end 
						8'h0B: 	begin
									RSTfpga <= 1'b1;
									$display("%g\t FPGA_EvtBuild:  reset pulse issued to the front-end boards.",$time);
								end
						8'h0C: 	begin
									OutputString <= {2'b11,Address,Version,64'b1100010111110000011111100000111111000001111110000011111100000111}; 
								end
						8'h0D:	begin
									EnrgCmd <= {1'b1,DataField[15:0],1'b0};
									PrtyE <= 1'b0;
								end
						8'h0E:	begin
									RstRam <= 1'b1;
									$display("%g\t FPGA_EvtBuild:  reset pulse issued to the RAM UART interface.",$time);
								end
						8'h0F: 	begin
									TrgCnfg <= DataField[3:0];
									$display("%g\t FPGA_EvtBuild:  trigger configuration set to %b.",$time,DataField[3:0]);
								end
						8'h10: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <=NTrgGen;
									OutputString[65:34]<= Nevent;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: Ntrig=%d, Nevent=%d",$time,NTrgGen,Nevent);
									$display("%g\t FPGA_EvtBuild: Local buffers full=%d, Enrg buffers full=%d.  ASIC buffers full=%d.",$time,NlclFull,NBufFull,NmostFullFE);
								end
						8'h11:	begin
									AbortOnError <= DataField[0];
								end
						8'h13:	begin
									IncDir <= DataField[0];
									EnableInc <= 1'b1;
								end
						8'h14:	begin
									ResetDly <= 1'b1;
								end
						8'h15: 	begin
									OutputString <= {2'b11,Address,MaskLyr,MaskTkr,48'b111111111111111111111111111111111111111111111111}; 
								end
						8'h16:	begin
									TrgRndm <= 1'b1;
								end
						8'h17:	begin
									TagOut <= DataField[0];
								end
						8'h18:	begin
									ClkVsRst <= DataField[0];
								end
						8'h19:	begin
									OutputString <= {2'b11,Address,4'h0,TrigType,TkrTrgMsk,6'b000000,CalTrgMsk,46'h0,2'b11};
								end
						8'h20: 	begin
									Reset <= 1'b1;
									$display("%g\t FPGA_EvtBuild:  local reset pulse issued.",$time);
								end
						8'h21:  begin
									EthRst <= 1'b1;
									$display("%g\t FPGA_EvtBuild:  sending reset pulse to the Zest-ET1 FPGA.",$time);
								end
						8'h22:	begin
									TkrTrgLen <= DataField[15:8];
									TkrTrgDly <= DataField[7:0];
								end
						8'h23:	begin
									OutputString <= {2'b11,Address,TkrTrgLen,TkrTrgDly,54'h0,2'b11};
								end
						8'h24:	begin
									CalTrgLen <= DataField[15:8];
									CalTrgDly <= DataField[7:0];
								end
						8'h25:	begin
									OutputString <= {2'b11,Address,CalTrgLen,CalTrgDly,54'h0,2'b11};
								end
						8'h26:	begin
									BufMngrType <= DataField[0];
									$display("%g\t FPGA_EvtBuild: buffer management mode set to %b",$time,DataField[0]);
								end
						8'h27:	begin
									LongCmd <= {1'b1,Address,4'b0001,8'h0C,~PrtyAddr};
									TestStart <= 1'b1;
								end
						8'h28:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[25:2] <= Ntests;
									OutputString[49:26]<= badTest;
									OutputString[65:50]<= Nerr;
									OutputString[77:66]<= {2'b11,Address,6'b110001};	
									$display("%g\t FPGA_EvtBuild: number of errors in %d data transmission test=%d",$time,Ntests,Nerr);															
								end
						8'h29:	begin
									StageAngle <= DataField[11:0];
								end
						8'h30:	begin
									resetConfig <= 1'b1;
									$display("%g\t FPGA_EvtBuild: configuration reset command received.",$time);
								end
						8'h31:  begin
						            GlbTrgDelay <= DataField[1:0];
									$display("%g\t FPGA_EvtBuild: setting global clock delay to %d",$time,DataField[2:0]);
						        end
						8'h32:  begin
						            TimePix <= DataField[0];
									$display("%g\t FPGA_EvtBuild:  setting timepix mode to %b",$time,DataField[0]);
						        end
						8'h33:	begin
									TimePixFrame <= DataField[15:0];
									$display("%g\t FPGA_EvtBuild:  setting the timepix frame to %d",$time,DataField[15:0]);
								end
						8'h34:  begin
									SpillSynch <= DataField[0];
								end
						8'h35:  begin
						            TrgBitsOut <= DataField[0];
						        end
						8'h40: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= NTagErr;
									OutputString[33:18]<= NHoldErr;
									OutputString[49:34]<= NDAQErr;
									OutputString[61:50]<= NCRCerr;
									OutputString[65:62]<= Error;
									OutputString[77:66]<= {2'b11,Address,6'b110001};	
									$display("%g\t FPGA_EvtBuild: NHoldErr=%d, NDAQErr=%d, NCRCerr=%d, NTagErr= %d, NAddressErr=%d, Error=%b",$time,NHoldErr,NDAQErr,NCRCerr,NTagErr,NAddressErr,Error);
								end
						8'h41:  begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= 0;
									OutputString[33:18]<= 0;
									OutputString[49:34]<= 0;
									OutputString[65:50]<= NAddressErr;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
								end
						8'h50: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= Nful1;
									OutputString[65:34]<= Nful2;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild front-end-FPGA buffer occupancy: Nful1=%d, Nful2=%d, Nful3=%d, Nful4=%d",$time,Nful1,Nful2,Nful3,Nful4);
								end
						8'h52: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= Nful3;
									OutputString[65:34]<= Nful4;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild front-end-FPGA buffer occupancy: Nful5=%d, Nful6=%d, Nful7=%d, Nful8=%d",$time,Nful5,Nful6,Nful7,Nful8);
								end
						8'h54: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= Nful5;
									OutputString[65:34]<= Nful6;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
								end
						8'h55: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= Nful7;
									OutputString[65:34]<= Nful8;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
								end
						8'h51:	begin	
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= {16'b0,Nbfl1};
									OutputString[65:34]<= {16'b0,Nbfl2};
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild FE buffer occupancy: Nbfl1=%d, Nbfl2=%d, Nbfl3=%d, Nbfl4=%d",$time,Nbfl1,Nbfl2,Nbfl3,Nbfl4);
								end
						8'h53:	begin	
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= {16'b0,Nbfl3};
									OutputString[65:34]<= {16'b0,Nbfl4};
									OutputString[77:66]<= {2'b11,Address,6'b110001};
								end
						8'h60:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2]<= NTrgHld;
									OutputString[65:34]<= {24'b0,NResetRs};
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: trg missed FE brd hold=%d, Ext. NtrgHld=%d, Tot. Trg Hold **Nbusy**=%d, NResetRs=%d",$time,Nhold,NTrgHld,Nbusy,NResetRs);
								end
						8'h61:	begin  
										OutputString[1:0] <= 2'b11;
										OutputString[33:2]<= NTrgTkr;
										OutputString[65:34]<= NTrgEnrg;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
										$display("%g\t FPGA_EvtBuild: NTrgTkr=%d, NTrgEnrg=%d.",$time,NTrgTkr,NTrgEnrg);
									end
						8'h62:	begin  
										OutputString[1:0] <= 2'b11;
										OutputString[33:2]<= NTrgEx;
										OutputString[65:34]<= NTrgGen;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
										$display("%g\t FPGA_EvtBuild: NTrgEx=%d, NTrgGen=%d.",$time,NTrgEx,NTrgGen);
									end
						8'h63:	begin
										OutputString[1:0] <= 2'b11;
										OutputString[17:2]<= ReadyStat;
										OutputString[33:18] <= {4'd0,FEbufStat};
										OutputString[65:34]<= FirstHold;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
										$display("%g\t FPGA_EvtBuild: FirstHold=%d, ReadyStat=%b, FEbufStat=%b.",$time,FirstHold,ReadyStat,FEbufStat);				
									end
						8'h67: 	begin
										OutputString[1:0] <= 2'b11;
										OutputString[17:2]<= 0;
										OutputString[33:18] <= 0;
										OutputString[49:34]<= 0;
										OutputString[65:50] <= BufHoldStat;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
										$display("%g\t FPGA_EvtBuild: FE-FPGA buffer status=%b.",$time,BufHoldStat);				
									end
						8'h66:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= nHldRdTrg;
									OutputString[33:18]<= NFEbufErr;
									OutputString[49:34]<= NenrgMiss; 
									OutputString[65:50]<= NcosmicMiss;
									OutputString[77:66]<= {2'b11,Address,5'b11000,BufMngrType};
									$display("%g\t FPGA_EvtBuild buffer manager errors=%d, # trgpls while sending Trg=%d",$time,NFEbufErr,nHldRdTrg);
									$display("%g\t FPGA_EvtBuild:  NenrgMiss=%d, NcosmicMiss=%d",$time,NenrgMiss,NcosmicMiss);
								end
						8'h64:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2]<= Nhold;
									OutputString[65:34]<= Nbusy;
									OutputString[77:66]<= {2'b11,Address,6'b110001};								
								end
						8'h65:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2]<= Nmissed;
									OutputString[65:34]<= NEthHld;
									OutputString[77:66]<= {2'b11,Address,6'b110001};		
									$display("%g\t FPGA_EvtBuild, # triggers missed for TKR FE busy=%d, # arrived during ethernet hold=%d",$time,Nmissed,NEthHld);
								end
						8'h80: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= Nlcl1;
									OutputString[33:18]<= Nlcl2;
									OutputString[49:34]<= Nlcl3;
									OutputString[65:50]<= Nlcl4;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild local buffer occupancy: Nlcl1=%d, Nlcl2=%d, Nlcl3=%d, Nlcl4=%d",$time,Nlcl1,Nlcl2,Nlcl3,Nlcl4);
								end
						8'h81: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= Nlcl13;
									OutputString[33:18]<= Nlcl14;
									OutputString[49:34]<= Nlcl15;
									OutputString[65:50]<= Nlcl16;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild local buffer occupancy: Nlcl13=%d, Nlcl14=%d, Nlcl15=%d, Nlcl16=%d",$time,Nlcl13,Nlcl14,Nlcl15,Nlcl16);
								end
						8'h82:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= minTrgTime;
									OutputString[33:18]<= 0;
									OutputString[65:34]<= NWordOut;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: minimum time between generated triggers=%d",$time,minTrgTime);
									$display("%g\t FPGA_EvtBuild: number of 12-bit words output=%d",$time,NWordOut);
								end
						8'h90: 	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= NSend;
									OutputString[65:34]<= NRead;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: NSend=%d, NRead=%d",$time,NSend,NRead);
								end
						8'h91:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= RunTime[35:4];
									OutputString[65:34]<= NTrgOut;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: NTrgOut=%d, RunTime=%d",$time,NTrgOut,RunTime);						
								end
						8'h92:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[33:2] <= NsndHold;
									OutputString[65:34]<= LiveTime[35:4];
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: LiveTime=%d, NsndHold=%d clock cycles",$time,LiveTime,NsndHold);				
								end
						8'h93:	begin
										DmpTrgMon <= 1'b1;
									end
						8'h94:	begin
										OutputString[1:0] <= 2'b11;
										OutputString[17:2] <= Nbcl3;
										OutputString[33:18]<= Nbcl2;
										OutputString[49:34]<= Nbcl1;
										OutputString[65:50]<= Nbcl0;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
									end
						8'h95:	begin
									OutputString[1:0] <= 2'b11;
									OutputString[17:2] <= NTrgOutExt;
									OutputString[33:18] <= 0;
									OutputString[65:34]<= NTrgInhibit;
									OutputString[77:66]<= {2'b11,Address,6'b110001};
									$display("%g\t FPGA_EvtBuild: NTrgOutExt=%d, NTrgInhibit=%d",$time,NTrgOutExt,NTrgInhibit);												
								end
						8'h98:	begin
										OutputString[1:0] <= 2'b11;
										OutputString[17:2] <= Nbcl7;
										OutputString[33:18]<= Nbcl6;
										OutputString[49:34]<= Nbcl5;
										OutputString[65:50]<= Nbcl4;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
									end
						8'h99:	begin
										OutputString[1:0] <= 2'b11;
										OutputString[17:2] <= Nbcl11;
										OutputString[33:18]<= Nbcl10;
										OutputString[49:34]<= Nbcl9;
										OutputString[65:50]<= Nbcl8;
										OutputString[77:66]<= {2'b11,Address,6'b110001};
									end
						8'h96:	begin  //Dump the configuration  
										OutputString <= {2'b11,Address,6'b110001,BufMngrType,RAMenable,TrgCnfg,TrigType,TagOut,TkrTrgMsk,MaskLyr,Ethernet,RAMbuf,ClkVsRst,TimePix,TimePixFrame,TrgBitsOut,6'd0,2'b11};
									end
						8'h97:	begin
										OutputString <= {2'b11,Address,6'b110001,TkrTrgLen,TkrTrgDly,CalTrgLen,CalTrgDly,1'b0,GlbTrgDelay,SpillSynch,26'd0,2'b11};
									end
						8'hE0:	begin
									I2Cmux <= Address;
									i2cAddr <= DataField[7:0];
									ReadDAC <= 1'b1;
									CntTime <= 0;
								end
						8'hF0:	begin
									I2Cmux <= Address;
									CntTime <= 0;
									if (NBytes == 3) begin
										StrobeDAC <= 1'b1;
										i2cAddr <= DataField[22:16];
									end else if (NBytes == 4) begin
										StrobeIna <= 1'b1;
										i2cAddr <= DataField[30:24];
									end
								end
						endcase
				end
		I2Cr:	begin
					ReadDAC <= 1'b0;
					CntTime <= CntTime + 1;
					OutputString <= {8'b11111011,I2Cmux,RegDAC,50'd0};
				end
		I2Cw:	begin
					StrobeDAC <= 1'b0;
					StrobeIna <= 1'b0;
					CntTime <= CntTime + 1;
				end
		Test:	begin
					Cnt <= Cnt + 1;
					OutputString <= {OutputString[76:0],1'b0};
				end
		Ct66:	begin
					Cnt <= Cnt + 1; 
					CmdOut <= {CmdOut[84:0],1'b0};
				end
		Stck:	begin
					Cnt4 <= Cnt4 + 1;
					TackOut <= {TackOut[2:0],1'b0};
				end
		DpWt:	begin
					CntTime<=CntTime+1;
					DMPStart <= 1'b0;
				end
		RnHd:	begin
					Cnt <= Cnt + 1;
					if (Cnt < 9) begin
						DataHdr <= RunHeader[Cnt];   //Sending the run header to the Ethernet output
						$display("%g\t FPGA_EvtBuild: Cnt=%d, Sending run header word %h",$time,Cnt,RunHeader[Cnt]);
						StrobeHdr <= 1'b1;
					end else StrobeHdr <= 1'b0;
				end
		ShCd:	begin
					HoldEndR <= 1'b0;
					AckRs <= 1'b0;
					Cnt4 <= Cnt4 + 1;
					ShrtCmd <= {ShrtCmd[8:0],1'b0};
					if (HoldEndR) $display("%g\t FPGA_EvtBuild ShCd, ShrtCmd=%b, Cnt4=%d, CMD=%b",$time,ShrtCmd,Cnt4,CMD);
				end
		LgCd:	begin
//					if (Cnt4==0) $display("%g\t FPGA_EvtBuild:  long command=%b",$time,LongCmd);
					TestStart <= 1'b0;
					Cnt4 <= Cnt4 + 1;
					LongCmd <= {LongCmd[24:0],1'b0};					
//					$display("%g\t FPGA_EvtBuild LgCd, LongCmd=%b, CMD=%b",$time,LongCmd,CMD);
				end
		ECmd:   begin
					if (InByte==8'h02) RunInProg <= 1'b1;
					if (InByte==8'h03) begin
						RunInProg <= 1'b0;
						if (RAMbuf) SendStream <= 1'b1;
						$display("%g\t FPGA_EvtBuild: State ECmd at end of run.  RAMbuf=%b",$time,RAMbuf);
					end
			    end
		ErgC:	begin
					Cnt <= Cnt + 1;
					if (Cnt == 16) begin
						EnrgCmd[17] <= PrtyE;
					end else begin
						PrtyE <= PrtyE^EnrgCmd[16];
						EnrgCmd <= {EnrgCmd[16:0],1'b0};
					end
				end
		EndS:   begin
					HoldEndR <= 1'b1;									//Hold the trigger, to let buffers flush out
		        end
		EndR:	begin
					if (CntTime < 3) TxD_start2 <= 1'b1;             	//Signal to the DAQ that the run is ending
					else TxD_start2 <= 1'b0;
					CntTime <= CntTime + 1;
//					$display("%g\t FPGA_EvtBuild: in state EndR.  CntTime=%d",$time,CntTime);
					if (CntTime==1023) begin    //Signal to the Ethernet interface that the run is ending
						endRun <= 1'b1;
						$display("%g\t FPGA_EvtBuild: endRun flag set. BufClr=%b. SendHold=%b. HoldTrig=%b. BusyHold=%b",$time,BufClr,SendHold,HoldTrig,BusyHold);
						$display("%g\t FPGA_EvtBuild: Local buffers full=%d, Enrg buffers full=%d.  ASIC buffers full=%d.",$time,NlclFull,NBufFull,NmostFullFE);
					end else endRun <= 1'b0;
				end
		EthT:	begin
					if (!EthBusy) begin
						CntTime <= CntTime + 1;   
					end
				end
		RsDl:	begin
					Cnt <= Cnt + 1;
				end
		DpRt:	begin
					CntTime <= CntTime + 1;
					DmpTrgMon <= 1'b0;
				end
		// Dtst:	begin
					// $display("%g\t FPGA_EvtBuild State Dtst: Data=%b, Address=%d, Nerr=%d, DoneTest=%b, TestStream=%b",$time,Data,Address,Nerr,DoneTest,TestStream);
				// end
	endcase
  end
end

wire [15:0] Nerr;
wire [23:0] Ntests, badTest;
reg TestStream, TestStart;
always @ (Data or Address or State) begin
	if (Address < 4'd12 && State == Dtst) begin
		TestStream = Data[Address];
	end else begin
		TestStream = 1'b0;
	end
end
evalTestStream evalTestStream0(Ntests,badTest,Nerr,DoneTest,TestStream,TestStart,Clock,Reset);

//Trigger logic.  First, stretch the trigger signals so that reliable coincidences can be formed.
//For the energy detector boards, decode the 4-bit serial word to pick off the 3 trigger bits
wire [9:0] TrgStrch;
TrgStretch Stretch0(Clock,Reset,FastOR[0],TrgStrch[0],TkrTrgLen,TkrTrgDly);		// V board
TrgStretch Stretch1(Clock,Reset,FastOR[1],TrgStrch[1],TkrTrgLen,TkrTrgDly);		// 2 halves of a T board
TrgStretch Stretch2(Clock,Reset,FastOR[2],TrgStrch[2],TkrTrgLen,TkrTrgDly);		// V board
TrgStretch Stretch3(Clock,Reset,FastOR[3],TrgStrch[3],TkrTrgLen,TkrTrgDly);		// 2 halves of a T board
TrgStretch Stretch4(Clock,Reset,FastOR[4],TrgStrch[4],TkrTrgLen,TkrTrgDly);		// 2 halves of a T board
TrgStretch Stretch5(Clock,Reset,FastOR[5],TrgStrch[5],TkrTrgLen,TkrTrgDly);		// V board
TrgStretch Stretch6(Clock,Reset,FastOR[6],TrgStrch[6],TkrTrgLen,TkrTrgDly);		// 2 halves of a T board
TrgStretch Stretch7(Clock,Reset,FastOR[7],TrgStrch[7],TkrTrgLen,TkrTrgDly);		// V board
//TrgStretch Stretch8(Clock,Reset,FastOR[8],TrgStrch[8],CalTrgLen,CalTrgDly);		// Energy digitizer board 1 (3 channels)
//TrgStretch Stretch9(Clock,Reset,FastOR[9],TrgStrch[9],CalTrgLen,CalTrgDly);		// Energy digitizer board 2 (2 channels)
TrgStretch StretchEx (Clock,Reset,TrgEx,TrgStrchEx,CalTrgLen,CalTrgDly);		// External trigger cleaned up here

//Receive the trigger and associated 3 bits of information from each energy board
wire [2:0] EnrgTrgWd0, EnrgTrgWd1;
RcvEnrgTrg RcvEnrgTrg0(.Clock(Clock), .Reset(Reset), .TReqIn(FastOR[8]), .TReqOut(EnrgTrg0), .TrgWord(EnrgTrgWd0), .Address(1'b0));
RcvEnrgTrg RcvEnrgTrg1(.Clock(Clock), .Reset(Reset), .TReqIn(FastOR[9]), .TReqOut(EnrgTrg1), .TrgWord(EnrgTrgWd1), .Address(1'b1));

reg CosmicCoincidence, EnrgCoincidence;
reg [3:0] TrigType;
always @ (TrigType or TkrTrgMsk or CalTrgMsk or TrgStrch or EnrgTrg0 or EnrgTrg1) begin
	if (TrigType == 4'h0) begin
		CosmicCoincidence = ((TrgStrch[0] | !TkrTrgMsk[0]) & (TrgStrch[1] | !TkrTrgMsk[1]))		//AND trigger logic
						&	((TrgStrch[2] | !TkrTrgMsk[2]) & (TrgStrch[3] | !TkrTrgMsk[3]))
						&	((TrgStrch[4] | !TkrTrgMsk[4]) & (TrgStrch[5] | !TkrTrgMsk[5]))
						&	((TrgStrch[6] | !TkrTrgMsk[6]) & (TrgStrch[7] | !TkrTrgMsk[7]));	
		EnrgCoincidence = ((EnrgTrg0 | !CalTrgMsk[0]) & (EnrgTrg1 | !CalTrgMsk[1]));
	end else begin
		CosmicCoincidence = ((TrgStrch[0] & TkrTrgMsk[0]) | (TrgStrch[1] & TkrTrgMsk[1])) 		//OR trigger logic
						|	((TrgStrch[2] & TkrTrgMsk[2]) | (TrgStrch[3] & TkrTrgMsk[3]))
						|	((TrgStrch[4] & TkrTrgMsk[4]) | (TrgStrch[5] & TkrTrgMsk[5]))
						|	((TrgStrch[6] & TkrTrgMsk[6]) | (TrgStrch[7] & TkrTrgMsk[7]));	
		EnrgCoincidence = ((EnrgTrg0 & CalTrgMsk[0]) | (EnrgTrg1 & CalTrgMsk[1]));
	end
end

always @ (posedge FastOR[0]) $display("%g\t FPGA_EvtBuild:  FPGA 0 Fast-OR=%b",$time,FastOR);
always @ (posedge TrgStrch[0]) $display("%g\t FPGA_EvtBuild: FPGA 0 trigger stretch=%b",$time,TrgStrch);
always @ (posedge EnrgCoincidence) $display("%g\t FPGA_EvtBuild: energy detector trigger coincidence.  EnrgTrg0=%b, EnrgTrg1=%b",$time,EnrgTrg0,EnrgTrg1);


//State machine to count trigger pulses for monitoring
parameter [1:0] LookSl = 2'b01;    //Look for the leading edge of a trigger signal
parameter [1:0] DoItSl = 2'b10;    //Count trigger pulses

reg [1:0] StateSl, NextStateSl;

always @ (StateSl or EnrgCoincidence or CosmicCoincidence or TrgStrchEx or TrgRndm or TrgCnfg or CalTrgMsk or TkrTrgMsk or TrgExLst or TrgTkrLst or TrgEnrgLst or TrgRndmLst) begin
	case (StateSl)
		LookSl:	begin
					if (TrgCnfg[1]) begin
						if (TrgStrchEx & !TrgExLst) NextStateSl = DoItSl;
						else NextStateSl = LookSl;
					end else if (TrgCnfg[0]) begin
						if (CalTrgMsk==2'b00) begin
							if (CosmicCoincidence & !TrgTkrLst) NextStateSl = DoItSl;
							else NextStateSl = LookSl;
						end else begin
							if (TkrTrgMsk != 0) begin
								if (CosmicCoincidence & !TrgTkrLst) NextStateSl = DoItSl;
								else NextStateSl = LookSl;
							end else begin
								if (EnrgCoincidence & !TrgEnrgLst) NextStateSl = DoItSl;
								else NextStateSl = LookSl;
							end
						end 
					end else begin
						if (TrgRndm & !TrgRndmLst) NextStateSl = DoItSl;
						else NextStateSl = LookSl;
					end
				end
		DoItSl: begin
					NextStateSl = LookSl;
				end
		default:	begin
						NextStateSl = LookSl;
					end
	endcase
end

reg TrgExLst, TrgTkrLst, TrgEnrgLst, TrgRndmLst;
reg [31:0] NTrgTkr, NTrgGen, NTrgEnrg, NTrgEx;
reg [15:0] NTrgOutExt;
reg [31:0] NTrgInhibit;
reg [35:0] TrigTime;
reg [15:0] delTime, minTrgTime;
always @ (posedge Clock) begin
	if (Reset) begin
		StateSl <= LookSl;
		NTrgEx <= 0;
		NTrgEnrg <= 0;
		NTrgTkr <= 0;
		NTrgGen <= 0;
		DeltaT <= 0;
		NTrgOut <= 0;
		Twrite <= 0;
		Tread <= 0;
		minTrgTime <= 16'b1111111111111111;
		TrigTime <= 0;
		NTrgInhibit <= 0;
	end else begin
		TrgExLst <= TrgStrchEx;
		TrgTkrLst <= CosmicCoincidence;
		TrgEnrgLst <= EnrgCoincidence;
		TrgRndmLst <= TrgRndm;
		StateSl <= NextStateSl;
		if (TrgPls) begin
			NTrgGen <= NTrgGen + 1;     //Count generated triggers 
		   if (HoldTimePix) NTrgInhibit <= NTrgInhibit + 1;
			$display("%g\t FPGA_EvtBuild: Generated trigger number %d, time since previous=%d clk ticks",$time,NTrgGen+1,RunTime-TrigTime);
			$display("%g\t FPGA_EvtBuild: trigger time set to %d",$time,RunTime);
			TrigTime <= RunTime;
			delTime <= RunTime - TrigTime;
			if (delTime < minTrgTime) minTrgTime <= delTime;
			DeltaT <= 0;
			DeltaTlast <= DeltaT;
		end else begin
			if (DeltaT != 12'b111111111111) DeltaT <= DeltaT + 1;
		end
		if (TrgPlsAcc) begin
			NTrgOut <= NTrgOut + 1;    //Count accepted triggers
			$display("%g\t FPGA_EvtBuild: Accepted trigger number %d.  Number generated=%d.  Number Tkr=%d.  Number Enrg=%d",$time,NTrgOut+1,NTrgGen,NTrgTkr,NTrgEnrg);
            $display("%g\t FPGA_EvtBuild: queuing delta-t=%d and trig time=%d in location %d",$time,DeltaTlast,TrigTime,Twrite);
			DeltaTbuf[Twrite] <= DeltaTlast;   //Queue the trigger delta-T values
			if (TimePix) begin
				TimeTags[Twrite] <= {TrgPlsAccOut,TrigTime[34:0]};  //Tag first event in frame when running with TimePix
			end else begin
				TimeTags[Twrite] <= TrigTime;  //Queue the time stamps
			end
			TrgWds[Twrite] <= {EnrgTrgWd1,EnrgTrgWd0};
			$display("%g\t FPGA_EvtBuild: writing %b into TrgWds at location %d.  TrgWds=%b %b %b %b %b %b %b %b",$time,{EnrgTrgWd1,EnrgTrgWd0},Twrite,TrgWds[0],TrgWds[1],TrgWds[2],TrgWds[3],TrgWds[4],TrgWds[5],TrgWds[6],TrgWds[7]);
			Twrite <= Twrite + 1;
		end
		if (EvStart) begin
			DeltaTout <= DeltaTbuf[Tread];
			TimeTagOut <= TimeTags[Tread];
			TrgWdOut <= TrgWds[Tread];
			$display("%g\t FPGA_EvtBuild: reading from TrgWds at location %d.  TrgWds=%b %b %b %b %b %b %b %b",$time,Tread,TrgWds[0],TrgWds[1],TrgWds[2],TrgWds[3],TrgWds[4],TrgWds[5],TrgWds[6],TrgWds[7]);
			Tread <= Tread + 1;
			$display("%g\t FPGA_EvtBuild: Number triggers accepted=%d.",$time,NTrgOut);
			$display("%g\t FPGA_EvtBuild: delta-t=%d, time tag=%d from location %d",$time,DeltaTbuf[Tread],TimeTags[Tread],Tread);
			$display("%g\t FPGA_EvtBuild: Local buffers full=%d, Enrg buffers full=%d.  ASIC buffers full=%d.",$time,NlclFull,NBufFull,NmostFullFE);
		end
		case (StateSl)
			LookSl:	begin
						if (RunInProg & !HoldEndR) begin
							if (TrgStrchEx & !TrgExLst) NTrgEx <= NTrgEx + 1;
							if (CosmicCoincidence & !TrgTkrLst) begin
								NTrgTkr <= NTrgTkr + 1;
								$display("%g\t FPGA_EvtBuild: CosmicCoincidence number %d",$time,NTrgTkr+1);
							end
							if (EnrgCoincidence & !TrgEnrgLst) begin
								NTrgEnrg <= NTrgEnrg + 1;
								$display("%g\t FPGA_EvtBuild: EnrgCoincidence number %d",$time,NTrgEnrg+1);
							end
						end
					end
		endcase
	end
end

//Global trigger logic
wire [15:0] NenrgMiss, NcosmicMiss;
assign TrgPlsNoDly = (TrgPlsOut & !Abort);
TriggerLogic TriggerLogic0(NenrgMiss,NcosmicMiss,TrgPlsOut,EnrgCoincidence,CosmicCoincidence,TrgStrchEx,TrgRndm,RunInProg,HoldEndR,TrgCnfg,CalTrgMsk,TkrTrgMsk,Clock,Reset);

//Delay the trigger by a programmed amount, without adding any dead time.
reg [2:0] TrgShft;
always @ (posedge Clock) begin
    if (Reset) begin
	    TrgShft <= 0;
	end else begin
        TrgShft <= {TrgShft[1:0],TrgPlsNoDly};
	end
end

reg TrgPls;
always @ (TrgShft or TrgPlsNoDly or GlbTrgDelay) begin
    case (GlbTrgDelay)
	   2'd0: TrgPls = TrgPlsNoDly;
		2'd1: TrgPls = TrgShft[0];
		2'd2: TrgPls = TrgShft[1];
		2'd3: TrgPls = TrgShft[2];
	endcase
end

//Modify the triggering for operating together with TimePix
parameter [4:0] WaitPx=5'b00001;                //Wait for a starting trigger
parameter [4:0] TrigPx=5'b00010;                //Generate 3-clock trigger signal
parameter [4:0] FramPx=5'b00100;        			//During the TimePix frame trigger pCT but not TimePix
parameter [4:0] InhbPx=5'b01000;                //After the TimePix frame, inhibit the trigger until TimePix is ready again
parameter [4:0] DlayPx=5'b10000;                //Fixed delay following the inhibit          
reg [4:0] StatePx, NextStatePx;
reg TrgPlsAccOut;
reg [19:0] CntPx;

reg HoldTimePix;
always @ (StatePx or TrgPlsAcc or TimePix or TrgInhibit or CntPx or TimePixFrame) begin
	case (StatePx)
		WaitPx:	begin
					  if (TimePix & TrgPlsAcc) NextStatePx = TrigPx;
					  else NextStatePx = WaitPx;
					  TrgPlsAccOut = TrgPlsAcc;
					  HoldTimePix = 1'b0;
				   end
		TrigPx:  begin
					  TrgPlsAccOut = 1'b1;
					  HoldTimePix = 1'b0;
					  if (CntPx == 2) NextStatePx = FramPx;
					  else NextStatePx = TrigPx;
					end
		FramPx:	begin
					  TrgPlsAccOut = 1'b0;
					  if (CntPx[17:2] == TimePixFrame) NextStatePx = InhbPx;
					  else NextStatePx = FramPx;
					  HoldTimePix = 1'b0;
				   end
		InhbPx:	begin
					  TrgPlsAccOut = 1'b0;
					  HoldTimePix = 1'b1;
					  if (!TrgInhibit) NextStatePx = DlayPx;
					  else NextStatePx = InhbPx;
				   end
		DlayPx:	begin
		           if (CntPx == 1000000) NextStatePx = WaitPx;
					  else NextStatePx = DlayPx;
					  TrgPlsAccOut = 1'b0;
					  HoldTimePix = 1'b1;
					end
		default:	begin
						NextStatePx = WaitPx;
						TrgPlsAccOut = TrgPlsAcc;
						HoldTimePix = 1'b0;
					end
	endcase
end 

always @ (posedge Clock) begin
	if (Reset) begin
		StatePx <= WaitPx;
		NTrgOutExt <= 0;
	end else begin
		StatePx <= NextStatePx;
		case (StatePx) 
			WaitPx:	begin
						   CntPx <= 0;
							if (TrgPlsAcc) begin
			               NTrgOutExt <= NTrgOutExt + 1;	//Count triggers going out to TimePix
		               end
					   end
			TrigPx:  begin
			            CntPx <= CntPx + 1;
						end
			FramPx:	begin
						   CntPx <= CntPx + 1;
					   end
			DlayPx:  begin
			            CntPx <= CntPx + 1;
						end
		endcase
	end
end

//Monitor the trigger rate versus time
wire [7:0] TxD_data1;
reg [7:0] TxD_data2;
reg TxD_start2;
reg DmpTrgMon;			//Signal to initiate output of the trigger monitoring results
BeamMonitor BeamMonitorU(SpillFlag,DmpTrgDone, TxD_start1, TxD_data1, TxD_busy, DmpTrgMon, TrgPls, TrgPlsAcc, RunInProg, Clock, Reset);
always @ (TxD_start0 or TxD_data0 or TxD_start1 or TxD_data1 or State or TxD_start2 or TxD_data2) begin
	if (State == DpRt) begin
		TxD_start = TxD_start1;
		TxD_data = TxD_data1;
	end else if (State == EndR) begin
	    TxD_start = TxD_start2;
		TxD_data = TxD_data2;
	end else begin
		TxD_start = TxD_start0;
		TxD_data = TxD_data0;
	end
end

//MUX the tracker command output lines
always @ (ShrtCmd or CmdOut or CmdRead or State or RunInProg or TrgCnfg or InByte or HoldExt or LongCmd)
begin
	if (RunInProg) begin
		if (State==ShCd)
			CMD = ShrtCmd[9];         //Short command to the front-end boards
		else if ((State==Deco || State==Ct66) && TrgCnfg==0) 
			CMD = CmdOut[85];		  //Command to the tracker ASICs in calibration runs.
		else
			CMD = CmdRead;			  //Read and Send commands generated automatically to acquire data during the run
	end else begin
		if ((State==Deco && InByte==8'h01) || State==Ct66) CMD = CmdOut[85];    //Commands to ASICs
		else if (State==ShCd) CMD = ShrtCmd[9];				//Short commands to front-end boards
		else if (State==LgCd) CMD = LongCmd[25];			//Long commands to front-end boards
		else CMD = 1'b0;
	end
end

always @ (State or EnrgCmd or CmdReadE or RunInProg) begin
	if (RunInProg) begin
		CmdE = CmdReadE;							//Read commands generated automatically to acquire data during the run
	end else begin
		if (State==ErgC) CmdE = EnrgCmd[17];		//Commands to the front-end energy boards
		else CmdE = 1'b0;
	end
end

assign Diagnostic1 = BufClrStrm[0];
assign Diagnostic2 = TrgPlsAccOut;      //Routed to the external trigger SMA output, to trigger TimePix 

//Multiplex the data line going into the block RAM storage
always @ (Data or RAMenable or State or OutputString or RunInProg or Address or Data or DataStream or MaskLyr) begin
	if (State==Test) DataStore = OutputString[77];
	else if (RAMenable) begin
		if (RunInProg) begin
			DataStore = 0;
		end else if (State == Dtst) begin
			DataStore = 0;
		end else begin
			case (Address)
				4'h0: DataStore = (Data[0] & MaskLyr[0]) | DataStream;    //DataStream is event data that normally would go to Ethernet
				4'h1: DataStore = (Data[1] & MaskLyr[1]) | DataStream; 
				4'h2: DataStore = (Data[2] & MaskLyr[2]) | DataStream;
				4'h3: DataStore = (Data[3] & MaskLyr[3]) | DataStream; 
				4'h4: DataStore = (Data[4] & MaskLyr[4]) | DataStream;
				4'h5: DataStore = (Data[5] & MaskLyr[5]) | DataStream; 
				4'h6: DataStore = (Data[6] & MaskLyr[6]) | DataStream;
				4'h7: DataStore = (Data[7] & MaskLyr[7]) | DataStream; 
				4'h8: DataStore = (Data[8] & MaskLyr[8]) | DataStream;
				4'h9: DataStore = (Data[9] & MaskLyr[9]) | DataStream; 
				4'ha: DataStore = (Data[10] & MaskLyr[10]) | DataStream;
				4'hb: DataStore = (Data[11] & MaskLyr[11]) | DataStream; 
				4'hc: DataStore = (Data[12] & MaskLyr[12]) | DataStream;
				4'hd: DataStore = (Data[14] & MaskLyr[14]) | DataStream;  //register readback info comes on just the first data line of E-det board.
				4'he: DataStore = 1'b0;
				4'hf: DataStore = DataStream;
			endcase
		end
	end else begin
		DataStore = 1'b0;
	end
end

//Block RAM storage used to buffer bits going back to the computer via UART
assign RAMReset = RstRam | Reset;
wire [7:0] TxD_data0;
DataRAMbufferSimple DataRAMbuffer_DAT(TxD_start0, TxD_data0, DMPDone, DMPStart, TxD_busy, DataStore, Clock, RAMReset, 4'b0000);

//Mux the outgoing trigger signal
always @ (Tack or TackOut[3] or State) begin
	if (State == Stck) 
		TrgOut = TackOut[3];     //Trigger signal generated by explicit command
	else
		TrgOut = Tack;			 //Trigger signal generated from the internal trigger logic
end

//Counters for monitoring the data acquisition triggers
reg [15:0] nHldRdTrg;
reg [15:0] Nbcl0,Nbcl1,Nbcl2,Nbcl3,Nbcl4,Nbcl5,Nbcl6,Nbcl7,Nbcl8,Nbcl9,Nbcl10,Nbcl11;
always @ (posedge Clock) begin
	if (Reset) begin
		Nbcl0 <= 0;
		Nbcl1 <= 0;
		Nbcl2 <= 0;
		Nbcl3 <= 0;
		Nbcl4 <= 0;
		Nbcl5 <= 0;
		Nbcl6 <= 0;
		Nbcl7 <= 0;
		Nbcl8 <= 0;
		Nbcl9 <= 0;
		Nbcl10 <= 0;
		Nbcl11 <= 0;
		Nbusy <= 0;
		Nful1 <= 0;
		Nful2 <= 0;
		Nful3 <= 0;
		Nful4 <= 0;
		Nful5 <= 0;
		Nful6 <= 0;
		Nful7 <= 0;
		Nful8 <= 0;
		NTrgHld <= 0;
		Nlcl1 <= 0;
		Nlcl2 <= 0;
		Nlcl3 <= 0;
		Nlcl4 <= 0;
		NEthHld <= 0;
		Nlcl13 <= 0;
		Nlcl14 <= 0;
		Nlcl15 <= 0;
		Nlcl16 <= 0;
		Nbfl1 <= 0;
		Nbfl2 <= 0;
		Nbfl3 <= 0;
		Nbfl4 <= 0;
		nHldRdTrg <= 0;
		Nhold <= 0;
	end else begin
		if (Blip[0]) Nbcl0 <= Nbcl0 + 1;
		if (Blip[1]) Nbcl1 <= Nbcl1 + 1;
		if (Blip[2]) Nbcl2 <= Nbcl2 + 1;
		if (Blip[3]) Nbcl3 <= Nbcl3 + 1;
		if (Blip[4]) Nbcl4 <= Nbcl4 + 1;
		if (Blip[5]) Nbcl5 <= Nbcl5 + 1;
		if (Blip[6]) Nbcl6 <= Nbcl6 + 1;
		if (Blip[7]) Nbcl7 <= Nbcl7 + 1;
		if (Blip[8]) Nbcl8 <= Nbcl8 + 1;
		if (Blip[9]) Nbcl9 <= Nbcl9 + 1;
		if (Blip[10]) Nbcl10 <= Nbcl10 + 1;
		if (Blip[11]) Nbcl11 <= Nbcl11 + 1;
		if (TrgPls) begin
			$display("%g\t FPGA_EvtBuild TrgPls: NBufFull=%d, NmostFullFE=%d, NlclFull=%d, HoldEth=%b, HldRdTrg=%b, HoldTrig=%b, HoldExt=%b",$time,NBufFull,NmostFullFE,NlclFull,HoldEth,HldRdTrg,HoldTrig,HoldExt);
			if (BusyHold) begin
				Nbusy <= Nbusy + 1;
				$display("%g\t FPGA_EvtBuild: trigger missed due to buffer manager putting trigger on hold.  Nbusy=%d Bufree=%b BufClr=%b",$time,Nbusy,Bufree,BufClr);
			end
			if (HldRdTrg) nHldRdTrg <= nHldRdTrg + 1;
			if (HoldTrig) begin
				Nhold <= Nhold + 1;
				$display("%g\t FPGA_EvtBuild: trigger missed due to FE board buffers being full.  Nhold=%d Bufree=%b BufClr=%b",$time,Nhold,Bufree,BufClr);
			end
			case (NBufFull)
				4'h1: Nful1 <= Nful1 + 1;
				4'h2: Nful2 <= Nful2 + 1;
				4'h3: Nful3 <= Nful3 + 1;
				4'h4: Nful4 <= Nful4 + 1;
				4'h5: Nful5 <= Nful5 + 1;
				4'h6: Nful6 <= Nful6 + 1;
				4'h7: Nful7 <= Nful7 + 1;
				4'h8: Nful8 <= Nful8 + 1;
			endcase
			case (NmostFullFE)
				3'b001: Nbfl1 <= Nbfl1 + 1;
				3'b010: Nbfl2 <= Nbfl2 + 1;
				3'b011: Nbfl3 <= Nbfl3 + 1;
				3'b100: Nbfl4 <= Nbfl4 + 1;
			endcase
			case (NlclFull)
				5'h01: Nlcl1 <= Nlcl1 + 1;
				5'h02: Nlcl2 <= Nlcl2 + 1;
				5'h03: Nlcl3 <= Nlcl3 + 1;
				5'h04: Nlcl4 <= Nlcl4 + 1;
				5'h05: Nlcl13 <= Nlcl13 + 1;
				5'h06: Nlcl14 <= Nlcl14 + 1;
				5'h07: Nlcl15 <= Nlcl15 + 1;
				5'h08: Nlcl16 <= Nlcl16 + 1;
			endcase
			if (HoldExt) NTrgHld <= NTrgHld + 1;
			if (HoldEth) NEthHld <= NEthHld + 1;
		end
	end
end

//Instantiate the program to monitor the buffer clear signals from the front-end tracker boards
wire [15:0] NFEbufErr;
wire [2:0] NmostFullFE;
wire [11:0] BufClr, Blip;
BufClrMngr BufClrMngr_U(MaskTkr, NFEbufErr, Blip, BufClr, BufClrStrm, TrgPlsAcc, Clock, Reset, NmostFullFE);
																				
//Instantiate the program to monitor the front-end buffers and generate read commands and triggers
wire [31:0] Nmissed;
pCT_BufMngr pCT_BufMngr_U (HoldError, HoldExt | HoldEth | HoldTimePix, HldRdTrg, ResetRs | Reset, Clock, TrgPls, Tack, CmdRead, CmdReadE, BusyHold, SendPls, HoldTrig, SendHold, BufClr, TrgPlsAcc, BufMngrType, NRead, Nmissed);
always @ (posedge HoldTrig) $display("%g\t FPGA_EvtBuild:  trigger hold signal asserted.  FE FPGA buffers full!",$time);
always @ (HoldEth) $display("%g\t FPGA_EvtBuild:  Ethernet hold status changed = %b",$time,HoldEth);

//Instantiate the program to build the event from multiple data streams
always @ (Data or RunInProg or MaskLyr) begin
	if (RunInProg) EventData = (Data & MaskLyr);
	else EventData = 0;
end

wire [15:0] BufRdy, Bufree;
EvtBuild EvtBuild_U(TrgPlsAcc, Bufree, BufRdy, EvStart, TagErr, CRCerr, NlclFull, HoldTrig, NBufFull, Nevent, ErrSgnl, AddErr, SendHold, DataOut, StrobeOut, Clock, Reset, EventData, MaskLyr, HoldEth, DeltaTout, TimeTagOut, TagOut, BufMngrType, TrgBitsOut, TrgWdOut);
always @ (posedge Clock) begin
	if (Reset) begin
		NWordOut <= 0;
	end else begin
		if (StrobeOut) begin
			$display("%g\t FPGA_EvtBuild:  Event data output word=%b, %h, %d",$time,DataOut,DataOut,DataOut);
			NWordOut <= NWordOut + 1;
		end
	end
end
//PrntPctEvnt PrntPctEvnt0(DataOut,StrobeOut,Clock,Reset,TagOut,endRun);     //Readable data print-out for test-bench work

//Ethernet data output:
wire [15:0] EthDataOut;
reg [11:0] DataIn;
reg StrobeIn;
always @ (State or StrobeHdr or DataHdr or StrobeOut or DataOut) begin
	if (State == RnHd) begin
		StrobeIn = StrobeHdr;
		DataIn = DataHdr;
	end else begin
		StrobeIn = StrobeOut;
		DataIn = DataOut;
	end
end
EtherBuffer EtherBufferU(EthDataOut,EthStrobeOut,DataIn,StrobeIn,endRun,Clock,Reset);
assign HoldEth = Ethernet & EthBusy;
always @ (RunInProg or Ethernet or EthStrobeOut or EthDataOut or State or EthBusy or CntTime) begin
	if (RunInProg && Ethernet) begin
		EthEn = EthStrobeOut;
		EthDat = EthDataOut;
	end else if (State == EthT) begin
		if (EthBusy) EthEn = 1'b0;
		else EthEn = 1'b1;
		EthDat = CntTime[15:0];
	end else begin
		EthEn = 1'b0;
		EthDat = 0;
	end
end

//Instantiate a program to buffer the data and then stream them to DataRAMbuffer, if requested
EvtBufTmp16  EvtBufTmp_U(DataStream,SendStream,EthDataOut,(EthStrobeOut & RAMbuf),Clock,Reset);

//State machine to monitor the incoming data for front-end chip errors and reset the front-end chips
//and the buffer management if necessary
parameter [3:0] IdleRs = 4'b0001;	//Look for an error signal to arrive
parameter [3:0] HoldRs = 4'b0010;	//Hold the trigger and wait for data to stop flowing
parameter [3:0] RsetRs = 4'b0100;	//Force a reset signal to the buffer manager and front-end board
parameter [3:0] WaitRs = 4'b1000;	//Keep the trigger held for a millisecond for all reset activity to complete

reg [3:0] StateRs, NextStateRs;
reg [16:0] CntRs;

always @ (StateRs or ErrSgnl or CntRs or AckRs) begin
	case (StateRs)
		IdleRs:	begin
					if (ErrSgnl) NextStateRs = HoldRs;
					else NextStateRs = IdleRs;
				end
		HoldRs:	begin
					if (CntRs[9:0] == 10'b1111111111) NextStateRs = RsetRs;
					else NextStateRs = HoldRs;
				end
		RsetRs:	begin
					if (AckRs) NextStateRs = WaitRs;
					else NextStateRs = RsetRs;
				end
		WaitRs:	begin
					if (CntRs == 17'b11111111111111111) NextStateRs = IdleRs;
					else NextStateRs = WaitRs;
				end
		default:	begin
						NextStateRs = IdleRs;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateRs <= IdleRs;
		ResetRs <= 1'b0;
		NResetRs <= 0;
	end else begin
		StateRs <= NextStateRs;
		NeventLst <= Nevent[5:0];
		case (StateRs) 
			IdleRs:	begin
						HoldExt <= 1'b0;
						CntRs <= 0;
					end
			HoldRs:	begin
						$display("%g\t FPGA_EvtBuild:  DAQ error encountered.  Starting reset procedure.",$time);
						HoldExt <= 1'b1;
						if (Nevent[5:0] > NeventLst) CntRs <= 0;
						else CntRs <= CntRs + 1;
					end
			RsetRs:	begin
						ResetRs <= 1'b1;
						CntRs <= 0;
					end
			WaitRs:	begin
						if (CntRs == 0) NResetRs <= NResetRs + 1;
						CntRs <= CntRs + 1;
						ResetRs <= 1'b0;
					end
		endcase
	end
end

reg SDAout1, SCL1;

//MUX signals between the two I2C programs
assign SDAen1 = SDAenIna | SDAenDAC;
always @ (SDAenIna or SDAoutIna or SDAoutDAC) begin
	if (SDAenIna) SDAout1 = SDAoutIna;
	else SDAout1 = SDAoutDAC;
end
assign SCLen1 = SCLenIna | SCLenDAC;
always @ (SCLenIna or SCLina or SCLDAC) begin
	if (SCLenIna) SCL1 = SCLina;
	else SCL1 = SCLDAC;
end

//Instantiate the program to load and read the registers of the DAC chips, or any i2c chip needing a 2-byte data field.
reg SDAin1;
LoadDAC LoadDAC_U(DACerr,RegDAC,i2cDatRdy,SCLDAC,SCLenDAC,SDAoutDAC,SDAenDAC,SDAin1,DataField[15:0],i2cAddr,StrobeDAC,ReadDAC,Clock,Reset,Reset);
always @ (posedge i2cDatRdy) begin
	$display("%g\t EnergyEvtBuild:  finished reading I2C register.  Contents=%b",$time,RegDAC);
end

//Instantiate the program to load the registers of the ina226 chips or the ads1013 ADC, or any i2c chip needing a 3-byte data field
//This program can also read the registers, but LoadDAC does exactly the same for that purpose.
LoadIna226 LoadIna226_U(InaErr,RegIna,inaDatRdy,SCLina,SCLenIna,SDAoutIna,SDAenIna,SDAin1,DataField[23:0],i2cAddr,StrobeIna,ReadIna,Clock,Reset,Reset);

//Route the i2c signals to and from the correct board 
always @ (I2Cmux or SCL1 or SDAout1 or SDAin or SDAen1 or SCLen1) begin
	case (I2Cmux)
		4'h0: begin SCL = {9'b111111111,SCL1};			SDAout = {9'b111111111,SDAout1};		SDAin1 = SDAin[0];	SDAen = {9'b111111111,~SDAen1};			SCLen = {9'b111111111,~SCLen1};			end
		4'h1: begin SCL = {8'b11111111,SCL1,1'b1};		SDAout = {8'b11111111,SDAout1,1'b1};	SDAin1 = SDAin[1];	SDAen = {8'b11111111,~SDAen1,1'b1};	SCLen = {8'b11111111,~SCLen1,1'b1};	end
		4'h2: begin SCL = {7'b1111111,SCL1,2'b11};		SDAout = {7'b1111111,SDAout1,2'b11};	SDAin1 = SDAin[2];	SDAen = {7'b1111111,~SDAen1,2'b11};	SCLen = {7'b1111111,~SCLen1,2'b11};	end
		4'h3: begin SCL = {6'b111111,SCL1,3'b111};		SDAout = {6'b111111,SDAout1,3'b111};	SDAin1 = SDAin[3];	SDAen = {6'b111111,~SDAen1,3'b111};	SCLen = {6'b111111,~SCLen1,3'b111};	end		
		4'h4: begin SCL = {5'b11111,SCL1,4'b1111};		SDAout = {5'b11111,SDAout1,4'b1111};	SDAin1 = SDAin[4];	SDAen = {5'b11111,~SDAen1,4'b1111};	SCLen = {5'b11111,~SCLen1,4'b1111};	end
		4'h5: begin SCL = {4'b1111,SCL1,5'b11111};		SDAout = {4'b1111,SDAout1,5'b11111};	SDAin1 = SDAin[5];	SDAen = {4'b1111,~SDAen1,5'b11111};	SCLen = {4'b1111,~SCLen1,5'b11111};	end
		4'h6: begin SCL = {3'b111,SCL1,6'b111111};		SDAout = {3'b111,SDAout1,6'b111111};	SDAin1 = SDAin[6];	SDAen = {3'b111,~SDAen1,6'b111111};	SCLen = {3'b111,~SCLen1,6'b111111};	end
		4'h7: begin SCL = {2'b11,SCL1,7'b1111111};		SDAout = {2'b11,SDAout1,7'b1111111};	SDAin1 = SDAin[7];	SDAen = {2'b11,~SDAen1,7'b1111111};	SCLen = {2'b11,~SCLen1,7'b1111111};	end
		4'h8: begin SCL = {1'b1,SCL1,8'b11111111};		SDAout = {1'b1,SDAout1,8'b11111111};	SDAin1 = SDAin[8];	SDAen = {1'b1,~SDAen1,8'b11111111};	SCLen = {1'b1,~SCLen1,8'b11111111};	end
		4'h9: begin SCL = {SCL1,9'b111111111};		SDAout = {SDAout1,9'b111111111};	SDAin1 = SDAin[9];	SDAen = {~SDAen1,9'b111111111};	SCLen = {~SCLen1,9'b111111111};	end
		default: begin SCL = 10'b1111111111;	SDAout = 10'b1111111111;	SDAin1 = 1'b1;		SDAen = 10'b1111111111;		SCLen = 10'b1111111111;	end
	endcase
end

endmodule
