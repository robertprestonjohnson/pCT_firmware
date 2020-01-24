//Data acquisition code to run in the Spartan-6 FPGA on the pCT energy detector digitizer board
//R. Johnson,  May 2013
//July 11, 2013 -- Modified not to write out the delta-T word for reduced data and 3 channels
//				   This is to avoid repeating that word between the two energy digitizer boards.
//August 1, 2013 -- fixed trigger logic choice.  V3
//August 4, 2013 -- Added facility to set the pedestal offset independently for each channel.  V4
//August 12, 2013-- Added queue for the read event commands.  V5
//August 16, 2013-- Added some summary printout for test bed work.  V6
//August 17, 2013-- Added parity bit to trigger stream.  V7
//August 22, 2013-- Added TReq counters. V8
//August 27, 2013-- Reset scheme modified.  V9
//September 10, 2013-- Added buffer occupancy monitoring.  V10
//September 10, 2013-- Added a command to reset the configuration.  V11
//September 13, 2013-- Increased the number of buffers from 4 to 8.  V12
//September 18, 2013-- Added stretching of trigger signals prior to coincidence.  V13
//September 26, 2013-- Fixed bug in trigger queue.  V14
//October 8, 2013-- Fixed zeroing of counters and event counting.  V15
//October 16, 2013-- Changed delta-T count so that it doesn't wrap around.  V16
//October 25, 2013-- Send all 3 read tag bits in the command.  V17
//October 28, 2013-- Eliminated debug mode and added option not to write out pedestals.  V18
//November 1, 2013-- Move the data reduction to occur while reading from the FIFO, to save time.  V19
//January 15, 2014-- Remove the calculation and output of the time since the previous trigger.  V20
//January 26, 2014-- Modified to use both available data links when operating with reduced data and pedestals output
//February 27, 2014-- Add selection between internal and external clock.  V22
//March 16, 2014-- Increase the read delay range to 5 bits and improved logic for checking pointer difference.  V23
//March 28, 2014-- Fixed bug in the queuing of read commands that caused the slow readout in the last beam test.  V24
//April 19, 2014-- small improvements in monitoring. V25
//April 21, 2014-- Changed the data reduction algorithm to add a fixed number of samples around the peak, with a fixed pedestal.  V26
//April 23, 2014-- Add some debugging stuff. V27
//April 23, 2014-- Attempted timing improvements.  V28
//April 25, 2014-- Removed check on address interference, because it cannot work with noncomensurate frequencies.  V30 
//April 26, 2014-- Implemented dual ported RAM for the start read-address FIFO to ensure that we latch it with the correct clock.  V31
//April 14, 2016-- Send the TReq out as a 4-bit string with the trigger bit pattern encoded.  V32
module EnergyDAQ(SDAin,SCLin,ClkSelect,ClkGate,Dout,Dout2,TReqOut4,Clock,ClkDigi,ResetHard,CmdIn,TrgIn,TReqComp,Dig0,Dig1,Dig2,Address);

output Dout;			//Serial data output stream going to the event builder
output Dout2;			//Second serial output stream going to the event builder
output TReqOut4;	    //Output trigger information to the trigger processor in the event builder
						//That trigger processor may use tracker and/or energy-detector information
output ClkSelect;		//Select between internal (0) and external (1) clock sources for the digitizers
output ClkGate;			//Switch to turn the digitizer clocks on/off
input Clock;			//100 MHz system clock
input ClkDigi;			//Digitizer clock, up to 65 MHz
input ResetHard;		//System reset.  Reset the command interpreter and everything else.
input CmdIn;			//Serial command stream from the event builder
input TrgIn;			//Input trigger acknowledge from the trigger processor in the event builder
input [2:0] TReqComp;	//Trigger requests from the 3 channel comparators
input [14:0] Dig0;		//Digitizations and OTR from first channel
input [14:0] Dig1;		//Digitizations and OTR from second channel
input [14:0] Dig2;		//Digitizations and OTR from third channel
input [3:0] Address;    //Board address, set by switches on the energy detector digitizer board
input SDAin,SCLin;		//i2c signals not used, except to avoid them getting optimized away

wire ResetHard;
reg Dout, TReqOut;
reg TrgError, RdyError, TimeOut, ClkGate, ClkSelect;
reg TReqOut4;

reg [23:0] CntTime;		//Big counter for implementing time-outs for some commands
reg [15:0] NPrtyError, NTimeOut, NRdyError, NTrgError;
							
reg [2:0] TrgMask;		// Trigger mask, set bit to one for a channel to participate in the trigger
reg TrgType;			// 1 = OR of 3 comparators
						// 0 = AND of 3 comparators
		
parameter [7:0] Version=8'd32;
		
//Commands to this program:
//Format:
//		Start bit
//		4-bit board address
//		4-bit command, or 8-bit command
//		8-bit data field, or 4-bit data field (for 8-bit commands)
//		parity bit (even parity expected, not counting start bit)
//Commands (15 4-bit commands, followed by the 8-bit commands):
// 0x:  Send out a 78-bit test pattern.
// 1x:  Load the trigger delay read offset.  Data field is the offset.
// 2x:  Load the number of samples.  Data field is the number of samples 1 to 16.
// 3x:  Set output option.  Data field 0=send all samples, 1=send reduced data
// 4x:  Set the pedestal offset value = datafield * 16;
// 5x:  Read an event.  Bits 2-0 of the data field give the 3-bit buffer number.
// 6x:  Read out trigger counters and error flags as well as register settings.
//				Data field 00= trigger counts
//				Data field 01= reads and sends
//				Data field 02= error counts
//				Data field 03= trigger delay offset sign and number of channels to output
//				Data field 04, 05= trigger output counts for the 3 channels
//				Data field 06= buffer occupancies
//				Data field 07= more buffer occupancies
//				Data field 0a= trigger delay offset
//				Data field 0b= settings for number of samples and read delay
//				Data field 0c= error monitoring
//				Data field 0d= error monitoring
//				Data field 0e= calibrated pedestals
//				Data field 0f= pedestal offsets
// 7x:  Reset the DAQ
// 8x:  Set the trigger mask (3 LSB of the data field)  7 to enable all 3 channels
// 9x:  Set the trigger type (LSB of the data field)    1 for OR, 0 for AND
// ax:  Set number of samples to sum and maximum position of the peak.
// bx:  Reset the configuration
// cx:  Load the read delay (may be needed if read clock is faster than digitizer clock).  Delay is 5 LSB of data field. 
// dx:  Load 8 bits into the calibrated pedestal. See command f6 to set which pedestal is loaded.  Don't forget that the pedestal is signed!
// ex:  Turn digitizer clocks on=1, off=0 (LSB of data field)
// f0:  Set the sign of the trigger delay offset (0= positive, normal; 1= negative)
// f1:  Choose whether to write out pedestals (1= yes, 0 = no)
// f2:  Set how many channels to write out in case of data reduction (1 to 3)
// f3:	Set to which channels the pedestal offset of commmand 4x gets applied.  Data field = 001, 010, 100, 011, etc
// f4:	Set the length in clock cycles to which to stretch the trigger signals before doing coincidence
// f5:	Select the source of the digitizer clock (0= internal, 1= external)
// f6:  Set which pedestal to load:
//			0000 = load least significant 8 bits of pedestal for channel 0
//			0001 = load least significant 8 bits of pedestal for channel 1
//			0010 = load least significant 8 bits of pedestal for channel 2
//			0100 = load most significant 6 bits of pedestal for channel 0
//			0101 = load most significant 6 bits of pedestal for channel 0
//			0110 = load most significant 6 bits of pedestal for channel 0
// f7:  Dump the 3 calibrated pedestals
//
// Algorithm:  For each of the 3 channels the 15-bit digitizer output (including out-of-range bit) is written into a FIFO
// buffer on every cycle of the digitizer clock (65 MHz maximum).  The FIFO is implemented as dual ported RAM, so it can
// be read at the same time it is being written.  It is written typically at 65 MHz and read at 100 MHz.  Both the write
// address and the starting read address are incremented every clock cycle of the digitizer clock, so the two pointers 
// are always separated by a fixed amount, which has to be tuned to correspond to the latency of the trigger.  Typically
// that tuning is done by running the system with the digitizations written out (not reduced), and the trigger delay read
// offset is varied until the expected pulse is seen.  Then, when a trigger arrives, a set of typically 16 digitizations is
// read from the FIFO at the system clock rate (100 MHz) beginning with the starting read address.
//		The 16 digitizations are stored in one of 4 memory buffers, according to the 2-bit tag included in the trigger received.
// This implements buffering of up to 4 events at the front end, as is done in the Tracker ASIC, in order to smooth out poisson
// fluctuations in the trigger arrival times.  In fact, a trigger could arrive while the state machine is busy moving the data
// of the previous trigger from the FIFO to a memory buffer.  Therefore, the triggers themselves have to be buffered until the
// state machine is ready, including buffering of the starting read address for each trigger.  
//		The data for a given event sit in one of the four buffers until a read command is received.  Then, if the mode is set
// not to do data reduction, the samples are read from the appropriate buffer and shifted out in a serial data stream, in a
// format documented elsewhere, including header information and a 6-bit CRC trailer.  Since another read command could arrive
// while the output state machine is busy, the read commands have to be queued.
//		If data reduction is requested (the normal mode), then the output state machine does the reduction as it reads samples
// from the memory buffer.  So far the algorithm is very simple.  The first sample is taken to be the pedestal, and it is subtracted
// from all of the remaining samples before they are added up to give the reduced result.  Both the pedestal and that summed
// pulse height are written out.  Only 8 bits are allocated to the pedestal, so if it tends to be larger than that, a constant
// offset can be subtracted from it before writing it out.  16 bits are allocated to the summed pulse height.  Adding 16 samples,
// in principle the sum should be 18 bits.  But a normal pulse should have at most a couple of samples near their maximum, so 
// the sum should not exceed the 16 bits.  
//		Finally, there is a state machine to receive commands from the event builder and send configuration and monitoring
// data back to the event builder.  Many of the commands are used to set up the configuration of the algorithm described here.
//
//Parameters for configuring the DAQ:
reg [7:0] ReadOffset;   	// Number of clocks between write and read pointers in the pipeline
reg [3:0] NSamples;			// Number of samples minus 1 to extract from the pipeline for each trigger
							// Comparator threshold
							
reg [7:0] ReadAddr0;		// Starting read address for the FIFO
reg [7:0] ReadAddr;			// Actual read address
wire [7:0] WriteAddr;		// Write address for the FIFO

wire [14:0] doa0, doa1, doa2;			//Output ports from the first port of the block RAM.  These are not used.
reg ReadEnable;							//Signal to enable readout from the dual-ported FIFO block RAM
wire [14:0] DigOut0, DigOut1, DigOut2;	//Output ports on the 2nd port of the dual-ported FIFO block RAM
reg signed [13:0] PedOffset0;  			//Offset to shift the range for the pedestal written out with reduced data.
reg signed [13:0] PedOffset1;
reg signed [13:0] PedOffset2;
wire signed [13:0] DigOut0prt, DigOut1prt, DigOut2prt;   //Signed 14-bit digitizations 
assign DigOut0prt = DigOut0[13:0];
assign DigOut1prt = DigOut1[13:0];
assign DigOut2prt = DigOut2[13:0];

reg [44:0] DigiStore0[0:15];  	//Eight local event buffers in the FPGA distributed RAM
reg [44:0] DigiStore1[0:15];
reg [44:0] DigiStore2[0:15];
reg [44:0] DigiStore3[0:15];
reg [44:0] DigiStore4[0:15];  	
reg [44:0] DigiStore5[0:15];
reg [44:0] DigiStore6[0:15];
reg [44:0] DigiStore7[0:15];
reg [7:0] BufOcc;				//Set for each buffer if full
reg Reset;						//Reset signal initiated by command
reg [31:0] NRead, NSent;		//Monitoring counters for read commands received and events sent out
reg [3:0] numSamples;			//Number of samples to add together when reducing data
reg [3:0] mxSmpl;				//Maximum sample number in which to look for a peak
reg [1:0] NChWr;				//Number of channels-1 to write out for reduced data

reg OffsetSign; 
reg PedsOut;
reg [2:0] PedOffMsk;			//3-bit mask to determine which channels get affected by loading a pedestal offset
wire [4:0] NSampleP1;			//The number of samples per pulse plus 1
assign NSampleP1 = NSamples + 1;
reg [3:0] pedLoadChoice;

//Trigger monitor, just to counter trigger pulses from each of the 3 discriminators
wire [31:0] Ntrig0, Ntrig1, Ntrig2, NTReq;
TrgMon TrgMon0(Clock,Reset,TReqComp[0],Ntrig0);
TrgMon TrgMon1(Clock,Reset,TReqComp[1],Ntrig1);
TrgMon TrgMon2(Clock,Reset,TReqComp[2],Ntrig2);

reg [3:0] TrgLength;
wire [2:0] TrgStretch;
EnrgTrgStrtch Stretch0(TrgStretch[0], TReqComp[0], TrgLength, Clock, Reset);
EnrgTrgStrtch Stretch1(TrgStretch[1], TReqComp[1], TrgLength, Clock, Reset);
EnrgTrgStrtch Stretch2(TrgStretch[2], TReqComp[2], TrgLength, Clock, Reset);

//Trigger logic:
always @ (TrgType or TrgMask or TrgStretch) begin
	if (!TrgType) begin     //AND logic
		TReqOut = (TrgStretch[0] | !TrgMask[0]) & (TrgStretch[1] | !TrgMask[1]) & (TrgStretch[2] | !TrgMask[2]);
	end else begin          //OR logic
		TReqOut = (TrgStretch[0] & TrgMask[0]) | (TrgStretch[1] & TrgMask[1]) | (TrgStretch[2] & TrgMask[2]);
	end
end
TrgMon TrgMonAll(Clock,Reset,TReqOut,NTReq);

//always @ (posedge TReqOut) $display("%g\t EnergyDAQ:  sending TReq out.",$time);

//Each time there is a TReqOut signal, output a 4-bit string.  The first bit is 1, and the other 3 are the
//status of the 3 stretched trigger signals

parameter [3:0] WaitTb=4'b0001;     // Wait for a trigger signal
parameter [3:0] AcOrTb=4'b0010;     // Accumulate an OR signal over a fixed number of clock cycles
parameter [3:0] WritTb=4'b0100;     // Shift out the 4-bit trigger word
parameter [3:0] DlayTb=4'b1000;     // Wait for the trigger to go low

reg [3:0] StateTb, NextStateTb;
reg [3:0] TrgPat;
reg [1:0] CntTb;
reg [3:0] CntLnTb;

always @ (StateTb or TReqOut or CntTb or CntLnTb or TrgLength or TrgPat) begin
    case (StateTb)
	    WaitTb: begin
		            if (TReqOut) NextStateTb = AcOrTb;
					else NextStateTb = WaitTb;
					TReqOut4 = 1'b0;
		        end
		AcOrTb: begin
		            if (CntLnTb >= TrgLength) NextStateTb = WritTb;
					else NextStateTb = AcOrTb;
					TReqOut4 = 1'b0;
		        end
		WritTb: begin
		            if (CntTb == 3) NextStateTb = DlayTb;
		            else NextStateTb = WritTb;
					TReqOut4 = TrgPat[3];
		        end
		DlayTb: begin
		            if (TReqOut) NextStateTb = DlayTb;
					else NextStateTb = WaitTb;
					TReqOut4 = 1'b0;
		        end
		default: begin
		            NextStateTb = WaitTb;
					TReqOut4 = 1'b0;
		         end
	endcase
end

always @ (posedge Clock) begin
    if (Reset) begin
	    StateTb <= WaitTb;
	end else begin
	    StateTb <= NextStateTb;
		if (StateTb != WaitTb || TReqOut) $display("%g\t EnergyDAQ %d: StateTb=%b, TReqOut=%b, TrgStretch=%b, TrgPat=%b, TReqOut4=%b, CntLnTb=%d, CntTb=%d",$time,Address,StateTb,TReqOut,TrgStretch,TrgPat,TReqOut4,CntLnTb,CntTb);
		case (StateTb)
		    WaitTb: begin
			            if (TReqOut) TrgPat <= {1'b1,TrgStretch};
						CntTb <= 0;
						CntLnTb <= 1;
			        end
			AcOrTb: begin
			            TrgPat[2:0] <= (TrgPat[2:0] | TrgStretch);
			            CntLnTb <= CntLnTb + 1;
			        end
		    WritTb: begin
			            CntTb <= CntTb + 1;
			            TrgPat <= {TrgPat[2:0],1'b0};
					end
		endcase
	end
end


//Command Interpreter State Machine:
parameter [7:0] WaitCM=8'b00000001;		//Wait for a start bit
parameter [7:0] CmdBCM=8'b00000010;		//Read in the command byte
parameter [7:0] DatBCM=8'b00000100;		//Read in the data byte
parameter [7:0] DecoCM=8'b00001000;		//Act on the command
parameter [7:0] ReadCM=8'b00010000;		//Read command action
parameter [7:0] RgLdCM=8'b00100000;		//Load the output register
parameter [7:0] RegOCM=8'b01000000;		//Output a 78 bit register
parameter [7:0] RsetCM=8'b10000000;		//Reset the DAQ
reg [7:0] StateCM, NextStateCM;
reg [6:0] Cnt1CM;
reg [3:0] Cnt2CM;
reg [7:0] Command;
reg [7:0] DatField;
reg Prty, PrtyError;
reg [4:0] NPause;
reg [77:0] RegOut;
reg [2:0] ReadTag;
reg ReadPulse;
reg [1:0] TrgCnt;
reg DoutReg;
reg Reduce;
reg [7:0] BufClr;		//Signal to clear the buffer busy signal

wire signed [7:0] SgnDatField;    //Sometimes the input data field needs to be signed
assign SgnDatField = DatField;

always @ (StateCM or CmdIn or Cnt1CM or Cnt2CM or Command or ThisBoard or RegOut) begin
	case (StateCM) 
		WaitCM:	begin
					DoutReg = 1'b0;
					if (CmdIn == 1'b1) NextStateCM = CmdBCM;
					else NextStateCM = WaitCM;
				end
		CmdBCM:	begin
					DoutReg = 1'b0;
					if (Cnt1CM == 7) NextStateCM = DatBCM;
					else NextStateCM = CmdBCM;
				end
		DatBCM:	begin
					DoutReg = 1'b0;
					if (Cnt2CM == 7) NextStateCM = DecoCM;
					else NextStateCM = DatBCM;
				end
		DecoCM:	begin
					DoutReg = 1'b0;
					case (Command[3:0])
						4'h0:	NextStateCM = RegOCM;
						4'h5: NextStateCM = ReadCM;
						4'h6: NextStateCM = RgLdCM;
						4'h7:	NextStateCM = RsetCM;
						default:	NextStateCM = WaitCM;
					endcase
					end
		ReadCM:	begin
						DoutReg = 1'b0;
						NextStateCM = WaitCM;
					end
		RgLdCM:	begin
						DoutReg = 1'b0;
						NextStateCM = RegOCM;
					end
		RegOCM:	begin
					if (ThisBoard) DoutReg = RegOut[77];
					else DoutReg = 1'b0;
					if (Cnt1CM == 77) NextStateCM = WaitCM;
					else NextStateCM = RegOCM;
				end
		RsetCM:	begin
					DoutReg = 1'b0;
					if (Cnt1CM == 9) NextStateCM = WaitCM;   //Hold Reset high for 10 clock cycles in case the digitizer clock is very slow.
					else NextStateCM = RsetCM;
				end
		default: begin
					DoutReg = 1'b0;
					NextStateCM = WaitCM;
				 end
	endcase
end

assign ThisBoard = ((Command[7:4]==Address) || (Command[7:4]==4'hf));

reg ResetCF, ResetLast;
always @ (posedge Clock) begin
	ResetLast <= Reset;			//To stretch out the reset pulse for the longer-period clkdigi (only important for simulation)
	if (Reset) begin            //This soft reset should not mess up the configuration
		NPrtyError <= 0;
		NRead <= 0;
		Reset <= 1'b0;
	end else if (ResetCF) begin   
	    TrgType <= 1'b1;
		TrgMask <= 3'b111;
		Reset <= 1'b1;
		OffsetSign <= 1'b0;
		PedOffMsk <= 3'b111;
		NSamples <= 4'hf;
		Reduce <= 1'b0;
		numSamples <= 4'h4;
		mxSmpl <= 4'h8;
		NChWr <= 2'h3;
		PedOffset0 <= 0;
		PedOffset1 <= 0;
		PedOffset2 <= 0;
		ClkGate <= 1'b0;
		NPause <= 0;
		ResetCF <= 1'b0;
		TrgLength <= 4'd4;
		PedsOut <= 1'b1;
		ClkSelect <= 1'b0;
		FixedPed0 <= 0;
		FixedPed1 <= 0;
		FixedPed2 <= 0;
	end else if (ResetHard) begin
		StateCM <= WaitCM;
		ResetCF <= 1'b1;
		ResetLast <= 1'b0;
	end else begin
		StateCM <= NextStateCM;
		case (StateCM)
			WaitCM:	begin
						Cnt1CM <= 0;
						Cnt2CM <= 0;
						Prty <= 1'b0;
						ReadPulse <= 1'b0;
						PrtyError <= 1'b0;
					end
			CmdBCM:	begin
						Cnt1CM <= Cnt1CM + 1;
						Command <= {Command[6:0],CmdIn};
						Prty <= Prty^CmdIn;
					end
			DatBCM:	begin
						Cnt2CM <= Cnt2CM + 1;
						DatField <= {DatField[6:0],CmdIn};
						Prty <= Prty^CmdIn;
					end
			DecoCM:	begin
						if (Prty^CmdIn != 1'b0) begin
							PrtyError <= 1'b1;
							NPrtyError <= NPrtyError + 1;
							$display("%g\t EnergyDAQ %d:  parity error encountered!  Command=%b, DatField=%b",$time,Address,Command,DatField);
						end
						Cnt1CM <= 0;
						$display("%g\t EnergyDAQ DecoCM:  received command %b, data %b",$time,Command,DatField);
						case (Command[3:0])
							4'h0:	begin
										RegOut <= {2'b11,Address,6'b110001,Version,58'b1111000010111110000101111100001011111000010111110000101111};
										$display("%g\t EnergDAQ %d:  dumping test string %b",$time,Address,ThisBoard);
									end
							4'h1:	begin
										if (ThisBoard) ReadOffset <= DatField;
										$display("%g\t EnergyDAQ %d:  read offset set to %d %b",$time,Address,DatField,ThisBoard);
									end
							4'h2:	if (ThisBoard) begin
										NSamples <= DatField[4:0]-1;
										$display("%g\t EnergyDAQ %d:  number samples set to %d %b",$time,Address,DatField[4:0],ThisBoard);
									end
							4'h3:	if (ThisBoard) begin
										Reduce <= DatField[0];
										$display("%g\t EnergyDAQ %d:  set data reduction control to %b %b",$time,Address,DatField[0],ThisBoard);
									end
							4'h4: 	if (ThisBoard) begin
										if (PedOffMsk[0]) PedOffset0 <= SgnDatField*16;
										if (PedOffMsk[1]) PedOffset1 <= SgnDatField*16;
										if (PedOffMsk[2]) PedOffset2 <= SgnDatField*16;
									end
							4'h7: begin
										if (ThisBoard) Reset <= 1'b1;
										$display("%g\t EnergyDAQ %d:  reset command received for Reset=%b",$time,Address,ThisBoard);
									end
							4'h8:	begin
										if (ThisBoard) TrgMask <= DatField[2:0];
										$display("%g\t EnergyDAQ %d:  trigger mask set to %b %b",$time,Address,DatField[2:0],ThisBoard);
									end
							4'h9:	begin
										if (ThisBoard) TrgType <= DatField[0];
										$display("%g\t EnergyDAQ %d:  trigger type set to %b %b",$time,Address,DatField[0],ThisBoard);
									end
							4'ha:	begin
										if (ThisBoard) begin
											numSamples <= DatField[3:0];
											mxSmpl <= DatField[7:4];
											$display("%g\t EnergyDAQ:  number of samples to add set to %d",$time,DatField[3:0]);
											$display("%g\t EnergyDAQ:  limit for peak search set to %d",$time,DatField[3:0]);
										end
									end
							4'hb:	begin
										if (ThisBoard) begin
											$display("%g\t EnergyDAQ %d:  resetting the configuration.",$time,Address);
											ResetCF <= 1'b1;
										end
									end
							4'hc:	begin
										if (ThisBoard) NPause <= DatField[4:0];
										$display("%g\t EnergyDAQ %d:  read delay set to %b %b",$time,Address,DatField[4:0],ThisBoard);
									end
							4'hd:	begin
										if (ThisBoard) begin
											case (pedLoadChoice)
												4'b0000:	FixedPed0[7:0] <= DatField;
												4'b0001:	FixedPed1[7:0] <= DatField;
												4'b0010:	FixedPed2[7:0] <= DatField;
												4'b0100:	FixedPed0[13:8] <= DatField[5:0];
												4'b0101:	FixedPed1[13:8] <= DatField[5:0];
												4'b0110:	FixedPed2[13:8] <= DatField[5:0];
											endcase
											$display("%g\t EnergyDAQ %d:  setting pedestal choice %b to %b",$time,Address,pedLoadChoice,DatField);
										end
									end
							4'he: 	begin
										if (ThisBoard) begin
											ClkGate <= DatField[0];
											$display("%g\t EnergyDAQ %d:  set digitizer clock gate=%b",$time,Address,DatField[0]);
										end
									end
							4'hf: 	begin
										case (DatField[7:4])   //8-bit commands with 4-bit data fields
											4'h0:	begin
														if (ThisBoard) begin
															OffsetSign <= DatField[0];
															$display("%g\t EnergyDAQ %d:  set trigger offset sign=%b",$time,Address,DatField[0]);
														end
													end
											4'h1:	begin
														if (ThisBoard) begin
															PedsOut <= DatField[0];
															$display("%g\t EnergyDAQ %d:  set pedestal output flag to %b",$time,Address,DatField[0]);
														end
													end
											4'h2:	begin
														if (ThisBoard) begin
															if (DatField[1:0] != 2'b00) NChWr <= DatField[1:0];
															$display("%g\t EnergyDAQ %d:  number of channels to write set to %d",$time,Address,DatField[1:0]);
														end
													end
											4'h3: 	begin 
														if (ThisBoard) begin
															PedOffMsk <= DatField[2:0];
															$display("%g\t EnergyDAQ %d:  pedestal offset mask set to %b",$time,Address,DatField[2:0]);
														end													
													end
											4'h4:	begin
														if (ThisBoard) begin
															TrgLength <= DatField[3:0];
															$display("%g\t EnergyDAQ %d: trigger length set to %d",$time,Address,DatField[3:0]);
														end
													end
											4'h5:	begin
														if (ThisBoard) begin
															ClkSelect <= DatField[0];
															$display("%g\t EnergyDAQ %d: digitizer clock source set to %d",$time,Address,DatField[0]);
														end
													end
											4'h6:	begin
														if (ThisBoard) begin
															pedLoadChoice <= DatField[3:0];
															$display("%g\t EnergyDAQ %d: pedestal load choice set to %b",$time,Address,DatField[3:0]);
														end
													end
										endcase
									end
						endcase
					end
		ReadCM:	begin
						ReadTag <= DatField[2:0];       //Read event command
						if (ThisBoard) begin
							NRead <= NRead + 1;
							ReadPulse <= 1'b1;
						end
						$display("%g\t EnergyDAQ %d:  read event command received for tag %b ReadPulse %b",$time,Address,DatField[2:0],ThisBoard);
					end
		RgLdCM:	begin
							case (DatField[3:0])    //Commands that send configuration or monitoring information back to the event builder
								4'h0: 	begin
											RegOut <= {2'b11,Address,6'b110001,NTack,NTReq,2'b11};
											$display("%g\t EnergyDAQ %d: NTack=%d, NTReq=%d.",$time,Address,NTack,NTReq);
										end
								4'h1: 	begin
											RegOut <= {2'b11,Address,6'b110001,NSent,NRead,2'b11};
											$display("%g\t EnergyDAQ %d:  NSent=%d, NRead=%d.",$time,Address,NSent,NRead);
										end
								4'h2: 	begin
											RegOut <= {2'b11,Address,6'b110001,NPrtyError,NTimeOut,NRdyError,NTrgError,2'b11};
											$display("%g\t EnergyDAQ %d:  NPrtyError=%d, NTimeOut=%d, NRdyError=%d, NTrgError=%d.",$time,Address,NPrtyError,NTimeOut,NRdyError,NTrgError);
										end
								4'h3:	begin  
											RegOut <= {2'b11,Address,6'b110001,7'b0000000,OffsetSign,TrgType,5'b00000,NChWr,1'b0,numSamples,1'b0,mxSmpl,PedsOut,TrgLength,ClkSelect,30'd0,2'b11};
											$display("%g\t EnergyDAQ %d:  dumping OffsetSign=%b and NChWr=%d",$time,Address,OffsetSign,NChWr);
											$display("%g\t EnergyDAQ %d:  dumping trigger type %b %b",$time,Address,TrgType,ThisBoard);
										end
								4'h4:	begin
											RegOut <= {2'b11,Address,6'b110001,Nsequence,NTrgPrty,Ntrig0,2'b11};
											$display("%g\t EnergyDAQ %d: Number of trigger tags out of sequence=%d",$time,Address,Nsequence);
											$display("%g\t EnergyDAQ %d: Number of trigger parity errors=%d",$time,Address,NTrgPrty);
											$display("%g\t EnergyDAQ %d: Ntrig0=%d, Ntrig1=%d, Ntrig2=%d",$time,Address,Ntrig0,Ntrig1,Ntrig2);
										end
								4'h5:	begin
											RegOut <= {2'b11,Address,6'b110001,Ntrig1,Ntrig2,2'b11};
											$display("%g\t EnergyDAQ %d: Number of trigger parity errors=%d",$time,Address,NTrgPrty);
											$display("%g\t EnergyDAQ %d: Ntrig0=%d, Ntrig1=%d, Ntrig2=%d",$time,Address,Ntrig0,Ntrig1,Ntrig2);
										end
								4'h6:	begin
											RegOut <= {2'b11,Address,6'b110001,NOcc1,NOcc2,NOcc3,NOcc4,2'b11};
											$display("%g\t EnergyDAQ %d: Buffer occupancies = %d %d %d %d.",$time,Address,NOcc1,NOcc2,NOcc3,NOcc4);
											$display("%g\t EnergyDAQ %d: Buffer occupancies = %d %d %d %d.",$time,Address,NOcc5,NOcc6,NOcc7,NOcc8);
										end
								4'h7:	begin
											RegOut <= {2'b11,Address,6'b110001,NOcc5,NOcc6,NOcc7,NOcc8,2'b11};
										end
								4'h8:	begin
//											RegOut <= {2'b11,Address,6'b110001,NRdSequence,NoverWrite,DoutTime,2'b11};
											RegOut <= {2'b11,Address,6'b110001,NRdSequence,16'b0,DoutTime,2'b11};
											$display("%g\t EnergyDAQ %d:  Number of read tags out of sequence=%d",$time,Address,NRdSequence);
											$display("%g\t EnergyDAQ %d:  Number of clock cycles with output active=%d",$time,Address,DoutTime);
//											$display("%g\t EnergyDAQ %d:  Number of interferences between FIFO read and write pointers=%d",$time,Address,NoverWrite);
										end
								4'ha:	begin    //SDAin and SCLin put in here just to keep them from getting optimized away
											RegOut <= {2'b11,Address,2'b11,SDAin,SCLin,2'b01,ReadOffset,56'd0,2'b11};
											$display("%g\t EnergyDAQ %d:  dumping read offset %b %b",$time,Address,ReadOffset,ThisBoard);
										end
								4'hb:	begin
											RegOut <= {2'b11,Address,6'b110001,3'b000,NSampleP1,3'b000,NPause,5'b00000,TrgMask,40'd0,2'b11};
											$display("%g\t EnergyDAQ %d:  Dumping number of samples: %b %b",$time,Address,NSampleP1,ThisBoard);
											$display("%g\t EnergyDAQ %d:  Dumping read delay %b %b",$time,Address,NPause,ThisBoard);
											$display("%g\t EnergyDAQ %d:  dumping trigger mask %b %b",$time,Address,TrgMask,ThisBoard);
										end
								4'hc:	begin
											RegOut <= {2'b11,Address,6'b110001,NStateZero,State,49'd0,2'b11};
										end
								4'hd:	begin
											RegOut <= {2'b11,Address,6'b110001,NTrgBfErr,NQerr,Ntrigs,BufOcc,tagSave,occSave,NTrgOut,stateSave,2'd0,2'b11};	
											$display("%g\t EnergyDAQ %d: NTrgBfErr=%d NQerr=%d BufOcc=%b tagSave=%d occSave=%b NTrgOut=%d stateSave=%b",$time,Address,NTrgBfErr,NQerr,BufOcc,tagSave,occSave,NTrgOut,stateSave);
										end					
								4'he: 	begin
											RegOut <= {2'b11,Address,6'b110001,FixedPed0,FixedPed1,FixedPed2,22'd0,2'b11};
											$display("%g\t EnergyDAQ %d:  Dumping calibrated pedestals %d %d %d",$time,Address,FixedPed0,FixedPed1,FixedPed2);													
										end		
								4'hf:	begin
											RegOut <= {2'b11,Address,6'b110001,PedOffset0,PedOffset1,PedOffset2,22'd0,2'b11};
											$display("%g\t EnergyDAQ %d: Dumping pedestal offsets %d %d %d",$time,Address,PedOffset0,PedOffset1,PedOffset2);
										end
								default: RegOut <= {2'b11,Address,6'b110001,63'b0,2'b11};
							endcase
						end
			RegOCM:	begin
//						if (Cnt1CM == 0) $display("%g\t EnergyDAQ: Shift out register %b.",$time,RegOut);
						Cnt1CM <= Cnt1CM + 1;
						RegOut <= {RegOut[76:0],1'b0};
					end
			RsetCM:	begin
						Cnt1CM <= Cnt1CM + 1;
					end
		endcase
	end
end

//Increment the read address every cycle of the digitizer clock (65 MHz max)
always @ (posedge ClkDigi) begin
	if (Reset | ResetLast) begin
		ReadAddr0 <= 0;
	end else begin
		ReadAddr0 <= ReadAddr0 + 1;
	end
end

//Calculate the address for writing digitizations into the FIFO
//Spacing between write and read must correspond to the trigger latency
assign WriteAddr = (OffsetSign) ? ReadAddr0 - ReadOffset : ReadAddr0 + ReadOffset;     

//Decode the incoming trigger signal.  TrigPulse is 1 clock long, while TrigTag contains the two bit buffer identifier.
//The same program counts parity errors.
wire [1:0] TrigTag;
wire [15:0] NTrgPrty;
TriggerDecoder TriggerDecoderEnrg(Clock,Reset,TrgIn,TrigPulse,TrigTag,NTrgPrty);

parameter WriteEnable = 1'b1;  //Writing into the pipeline happens every cycle of the digitizer clock

//Instantiate block RAM for each of the three digitizer outputs.  These are the 256 clock deep FIFOs.
DigiBufRAM DigiBuf0 (ClkDigi, Clock, WriteEnable, ReadEnable, WriteEnable, WriteAddr, ReadAddr, Dig0, doa0, DigOut0);
DigiBufRAM DigiBuf1 (ClkDigi, Clock, WriteEnable, ReadEnable, WriteEnable, WriteAddr, ReadAddr, Dig1, doa1, DigOut1);
DigiBufRAM DigiBuf2 (ClkDigi, Clock, WriteEnable, ReadEnable, WriteEnable, WriteAddr, ReadAddr, Dig2, doa2, DigOut2);

//State machine to buffer the incoming triggers
//This is needed because it takes some time to process a trigger and read the samples from the digitization FIFO
parameter [3:0] WaitBf=4'b0001; 	//Wait for a trigger.  If no new trigger, see if there is one pending and issue it.
parameter [3:0] StorBf=4'b0010; 	//Store the trigger in the FIFO
parameter [3:0] Str2Bf=4'b0100;		//Another clock cycle for storing the FIFO start address
parameter [3:0] IssuBf=4'b1000;  	//Issue a trigger

reg [3:0] StateBf, NextStateBf;
reg [3:0] Ntrigs,Twrite,Tread;
reg [15:0] Tg0, Tg1;              //FIFO for the trigger tags
reg [7:0] StartAddress;
reg TrgPl;
reg [2:0] TagPl;				//Trigger tag.  The MSB is generated internally.

always @ (StateBf or TrigPulse or TrigTag or Ntrigs or NextState) begin
	case (StateBf)
		WaitBf:	begin
					if (TrigPulse) begin
						NextStateBf = StorBf;
						FIFOreadEnable = 1'b0;
					end else if (Ntrigs > 0 && (NextState==Wait)) begin
						NextStateBf = IssuBf;
						FIFOreadEnable = 1'b1;
					end else begin
						NextStateBf = WaitBf;
						FIFOreadEnable = 1'b0;
					end
					FIFOwriteEnable = 1'b0;
				end
		StorBf: begin
					NextStateBf = Str2Bf;
					FIFOwriteEnable = 1'b1;
					FIFOreadEnable = 1'b0;
				end
		Str2Bf: begin			//The read address is written into the FIFO with the slower clock, so we have to keep the write enable high here for 2 clock cycles.
					NextStateBf = WaitBf;
					FIFOwriteEnable = 1'b1;
					FIFOreadEnable = 1'b0;
				end
		IssuBf: begin
					if (TrigPulse) NextStateBf = StorBf;
					else NextStateBf = WaitBf;
					FIFOwriteEnable = 1'b0;
					FIFOreadEnable = 1'b0;
				end
		default:	begin
						NextStateBf = WaitBf;
						FIFOwriteEnable = 1'b0;
						FIFOreadEnable = 1'b0;
					end
	endcase
end

reg [15:0] NOcc1, NOcc2, NOcc3, NOcc4, NOcc5, NOcc6, NOcc7, NOcc8;
reg [1:0] LastTagP1;
reg [15:0] Nsequence;
reg [3:0] NBufOcc;
reg Tag3;

//Dual ported RAM to buffer the read start address while waiting for the state machine to be ready
reg FIFOwriteEnable, FIFOreadEnable;
wire [7:0] outFIFO, doaFIFO;
ReadFifoRAM ReadFIFO(ClkDigi, Clock, FIFOwriteEnable, FIFOreadEnable, FIFOwriteEnable, Twrite, Tread, ReadAddr0, doaFIFO, outFIFO);

reg [7:0] NTrgBfErr;
reg [15:0] NTrgOut;
reg [31:0] NTack;
always @ (posedge Clock) begin
	if (Reset) begin
		NTack <= 0;
		StateBf <= WaitBf;
		Twrite <= 0;
		Tread <= 0;
		Ntrigs <= 0;
		TrgPl <= 1'b0;
		NOcc1 <= 0;
		NOcc2 <= 0;
		NOcc3 <= 0;
		NOcc4 <= 0;
		NOcc5 <= 0;
		NOcc6 <= 0;
		NOcc7 <= 0;
		NOcc8 <= 0;
		LastTagP1 <= 0;
		Nsequence <= 0;
		Tag3 <= 1'b0;
		TrgError <= 1'b0;
		NTrgBfErr <= 0;
		NTrgOut <= 0;
	end else begin
		StateBf <= NextStateBf;
		NBufOcc <= BufOcc[0] + BufOcc[1] + BufOcc[2] + BufOcc[3] + BufOcc[4] + BufOcc[5] + BufOcc[6] + BufOcc[7];
		if (TrgError) NTrgBfErr <= NTrgBfErr + 1;
		if (TrigPulse) begin
			NTack <= NTack + 1;
			case (NBufOcc)
				4'b0001:	NOcc1 <= NOcc1 + 1;
				4'b0010:	NOcc2 <= NOcc2 + 1;
				4'b0011:	NOcc3 <= NOcc3 + 1;
				4'b0100:	NOcc4 <= NOcc4 + 1;
				4'b0101:	NOcc5 <= NOcc5 + 1;
				4'b0110:	NOcc6 <= NOcc6 + 1;
				4'b0111:	NOcc7 <= NOcc7 + 1;
				4'b1000:	NOcc8 <= NOcc8 + 1;
			endcase
		end
		case (StateBf)
			WaitBf:	begin
						TrgError <= 1'b0;
						if (TrigPulse) $display("%g\t EnergyDAQ %d: trigger pulse received for tag=%d",$time,Address,TrigTag);
						TrgPl <= 1'b0;
					end
			StorBf:	begin           //This introduces a 2-clock delay in the trigger 
						TrgPl <= 1'b0;
						Tg0[Twrite] <= TrigTag[0];
						Tg1[Twrite] <= TrigTag[1];
						Ntrigs <= Ntrigs + 1;
						$display("%g\t EnergyDAQ %d:  entering a trigger into the FIFO with tag %h, Addr=%d.  N=%d, Twrite=%d, Tread=%d, Tg1=%b, Tg0=%b",$time,Address,TrigTag,ReadAddr0,Ntrigs,Twrite,Tread,Tg1,Tg0);
						LastTagP1 <= TrigTag + 1;
						if (LastTagP1 != TrigTag) begin
							$display("%g\t EnergyDAQ %d: trigger tag out of sequence, %d %d.",$time,Address,LastTagP1,TrigTag);
							Nsequence <= Nsequence + 1;
						end
						if (Ntrigs == 4'hf) begin
							TrgError <= 1'b1;			//It seems that it should be impossible to get here with a 16-trigger FIFO!
							$display("%g\t EnergyDAQ %d error 5:  Overflow of the trigger pulse buffer.  Tag=%h",$time,Address,TrigTag);
						end
						if (TrigPulse) begin    //Triggers should never be so close together that this happens, I think!
							TrgError <= 1'b1;
							$display("%g\t EnergyDAQ %d error 4:  Trigger pulse for tag %h received while in state StorBf.",$time,Address,TrigTag);
						end
					end
			Str2Bf:	begin   
						Twrite <= Twrite + 1;
						if (TrigPulse) begin    //Triggers should never be so close together that this happens, I think!
							TrgError <= 1'b1;
							$display("%g\t EnergyDAQ %d error 4:  Trigger pulse for tag %h received while in state Str2Bf.",$time,Address,TrigTag);
						end
					end
			IssuBf:	begin
						if (Tg1[Tread] == 1'b1 && Tg0[Tread] == 1'b1) Tag3 <= ~Tag3;
						NTrgOut <= NTrgOut + 1;
						TrgPl <= 1'b1;
						TagPl <= {Tag3,Tg1[Tread],Tg0[Tread]};
						Tread <= Tread + 1;
						Ntrigs <= Ntrigs - 1;
						if (Ntrigs == 0) begin
							TrgError <= 1'b1;			
							$display("%g\t EnergyDAQ %d error 6:  Underflow of the trigger pulse buffer.  Tag=%h",$time,Address,TrigTag);
						end
						StartAddress <= outFIFO;
						$display("%g\t EnergyDAQ %d:  issuing a trigger from the FIFO for tag %b, Strt Addr=%d, BufOcc=%b",$time,Address,{Tag3,Tg1[Tread],Tg0[Tread]},outFIFO,BufOcc);
					end
		endcase
	end
end

//State machine to read the samples from the pipeline RAM when a trigger arrives
//We also apply the data reduction algorithm here
parameter [6:0] Wait=7'b0000001;			//Wait for a trigger
parameter [6:0] Load=7'b0000010;			//Load the starting read address
parameter [6:0] Frst=7'b0000100;			//Fetch the first samples from memory
parameter [6:0] Stor=7'b0001000;			//Read samples from RAM and into local registers
parameter [6:0] Paus=7'b0010000;			//Wait for the write pointer to move ahead
parameter [6:0] Last=7'b0100000;			//Store away the last sample
parameter [6:0] Redu=7'b1000000;			//Store the reduced data

reg [6:0] State, NextState, stateSave;
reg [3:0] Cntr;
reg [4:0] CntrP;
wire [3:0] StorAddr;
reg DoutDigi;
reg [3:0] nAdded0,nAdded1,nAdded2;					//Count the number of samples included in the pulse sum
reg signed [13:0] prevSmp0,prevSmp1,prevSmp2;
reg signed [13:0] maxPH0,maxPH1,maxPH2;
reg signed [15:0] Sample0,Sample1,Sample2;			  //Summed pulse area for each channel
reg signed [13:0] FixedPed0,FixedPed1,FixedPed2;	  //Constant calibrated pedestal values loaded by the user
reg signed [13:0] Pedestal0, Pedestal1, Pedestal2;    //First sample of each channel per event
wire signed [7:0] PedOut0,PedOut1,PedOut2;			  //Reduced precision pedestal value, optionally written out
reg [23:0] Reduced0[0:7];	//Eight buffers to store reduced data for channel 0
reg [23:0] Reduced1[0:7];	//Eight buffers to store reduced data for channel 1
reg [23:0] Reduced2[0:7];	//Eight buffers to store reduced data for channel 2
reg [2:0] OTRsave[0:7];		//Eight buffers to store the overflow indicators for reduced data
assign PedOut0 = Pedestal0-PedOffset0;   //For reduced data, offset the pedestal written out so that it fits into 8 bits
assign PedOut1 = Pedestal1-PedOffset1;
assign PedOut2 = Pedestal2-PedOffset2;

always @ (State or TrgPl or Cntr or NSamples or CntrP or NPause or WriteAddr or ReadAddr) begin
	case (State)
		Wait: 	begin
					if (TrgPl) NextState = Load;
					else NextState = Wait;
					ReadEnable = 1'b0;
				end
		Load:	begin
					ReadEnable = 1'b0;
					if (NPause != 0) NextState = Paus;
					else NextState = Frst;
				end
		Paus:	begin
					ReadEnable = 1'b0;
					if (CntrP > NPause) begin
						NextState = Frst;
					end else begin
						NextState = Paus;
						// if (WriteAddr>=ReadAddr) begin
							// if ((WriteAddr - ReadAddr) > NPause) NextState = Frst;
							// else NextState = Paus;
						// end else begin
							// if ((WriteAddr + (256-ReadAddr)) > NPause) NextState = Frst;
							// else NextState = Paus;
						// end
					end
				end
		Frst:	begin
					ReadEnable = 1'b1;   //Enable the RAM output ports
					NextState = Stor;
				end
		Stor:	begin  
					ReadEnable = 1'b1;   //Enable the RAM output ports
					if (Cntr == NSamples) NextState = Last;
					else NextState = Stor;
				end
		Last:	begin
					ReadEnable = 1'b0;
					NextState = Redu;
				end
		Redu:	begin
					ReadEnable = 1'b0;
					NextState = Wait;
				end
		default: 	begin
						ReadEnable = 1'b0;
						NextState = Wait;
					end
	endcase
end

//reg [15:0] NoverWrite;
reg [2:0] OTR;
reg [7:0] NStateZero;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		NRdyError <= 0;
		BufOcc <= 0;
		NTrgError <= 0;
//		NoverWrite <= 0;
		NStateZero <= 0;
	end else begin
		State <= NextState;
		if (State != Last) begin
			if (BufClr[0]) BufOcc[0] <= 1'b0;
			if (BufClr[1]) BufOcc[1] <= 1'b0;
			if (BufClr[2]) BufOcc[2] <= 1'b0;
			if (BufClr[3]) BufOcc[3] <= 1'b0;
			if (BufClr[4]) BufOcc[4] <= 1'b0;
			if (BufClr[5]) BufOcc[5] <= 1'b0;
			if (BufClr[6]) BufOcc[6] <= 1'b0;
			if (BufClr[7]) BufOcc[7] <= 1'b0;
			if (BufClr != 0) $display("%g\t EnergyDAQ %d State %b: BufClr signal, BufClr=%b, BufOcc=%b.",$time,Address,State,BufClr,BufOcc);
		end
		if (TrgPl && State != Wait) begin   //This shouldn't happen if the trigger buffering above is working
			RdyError <= 1'b1;
			$display("%g\t EnergyDAQ error 1:  Trigger pulse received with tag %h while the trigger processor is busy.",$time,TagPl);
			NRdyError <= NRdyError + 1;
		end
		case (State)
			Wait: 	begin
						RdyError <= 1'b0;
						if (TrgPl && BufOcc[TagPl]) begin
							NTrgError <= NTrgError + 1;
							$display("%g\t EnergyDAQ %d state Wait:  Trigger pulse with tag %h received while the buffer is occupied. %b",$time,Address,TagPl,BufOcc);
						end
						if (TrgPl) $display("%g\t EnergyDAQ %d:  Trigger pulse received.  WriteAddr=%d, ReadAddr=%d",$time,Address,WriteAddr,StartAddress);
					end
			Load:	begin
						$display("%g\t EnergyDAQ %d:  begin loading digitizations into buffer %h, WriteAddr=%d, ReadAddr=%d",$time,Address,TagPl,WriteAddr,StartAddress);
						ReadAddr <= StartAddress;   //The starting address was saved when the trigger arrived.
						Cntr <= 0;
						CntrP <= 1;
					end
			Paus:	begin
						CntrP <= CntrP + 1;
						$display("%g\t EnergyDAQ %d state Paus: ReadAddr=%d WriteAddr=%d NPause=%d",$time,Address,ReadAddr,WriteAddr,NPause);
					end
			Frst:	begin
						Cntr <= Cntr + 1;
						ReadAddr <= ReadAddr + 1;
					end
			Stor:	begin
						ReadAddr <= ReadAddr + 1;
						// if (ReadAddr == WriteAddr) begin  //This test doesn't work because it sometimes happens while WriteAddr is changing & unstable
							// $display("%g\t EnergyDAQ %d Error: conflict of write and read pointers=%d.  OffsetSign=%d ReadOffset=%d NPause=%d",$time,Address,ReadAddr,OffsetSign,ReadOffset,NPause);
							// NoverWrite <= NoverWrite + 1;
						// end
						Cntr <= Cntr + 1;
						$display("%g\t EnergyDAQ %d: %d  storing in location %h samples %d %d %d.  ReadAddr=%d WriteAddr=%d",$time,Address,Cntr,TagPl,DigOut0prt,DigOut1prt,DigOut2prt,ReadAddr,WriteAddr);
						case (TagPl)
							3'b000:	DigiStore0[StorAddr] <= {DigOut0,DigOut1,DigOut2};   //StoreAddr = Cntr - 1
							3'b001:	DigiStore1[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b010:	DigiStore2[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b011:	DigiStore3[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b100:	DigiStore4[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b101:	DigiStore5[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b110:	DigiStore6[StorAddr] <= {DigOut0,DigOut1,DigOut2};
							3'b111:	DigiStore7[StorAddr] <= {DigOut0,DigOut1,DigOut2};
						endcase
						if (Cntr == 1) begin
							prevSmp0 <= DigOut0prt - FixedPed0;
							prevSmp1 <= DigOut1prt - FixedPed1;
							prevSmp2 <= DigOut2prt - FixedPed2;
							Sample0 <= DigOut0prt - FixedPed0;
							Sample1 <= DigOut1prt - FixedPed1;
							Sample2 <= DigOut2prt - FixedPed2;
							Pedestal0 <= DigOut0prt;
							Pedestal1 <= DigOut1prt;
							Pedestal2 <= DigOut2prt;
							OTR[0] <= DigOut0[14];
							OTR[1] <= DigOut1[14];
							OTR[2] <= DigOut2[14];
							maxPH0 <= DigOut0prt;
							maxPH1 <= DigOut1prt;
							maxPH2 <= DigOut2prt;
							nAdded0 <= 1;
							nAdded1 <= 1;
							nAdded2 <= 1;
						end else begin	
							prevSmp0 <= (DigOut0prt - FixedPed0);
							if (DigOut0prt>maxPH0 && Cntr < mxSmpl) begin
								maxPH0 <= DigOut0prt;
								Sample0 <= (DigOut0prt - FixedPed0) + prevSmp0;
								nAdded0 <= 2;
							end else begin
								if (nAdded0 < numSamples) begin
									Sample0 <= Sample0 + (DigOut0prt - FixedPed0);
									nAdded0 <= nAdded0 + 1;
								end
							end
							prevSmp1 <= (DigOut1prt - FixedPed1);
							if (DigOut1prt>maxPH1 && Cntr < mxSmpl) begin
								maxPH1 <= DigOut1prt;
								Sample1 <= (DigOut1prt - FixedPed1) + prevSmp1;
								nAdded1 <= 2;
							end else begin
								if (nAdded1 < numSamples) begin
									Sample1 <= Sample1 + (DigOut1prt - FixedPed1);
									nAdded1 <= nAdded1 + 1;
								end
							end
							prevSmp2 <= (DigOut2prt - FixedPed2);
							if (DigOut2prt>maxPH2 && Cntr < mxSmpl) begin
								maxPH2 <= DigOut2prt;
								Sample2 <= (DigOut2prt - FixedPed2) + prevSmp2;
								nAdded2 <= 2;
							end else begin
								if (nAdded2 < numSamples) begin
									Sample2 <= Sample2 + (DigOut2prt - FixedPed2);
									nAdded2 <= nAdded2 + 1;
								end
							end
							OTR[0] <= OTR[0] | DigOut0[14];
							OTR[1] <= OTR[1] | DigOut1[14];
							OTR[2] <= OTR[2] | DigOut2[14];
						end
					end
			Last: 	begin
						$display("%g\t EnergyDAQ %d:  stored %d samples in location %h: %d %d %d.",$time,Address,NSamples,TagPl,DigOut0,DigOut1,DigOut2);
						if (BufClr[0] && TagPl != 0) BufOcc[0] <= 1'b0;
						if (BufClr[1] && TagPl != 1) BufOcc[1] <= 1'b0;
						if (BufClr[2] && TagPl != 2) BufOcc[2] <= 1'b0;
						if (BufClr[3] && TagPl != 3) BufOcc[3] <= 1'b0;
						if (BufClr[4] && TagPl != 4) BufOcc[4] <= 1'b0;
						if (BufClr[5] && TagPl != 5) BufOcc[5] <= 1'b0;
						if (BufClr[6] && TagPl != 6) BufOcc[6] <= 1'b0;
						if (BufClr[7] && TagPl != 7) BufOcc[7] <= 1'b0;
						$display("%g\t EnergyDAQ %d State Last: Setting BufOcc for tag %d, BufOcc=%b, BufClr=%b.",$time,Address,TagPl,BufOcc,BufClr);
						//I'm assuming here that the last sample is never used for the pulse-height analysis.  Could easily change this, of course.
						OTR[0] <= OTR[0] | DigOut0[14];
						OTR[1] <= OTR[1] | DigOut1[14];
						OTR[2] <= OTR[2] | DigOut2[14];
						case (TagPl)
							3'b000:	begin DigiStore0[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[0] <= 1'b1; end
							3'b001:	begin DigiStore1[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[1] <= 1'b1; end
							3'b010:	begin DigiStore2[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[2] <= 1'b1; end
							3'b011:	begin DigiStore3[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[3] <= 1'b1; end
							3'b100:	begin DigiStore4[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[4] <= 1'b1; end
							3'b101:	begin DigiStore5[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[5] <= 1'b1; end
							3'b110:	begin DigiStore6[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[6] <= 1'b1; end
							3'b111:	begin DigiStore7[NSamples] <= {DigOut0,DigOut1,DigOut2}; BufOcc[7] <= 1'b1; end
						endcase
					end
			Redu:	begin
						$display("%g\t EnergyDAQ %d: Storing reduced info: (%d %d) (%d %d) (%d %d)",$time,Address,Sample0,PedOut0,Sample1,PedOut1,Sample2,PedOut2);
						Reduced0[TagPl] <= {PedOut0,Sample0};    //Store the reduced data and pedestals in an array
						Reduced1[TagPl] <= {PedOut1,Sample1};
						Reduced2[TagPl] <= {PedOut2,Sample2};
						OTRsave[TagPl] <= OTR;
					end
			default: begin
							NStateZero <= NStateZero + 1;
						end
		endcase
	end
end

assign StorAddr = Cntr - 1;

//Queue the read commands until the read state machine is ready for them
parameter [2:0] WaitQu = 3'b001;
parameter [2:0] StorQu = 3'b010;
parameter [2:0] IssuQu = 3'b100;
reg [2:0] StateQu, NextStateQu;
reg [2:0] NQueue, QWrite, QRead;
reg [2:0] TagQueue[0:7];
reg [2:0] ReadTagOut;
reg [2:0] ReadTagLastP1;
reg [15:0] NRdSequence;
reg ReadPulseOut;
reg [7:0] NQerr;

always @ (StateQu or ReadPulse or NextStateRd or NextStateR2 or NQueue) begin
	case (StateQu)
		WaitQu:	begin
					if (ReadPulse) begin
						NextStateQu = StorQu;
					end else if (NextStateRd == WaitRd && NextStateR2 == WaitR2 && NQueue > 0) begin
						NextStateQu = IssuQu;
					end else begin
						NextStateQu = WaitQu;
					end
				end
		StorQu:	begin
					NextStateQu = WaitQu;
				end
		IssuQu:	begin
					if (ReadPulse) NextStateQu = StorQu;
					else NextStateQu = WaitQu;
				end
		default: begin
					NextStateQu = WaitQu;
				 end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateQu <= WaitQu;
		NQueue <= 0;
		QWrite <= 0;
		QRead <= 0;
		ReadPulseOut <= 1'b0;
		ReadTagLastP1 <= 0;
		NRdSequence <= 0;
		NQerr <= 0;
	end else begin
		StateQu <= NextStateQu;
		case (StateQu)
			WaitQu:	begin
						ReadPulseOut <= 1'b0;
					end
			StorQu:	begin
						NQueue <= NQueue + 1;
						if (NQueue == 7) begin
							$display("%g\t EnergyDAQ error!  Read tag queue overflow!  Should never happen if trigger is controlled.",$time);
							NQerr <= NQerr + 1;
						end
						$display("%g\t EnergyDAQ %d, adding tag %d to the queue.  NQueue=%d, QWrite=%d",$time,Address,ReadTag,NQueue,QWrite);
						TagQueue[QWrite] <= ReadTag;
						QWrite <= QWrite + 1;
					end
			IssuQu:	begin
						ReadPulseOut <= 1'b1;
						ReadTagOut <= TagQueue[QRead];
						ReadTagLastP1 <= TagQueue[QRead] + 1;
						$display("%g\t EnergyDAQ %d: new read tag=%b, old read tag=%b, plus 1=%b",$time,Address,TagQueue[QRead],ReadTagOut,ReadTagLastP1);
						if (TagQueue[QRead] != ReadTagLastP1) begin
							NRdSequence <= NRdSequence + 1;
							$display("%g\t EnergyDAQ %d: read tag out of sequence.  Last=%d, Now=%d.",$time,Address,ReadTagOut,TagQueue[QRead]);
						end
						NQueue <= NQueue - 1;
						if (NQueue == 0) begin
							$display("%g\t EnergyDAQ error!  Read tag queue underflow.",$time);
							NQerr <= NQerr + 1;
						end
						QRead <= QRead + 1;
						$display("%g\t EnergyDAQ %d, sending read pulse to output machine with tag %d.  NQueue=%d,  QRead=%d",$time,Address,TagQueue[QRead],NQueue,QRead);
					end
		endcase
	end
end

//State machine to send the data out from the local buffers when a read command is received.
//This implements the data formats described in the documentation for data flowing to the event builder.
parameter [7:0] WaitRd=8'b00000001;		//Wait for a read signal
parameter [7:0] BOccRd=8'b00000010;		//Wait for the buffer to be occupied
parameter [7:0] HdrrRd=8'b00000100;		//Shift out a header
parameter [7:0] GetDRd=8'b00001000;		//Get the digitizations from the appropriate buffer
parameter [7:0] ShftRd=8'b00010000;		//Shift out the digitizations
parameter [7:0] CRCoRd=8'b00100000;		//Shift out the CRC code
parameter [7:0] SndSRd=8'b01000000;		//Pack the reduced data into the output buffer
parameter [7:0] Wst4Rd=8'b10000000;		//Output 4 unused bits to align 12-bit words

reg [7:0] StateRd, NextStateRd;
reg [47:0] OutputBuf;
reg [5:0] CntRd;
reg [4:0] CntSmp;
wire [4:0] CntSmpP1;
reg [17:0] Header;
reg Stop,ErrFlg;
wire signed [13:0] Dig0prnt, Dig1prnt, Dig2prnt;
wire signed [7:0] PedPrnt;
wire signed [15:0] SmpPrnt;
reg [2:0] OTR2;
reg [1:0] Cnt4Rd;

assign PedPrnt = OutputBuf[23:16];      //Signed quantities for debug printout
assign SmpPrnt = OutputBuf[15:0];
assign Dig0prnt = OutputBuf[43:30];
assign Dig1prnt = OutputBuf[28:15];
assign Dig2prnt = OutputBuf[13:0];

always @ (StateRd or ReadPulseOut or CntTime or BufOcc or ReadTagOut or Header or OutputBuf or CntRd or CntSmp or NSamples or Done or Reduce or NSampleP1 or NChWr or PedsOut or Cnt4Rd) begin
	case (StateRd)
		WaitRd:	begin
					DoutDigi = 1'b0;
					if (ReadPulseOut) NextStateRd = BOccRd;
					else NextStateRd = WaitRd;
				end
		BOccRd:	begin
					DoutDigi = 1'b0;
					if (BufOcc[ReadTagOut]) begin
						NextStateRd = HdrrRd;
					end else NextStateRd = BOccRd;
				end
		HdrrRd:	begin
					DoutDigi = Header[17];
					if (Reduce) begin
						if (CntRd==16) NextStateRd = SndSRd;   
						else NextStateRd = HdrrRd;
					end else begin
						if (CntRd==16) NextStateRd = GetDRd;
						else NextStateRd = HdrrRd;
					end
				end
		SndSRd:	begin
					NextStateRd = ShftRd;
					if (CntSmp == 0) DoutDigi = Header[17];
					else begin
						if (PedsOut) DoutDigi = OutputBuf[23];
						else DoutDigi = OutputBuf[15];
					end
				end
		GetDRd:	begin
					NextStateRd = ShftRd;
					if (CntSmp == 0) DoutDigi = Header[17];
					else DoutDigi = OutputBuf[47];
				end
		ShftRd:	begin
					if (Reduce) begin
						if (PedsOut) begin
							if (CntRd == 22)
								if (CntSmp == 1) NextStateRd = CRCoRd;  //Send only the first channel out in this case
								else NextStateRd = SndSRd;
							else NextStateRd = ShftRd;
							DoutDigi = OutputBuf[23];
						end else begin
							if (CntRd == 14)
								if (CntSmp == NChWr) begin 
									if (NChWr == 2) NextStateRd = Wst4Rd;  //Pad 4 extra bits to make a 36-bit data section
									else NextStateRd = CRCoRd;
								end else NextStateRd = SndSRd;
							else NextStateRd = ShftRd;
							DoutDigi = OutputBuf[15];
						end
					end else begin
						if (CntRd == 46)
							if (CntSmp == NSampleP1) NextStateRd = CRCoRd;
							else NextStateRd = GetDRd;
						else NextStateRd = ShftRd;
						DoutDigi = OutputBuf[47];
					end
				end
		Wst4Rd:	begin
					if (Cnt4Rd == 3) NextStateRd = CRCoRd;
					else NextStateRd = Wst4Rd;
					DoutDigi = OutputBuf[15];
				end
		CRCoRd: begin
					if (Reduce) begin
						if (PedsOut) DoutDigi = OutputBuf[23];
						else DoutDigi = OutputBuf[15];
					end else DoutDigi = OutputBuf[47];
					if (Done) NextStateRd = WaitRd;
					else NextStateRd = CRCoRd;
				end
		default:	begin
						NextStateRd = WaitRd;
						DoutDigi = 1'b0;
					end
	endcase
end

reg [31:0] DoutTime;
reg [2:0] tagSave;
reg [7:0] occSave;
always @ (posedge Clock) begin
	if (Reset) begin
		$display("%g\t EnergyDAQ:  DAQ reset received",$time);
		StateRd <= WaitRd;
		NTimeOut <= 0;
		NSent <= 0;
		ErrFlg <= 1'b0;
		TimeOut <= 1'b0;
		TrgCnt <= 0;
		DoutTime <= 0;
		tagSave <= 0;
		occSave <= 0;
		stateSave <= 0;
	end else begin
		ErrFlg <= (TimeOut | PrtyError | RdyError | TrgError);
		StateRd <= NextStateRd;
//		$display("%g\t    %b      %b    %d %d    %b    %b",$time,StateRd,ReadPulseOut,CntRd,CntSmp,Stop,Done); 
		case (StateRd)
			WaitRd: begin
						CntTime <= 0;
						Stop <= 1'b0;
						BufClr <= 8'h00;
					end
			BOccRd: begin
					    //if (CntTime == 0) $display("%g\t EnergyDAQ:  waiting for buffer to be occupied",$time);
						$display("%g\t EnergyDAQ %d: BufOcc=%b, ReadTag=%h ",$time,Address,BufOcc,ReadTagOut);
						CntTime <= CntTime + 1;
						if (CntTime == 24'hffffff && NTimeOut == 0) begin
							TimeOut <= 1'b1;
							NTimeOut <= NTimeOut + 1;
							tagSave <= ReadTagOut;
							occSave <= BufOcc;
							stateSave <= State;
						end
						CntRd <= 0;
						if (CntTime == 0) $display("%g\t EnergyDAQ %d:  output new event.  ReadTag=%d, TrgCnt=%d.",$time,Address,ReadTagOut,TrgCnt);
						if (Reduce)
							Header <= {2'b10,Address,PedsOut,TrgCnt[0],ReadTagOut,Reduce,ErrFlg,NSampleP1};
						else
							Header <= {2'b10,Address,TrgCnt,ReadTagOut,Reduce,ErrFlg,NSampleP1};
						if (NextStateRd == HdrrRd) begin
							if (ReadTagOut == 3'b111) TrgCnt <= TrgCnt + 1;
						end
						OTR2 <= OTRsave[ReadTagOut];
						ErrFlg <= 1'b0;
					end
			HdrrRd:	begin	
						// if (CntRd == 0) begin
							// if (Reduce)	$display("%g\t EnergyDAQ:  sending out header = %b",$time,{Header[17:5],NChWr,OTR2});
							// else $display("%g\t EnergyDAQ:  sending out header = %b, NSamples=%d",$time,Header[17:5],NSampleP1);
						// end
						if (Reduce && CntRd == 0) Header <= {Header[16:5],NChWr,OTR2,1'b0};
						else Header <= {Header[16:0],1'b0};
						CntRd <= CntRd + 1;
						TimeOut <= 1'b0;
						CntSmp <= 0;
						DoutTime <= DoutTime + 1;
					end
			SndSRd:	begin
						CntRd <= 0;
						CntSmp <= CntSmp + 1;
						Header <= {Header[16:0],1'b0};
						case (CntSmp)
							5'b00000: OutputBuf[23:0] <= Reduced0[ReadTagOut];  //Only this one if pedestals are output
							5'b00001: OutputBuf[23:0] <= Reduced1[ReadTagOut];
							5'b00010: OutputBuf[23:0] <= Reduced2[ReadTagOut];
						endcase
						DoutTime <= DoutTime + 1;
					end
			GetDRd:	begin
						CntRd <= 0;
						CntSmp <= CntSmp + 1;
						Header <= {Header[16:0],1'b0};
						case (ReadTagOut)
							3'b000: begin OutputBuf <= {CntSmpP1[2:0],DigiStore0[CntSmp]}; end
							3'b001: begin OutputBuf <= {CntSmpP1[2:0],DigiStore1[CntSmp]}; end
							3'b010: begin OutputBuf <= {CntSmpP1[2:0],DigiStore2[CntSmp]}; end
							3'b011: begin OutputBuf <= {CntSmpP1[2:0],DigiStore3[CntSmp]}; end
							3'b100: begin OutputBuf <= {CntSmpP1[2:0],DigiStore4[CntSmp]}; end
							3'b101: begin OutputBuf <= {CntSmpP1[2:0],DigiStore5[CntSmp]}; end
							3'b110: begin OutputBuf <= {CntSmpP1[2:0],DigiStore6[CntSmp]}; end
							3'b111: begin OutputBuf <= {CntSmpP1[2:0],DigiStore7[CntSmp]}; end
						endcase
						DoutTime <= DoutTime + 1;
					end
			ShftRd:	begin
						// if (CntRd == 0) begin
							// if (Reduce) $display("%g\t EnergyDAQ: sending out reduced data %b, Ped=%d, Smp=%d",$time,OutputBuf[23:0],PedPrnt,SmpPrnt); 
							// else $display("%g\t EnergyDAQ: %d sending out digitizations %b 0=%d 1=%d 2=%d",$time,OutputBuf[47:45],OutputBuf[44:0],Dig0prnt,Dig1prnt,Dig2prnt);
						// end
						CntRd <= CntRd + 1;
						OutputBuf <= {OutputBuf[46:0],1'b0};
						if (Reduce) begin
							if (PedsOut) begin
								if (CntRd == 22 && CntSmp == 1) Stop <= 1'b1;	//The other state machine will send out other channels
							end else begin
								if (CntRd == 14 && CntSmp == 3) Stop <= 1'b1;
							end
						end else begin
							if (CntRd == 46 && CntSmp == NSampleP1) Stop <= 1'b1;
						end
						Cnt4Rd <= 0;
						DoutTime <= DoutTime + 1;
					end
			Wst4Rd:	begin
						Cnt4Rd <= Cnt4Rd + 1;
						OutputBuf <= {OutputBuf[46:0],1'b0};
						if (Cnt4Rd == 3) Stop <= 1'b1;
						DoutTime <= DoutTime + 1;
					end
			CRCoRd: begin
						OutputBuf <= {OutputBuf[46:0],1'b0};
						if (Done) begin
							NSent <= NSent + 1;
							BufClr[ReadTagOut] <= 1'b1;
//							$display("%g\t EnergyDAQ %d: State CRCoRd, Setting BufClr for tag %d, BufClr=%b, BufOcc=%b.",$time,Address,ReadTagOut,BufClr,BufOcc);
						end
						DoutTime <= DoutTime + 1;
						Stop <= 1'b0;
					end
		endcase
	end
end


assign CntSmpP1 = CntSmp + 1;

//Multiplex the output stream to be either register or digitizer information
always @ (DoutReg or DoutDigi or StateCM or DoutCRC or StateRd) begin
	if (StateCM == RegOCM) Dout = DoutReg;
	else if (StateRd == WaitRd || StateRd == BOccRd) Dout = 1'b0;
	else Dout = DoutCRC;
end

//Attache a 6-bit cyclic redundancy code to the digitizer data stream going to the event builder
CRC6 CRC6_Energy(DoutCRC,Done,DoutDigi,Stop,Reset,Clock,Address);
//always @ (posedge Stop) $display("%g\t EnergyDAQ:  stop signal for CRC6 encountered.",$time);

// always @ (posedge Done2) begin
	// $display("%g\t EnergyDAQ %d:  Done2 signal.",$time,Address);
// end
// always @ (posedge StopD2) begin
	// $display("%g\t EnergyDAQ %d:  StopD2 signal.",$time,Address);
// end

//A second state machine to send data to the event builder.  This one only operates when sending
//reduced data with pedestals, and it sends just the last one or two channels, with no 12-bit header
parameter [5:0] WaitR2=6'b000001;		//Wait for a read signal
parameter [5:0] BOccR2=6'b000010;		//Wait for the buffer to be occupied
parameter [5:0] HdrrR2=6'b000100;		//Shift out a HdrD2
parameter [5:0] SndSR2=6'b001000;		//Pack the reduced data into the output buffer
parameter [5:0] ShftR2=6'b010000;		//Shift out the digitizations
parameter [5:0] CRCoR2=6'b100000;		//Shift out the CRC code

reg [5:0] StateR2, NextStateR2;
reg [23:0] D2buf;
reg [5:0] CntR2;
reg [5:0] HdrD2;        //Header for the second data channel
reg StopD2;
reg [1:0] CntChns;

reg DoutDig2;
always @ (StateR2 or Reduce or PedsOut or ReadPulseOut or D2buf or ReadTagOut or BufOcc or HdrD2 or CntR2 or CntChns or NChWr or CntTime or Done2) begin
	case (StateR2)
		WaitR2:	begin
					DoutDig2 = 1'b0;
					if (ReadPulseOut & Reduce & PedsOut) NextStateR2 = BOccR2;
					else NextStateR2 = WaitR2;
				end
		BOccR2:	begin
					DoutDig2 = 1'b0;
					if (CntTime == 24'hffffff) NextStateR2 = WaitR2;  //CntTime is counted in the other output state machine
					else if (BufOcc[ReadTagOut]) begin
						NextStateR2 = HdrrR2;
					end else NextStateR2 = BOccR2;
				end
		HdrrR2:	begin
					DoutDig2 = HdrD2[5];
					if (CntR2==4) NextStateR2 = SndSR2;   
					else NextStateR2 = HdrrR2;
				end
		SndSR2:	begin
					NextStateR2 = ShftR2;
					if (CntChns == 1) DoutDig2 = HdrD2[5];   	//First time through, output the last header bit
					else begin
						DoutDig2 = D2buf[23];					//Output the last bit of the previous channel's data
					end
				end
		ShftR2:	begin
					if (CntR2 == 22)
						if (CntChns == NChWr) NextStateR2 = CRCoR2;
						else NextStateR2 = SndSR2;
					else NextStateR2 = ShftR2;
					DoutDig2 = D2buf[23];
				end
		CRCoR2: begin
					DoutDig2 = D2buf[23];
					if (Done2) NextStateR2 = WaitR2;
					else NextStateR2 = CRCoR2;
				end
		default:	begin
						NextStateR2 = WaitR2;
						DoutDig2 = 1'b0;
					end
	endcase
end

always @ (posedge Clock) begin
	if (Reset) begin
		StateR2 <= WaitR2;
	end else begin
		StateR2 <= NextStateR2;
//		if (StateR2 != WaitR2) $display("%g\t EnergyDAQ %d: StateR2=%b, CntR2=%d, HdrD2=%b, CntChns=%d, D2buf=%b, DoutDig2=%b, StopD2=%b, Dout2=%b",$time,Address,StateR2,CntR2,HdrD2,CntChns,D2buf,DoutDig2,StopD2,Dout2);
		case (StateR2)
			WaitR2: begin
						StopD2 <= 1'b0;
					end
			BOccR2: begin
						CntR2 <= 0;    //To count 6 header bits
						if (NChWr == 2) HdrD2 <= {2'b10,Address};  //Second bit tells the event builder how many channels to expect
						else HdrD2 <= {2'b11,Address};
					end
			HdrrR2:	begin	
						if (CntR2 == 0) $display("%g\t EnergyDAQ %d, sending header %b to second data line",$time,Address,HdrD2);
						HdrD2 <= {HdrD2[4:0],1'b0};
						CntR2 <= CntR2 + 1;
						CntChns <= 1;  //To count channels, but the other state machine is doing channel-0
					end
			SndSR2:	begin
						CntR2 <= 0;    //To count 24 bits for one channel's data
						CntChns <= CntChns + 1;
						HdrD2 <= {HdrD2[4:0],1'b0};
						case (CntChns)          //This state machine sends data of only the last two channels
							2'b01: D2buf[23:0] <= Reduced1[ReadTagOut];
							2'b10: D2buf[23:0] <= Reduced2[ReadTagOut];
						endcase
					end
			ShftR2:	begin
						CntR2 <= CntR2 + 1;
						D2buf <= {D2buf[22:0],1'b0};
						if (NextStateR2 == CRCoR2) StopD2 <= 1'b1;		
					end
			CRCoR2: begin
						D2buf <= {D2buf[22:0],1'b0};
						StopD2 <= 1'b0;
					end
		endcase
	end
end

//Attache a 6-bit cyclic redundancy code to the second digitizer data stream going to the event builder
CRC6 CRC6_Energy2(Dout2,Done2,DoutDig2,StopD2,Reset,Clock,Address);

endmodule

//Dual ported block RAM used as a pipeline to buffer the incoming digitizations pending a trigger
module DigiBufRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [7:0] addra, addrb;   //Addresses for the primary and secondary ports
input [14:0] dia;           //Input data register to write to memory
output [14:0] doa, dob;     //Output registers for the two ports
reg [14:0] RAM [255:0];     //Memory array
reg [14:0] dob, doa;

always @(posedge clka)
begin
	if (ena) begin
		if (wea) begin
			RAM[addra]<=dia;  
//			if (addr == 12 && inst == 1) $display("%g\t DigiBufRAM, storing in RAM location %d data=%d %b",$time,addra,dia,dia);
			doa <= dia;
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb) begin
		dob <= RAM[addrb];
//		if (addr == 12 && inst == 1) $display("%g\t DigiBufRAM, retrieving from RAM location %d data=%d %b",$time,addrb,RAM[addrb],RAM[addrb]);
	end
end
endmodule

//Dual ported block RAM used as a FIFO for buffering read start addresses
module ReadFifoRAM (clka, clkb, ena, enb, wea, addra, addrb, dia, doa, dob);
input clka, clkb;
input wea;                  //Write enable
input ena, enb;             //Primary and secondary read enables
input [3:0] addra, addrb;   //Addresses for the primary and secondary ports
input [7:0] dia;           //Input data register to write to memory
output [7:0] doa, dob;     //Output registers for the two ports
reg [7:0] RAM [15:0];     //Memory array
reg [7:0] dob, doa;

always @(posedge clka)
begin
	if (ena) begin
		if (wea) begin
			RAM[addra]<=dia;  
			doa <= dia;
		end else doa <= RAM[addra];        //Write first, then read
	end
end

always @(posedge clkb)
begin
	if (enb) begin
		dob <= RAM[addrb];
	end
end
endmodule
