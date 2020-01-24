//Module to buffer the data stream from an energy digitizer board
//and prepare it for insertion into the event stream.
//One state machine monitors the data stream coming from one
//tracker front-end board and writes the data into RAM in 12-bit
//words.  There is a FIFO pipeline of 16 events.  Every time
//a Send signal is received, the next event is output one 12-bit
//word per clock cycle, thus achieving a factor of 12 speedup of
//output rate versus input rate.
//There are 2 instances of this module, plus 12 of a similar
//module for the tracker boards.  All 14 receive data in parallel,
//but only one at a time outputs event data, so that events are built
//and sent into a buffer of 12-bit words.
//R. Johnson
//July 11, 2013 -- Modify so a delta-T word is not expected for reduced data with 3 channels
//August 11, 2013 -- Added buffering of errors
//August 27, 2013 -- removed unused error signal
//January 15, 2014 -- modified it so that delta-T is never expected.  Works only with v44 or later of event builder.
//January 26, 2014 -- modified to write out only the first channel in the case of reduced data with pedestals (2nd link will do the others)
module EnrgBuf2(TwoLinks,ErrBufOut, CRCerr, Bufree, BufOcc, NFull, BufRdy, DataOut, SendHold, AddErr, Clock, Reset, Data, Address, Send, SendErrBuf, Trigger, BufMngrType);
input BufMngrType;
input Trigger;			//Pulsed each time a trigger is accepted
input Clock;
input Reset; 
input Data;       		//Incoming serial data stream.
input Send;             //Send the next event
input SendErrBuf;		//Send error info for next event
input [3:0] Address;    //Expected address of the FPGA board sending data.
output AddErr;  		//If set, an FPGA address error was encountered.
output SendHold;		//Request to disable sending events from tracker boards to the event builder
output [11:0] DataOut;  //Output stream of 12-bit words
output BufRdy;			//Logic high if an output buffer is ready for reading
output [4:0] NFull;		//Number of buffers full, out of 16, for monitoring
output CRCerr;			//1-clock pulse each time a CRC error is caught
output [7:0] ErrBufOut;	//Buffered error information to write out with event
output TwoLinks;		//True if two data links are used per digitizer board
output [3:0] BufOcc;	//Current number of front-end-board buffers full
output Bufree;			//True if a front-end-board buffer is available

reg AddErr,CRCerr;
reg [11:0] DataOut;

reg [7:0] ErrBuf[0:15]; //Buffer error information here
reg [7:0] ErrBufIn, ErrBufOut;
reg [3:0] BufOcc;	//Keep track of how many front-end-board buffers are full

//Debug printout
//initial begin
//  $display("Time     Clk State Next Data CntH CntE CntC CntB CntF   Header    Chip        Cluster StWrd Packetype Address Reset");
//  $monitor("%g\t %b  %b  %b   %b   %h   %h   %h    %h    %h %b %b %b   %b %b %h  %b",$time,Clock,State,NextState,Data,CntH,CntE,CntC,CntB,CntF,Header,Chip,Cluster,StWrd,PacketType,Address,Reset);
//end

//always @ (posedge Clock) begin
//	if (Address == 0) begin
//  	$display("%g\t %b  %b   %b   %h   %h   %h    %h    %h %b %b %b   %b %b",$time,State,NextState,Data,CntH,CntE,CntC,CntB,CntF,Header,Chip,Cluster,StWrd,PacketType);
//	end
//end
reg [4:0] NFull;		 //Number of occupied buffers
reg [7:0] NWords[0:15];  //Number of words written into each event buffer
reg Done;

assign BufRdy = (NFull > 0);
assign SendHold = (NFull >= 12);

reg AllDone;
reg [7:0] Addr, Addw;
reg [3:0] WritePtr, ReadPtr;
reg [11:0] DataWordIn, DWd0,DWd1,DWd2,DWd3,DWd4,DWd5,DWd6,DWd7;
wire [11:0] DataWordOut0,DataWordOut1,DataWordOut2,DataWordOut3,DataWordOut4,DataWordOut5,DataWordOut6,DataWordOut7;
wire [11:0] DataWordOut8,DataWordOut9,DataWordOutA,DataWordOutB,DataWordOutC,DataWordOutD,DataWordOutE,DataWordOutF;
wire [11:0] doa0,doa1,doa2,doa3,doa4,doa5,doa6,doa7,doa8,doa9,doaA,doaB,doaC,doaD,doaE,doaF;

//State machine for monitoring the data stream and storing event data into a memory buffer
parameter [8:0] Wait=9'b000000001;  //Look for an event start bit 
parameter [8:0] Hdrr=9'b000000010;	//Get the 6-bit header, which includes the FPGA ID
parameter [8:0] Hdr2=9'b000000100;	//Get the 12-bit event header
parameter [8:0] Regs=9'b000001000;	//Clock in register information (will be ignored here)
parameter [8:0] RegO=9'b000010000;	//Finish with the register information and return to Wait
parameter [8:0] Digi=9'b000100000;	//Get the channel data, reduced or digis
parameter [8:0] CRCo=9'b001000000;	//Get the CRC and check it
parameter [8:0] Fini=9'b010000000;	//Store the last data word
parameter [8:0] SNwd=9'b100000000;  //Write the number of words into the first word of memory

reg [8:0] State, NextState;

reg [4:0] CntH;
reg [3:0] CntS;
reg [6:0] Cnt; 
reg [77:0] RegOut;
reg [5:0] CRC;
reg Reduce;
reg [1:0] NChan;

wire signed [7:0] PedPrnt;
wire signed [15:0] SmpPrnt;
assign PedPrnt = RegOut[23:16];
assign SmpPrnt = RegOut[15:0];

parameter [4:0] NSamples = 4'hf;
wire [5:0] CRCnew;
reg [15:0] ena;   //Write enables
reg [15:0] enb;   //Read enables
reg PacketType;
reg DataDly, Stop, Start, PedsOut;
reg [11:0] Header;

wire signed [13:0] Dig0, Dig1, Dig2;

assign Dig0 = RegOut[43:30];
assign Dig1 = RegOut[28:15];
assign Dig2 = RegOut[13:0];

always @ (State or Data or Cnt or CntH or CntS or Reduce or NChan or WritePtr or Header or Address or PacketType or PedsOut) begin
	case (State)
		Wait:	begin
					ena = 0;
					if (Data) NextState = Hdrr;
					else NextState = Wait;
				end
		Hdrr:	begin
					ena = 0;
					if (CntH == 11) 
						if (PacketType == 1'b1) NextState = Regs;
						else NextState = Hdr2;
					else NextState = Hdrr;
				end
		Regs:	begin
					ena = 0;
					if (Cnt == 77) NextState = RegO;
					else NextState = Regs;
				end
		RegO:	begin
					ena = 0;
					NextState = Wait;
				end
		Hdr2:	begin
					ena = 0;
					if (CntH == 12 && (Header[9:6] != Address)) NextState = Wait;		//Abort, error condition			
					else begin
						if (CntH==17) NextState = Digi;   
						else NextState = Hdr2;
					end
				end
		Digi:	begin
					if (Cnt == 0 || Cnt == 12 || Cnt == 24 || Cnt == 36) ena = FourToSixteen(WritePtr);  //Write header or data
					else ena = 0;
					if (Reduce) begin
						if (PedsOut) begin
							if (CntS == 0 && Cnt == 23) NextState = CRCo;   //Output only the first channel of data
							else NextState = Digi;	
						end else begin
							if (NChan == 3) begin
								if (CntS == 1 && Cnt == 23) NextState = CRCo;
								else NextState = Digi;
							end else begin
								if (CntS == 1 && Cnt == 11) NextState = CRCo;
								else NextState = Digi;
							end
						end
					end else begin
						if (CntS == NSamples && Cnt == 47) NextState = CRCo;
						else NextState = Digi;
					end
				end
		CRCo:	begin
					ena = 0;
					if (CntH == 5) NextState = Fini;
					else NextState = CRCo;
				end
		Fini:	begin
					ena = FourToSixteen(WritePtr);  //Write out the last data word
					NextState = SNwd;
				  end
		SNwd: 	begin
					ena = FourToSixteen(WritePtr);	//Write the number of words
					NextState = Wait;
				end
		default:	begin
						ena = 0;
						NextState = Wait;
					end
	endcase
end

reg Bufree;
always @ (BufMngrType or BufOcc) begin
	if (BufMngrType) Bufree = (BufOcc < 8);
	else Bufree = (BufOcc < 4);
end

assign TwoLinks = Reduce & PedsOut;
always @ (posedge Clock) begin
	if (Reset) begin
		State <= Wait;
		NFull <= 0;
		AddErr <= 1'b0;
		WritePtr <= 0;
		Start <= 1'b0;
		CRCerr <= 1'b0;
		Reduce <= 1'b1;
		PedsOut <= 1'b1;
		BufOcc <= 0;
	end else begin
		State <= NextState;
		DataDly <= Data;
//		if (State != Wait) $display("%g\t EnrgBuf2 %d: State=%b, Cnt=%d, CntS=%d, CntH=%d, Data=%b, Header=%b, ena=%b, DataWordIn=%b, WritePtr=%d",$time,Address,State,Cnt,CntS,CntH,Data,Header,ena,DataWordIn,WritePtr);
		if (Trigger & !Done) begin
			BufOcc <= BufOcc + 1;
//			$display("%g\t EnrgBuf2 %d, incrementing BufOcc to %d",$time,Address,BufOcc+1);
		end
		if (Done & !Trigger) begin
			BufOcc <= BufOcc - 1;
//			$display("%g\t EnrgBuf2 %d, decrementing BufOcc to %d",$time,Address,BufOcc-1);
		end
		if (AllDone && !Done && NFull != 0) begin
			NFull <= NFull - 1;
//			$display("%g\t EnrgBuf2 %d:  decrementing NFull=%d, BufRdy=%b",$time,Address,NFull,BufRdy);  
		end
		if (Done && !AllDone) begin
			NFull <= NFull + 1;
//			$display("%g\t EnrgBuf2 %d:  incrementing NFull=%d, BufRdy=%b",$time,Address,NFull,BufRdy);  
		end
		if ((NextState == CRCo) && (State != CRCo)) Stop <= 1'b1;
		case (State)
			Wait:	begin
						Done <= 1'b0;
						CntH <= 1;
						Cnt <= 0;
						CntS <= 0;
						Addw <= 1;
						Header <= {Header[10:0],Data};
						DataWordIn <= {DataWordIn[10:0],Data}; 
						if (Data) Start <= 1'b1;
					end
			Hdrr:	begin
						Start <= 1'b0;
						if (CntH == 1) PacketType <= Data;
						ErrBufIn <= 0;
						CntH <= CntH + 1;
						Header <= {Header[10:0],Data};
						DataWordIn <= {DataWordIn[10:0],Data}; 
					end
			Regs:	begin
						if (Cnt == 0) begin
//							$display("%g\t EnrgBuf2 %d: Header=%b",$time,Address,Header);
							Stop <= 1'b1;
						end else Stop <= 1'b0;
						RegOut <= {RegOut[76:0],Data};
						Cnt <= Cnt + 1;
					end
			RegO:	begin
						$display("%g\t EnrgBuf2 %d: Register data=%b",$time,Address,RegOut);
					end
			Hdr2:	begin
						CntH <= CntH + 1;
						Header <= {Header[10:0],Data};
						DataWordIn <= {DataWordIn[10:0],Data};    //Shift in the header and delta-time from the input serial stream
						if (CntH == 12) begin
//							$display("%g\t EnrgBuf2 %d: Header=%b, tag=",$time,Address,Header,Header[2:1]);
							Reduce <= Header[0];
							PedsOut <= Header[5];
//							if (Header[0] != Reduce || PedsOut != Header[5]) $display("%g\t EnrgBuf2 %d, changing TwoLinks flag, Reduce=%b, PedsOut=%b.",$time,Address,Header[0],Header[5]);
							ErrBufIn[7:3] <= Header[5:1];
							if (Header[9:6] != Address) begin
								AddErr <= 1'b1; 
								ErrBufIn[1] <= 1'b1;
								$display("%g\t EnrgBuf2 %d, Address mismatch=%d.  Ignoring start sequence.",$time,Address,Header[9:6]);
							end else ErrBufIn[1] <= 1'b0;
						end
						if (CntH == 15) begin
							NChan <= Header[1:0];
//							$display("%g\t EnrgBuf2 %d: Reduce=%b,  NChan=%h",$time,Address,Reduce,Header[1:0]);
						end
					end
			Digi:	begin
//						$display("%g\t EnrgBuf2 %d Digi:  CntS=%d, Cnt=%d, NChan=%d",$time,Address,CntS,Cnt,NChan);
						if (Cnt == 0) begin
							CntH <= 0;
							Addw <= Addw + 1;
						end
						if (Cnt == 12 || Cnt == 24 || Cnt == 36) Addw <= Addw + 1;
						Cnt <= Cnt + 1;
						RegOut <= {RegOut[76:0],Data};
						DataWordIn <= {DataWordIn[10:0],Data};
						if ((Cnt == 47 && !Reduce) || (Cnt == 23 && Reduce)) begin
							Cnt <= 0;
							CntS <= CntS + 1;
						end
					end
			CRCo:	begin
						Stop <= 1'b0;
						CntH <= CntH + 1;
						CRC <= {CRC[4:0],Data};
					end
			Fini:	begin
						// if (Reduce) $display("%g\t EnrgBuf2 %d: Reduced data2=%b %d %d",$time,Address,RegOut[23:0],PedPrnt,SmpPrnt);
						// else $display("%g\t EnrgBuf2 %d: %d %d Digitizations=%b %d %d %d",$time,Address,NSamples+1,RegOut[49:45],RegOut[44:0],Dig0,Dig1,Dig2);
						// $display("%g\t EnrgBuf2 %d: CRC=%b, CRCnew=%b",$time,Address,CRC,CRCnew);
					    NWords[WritePtr] <= Addw;
					    DataWordIn <= Addw;
					    Addw <= 0;
//					    $display("%g\t EnrgBuf2 %d:  incrementing the write pointer",$time,Address);
					    if (CRC != CRCnew) begin
							CRCerr <= 1'b1;
							ErrBufIn[0] <= 1'b1;
							$display("%g\t EnrgBuf2 %d:  CRC mismatch.  CRC=%b,  CRCnew=%b",$time,Address,CRC,CRCnew);
						end else ErrBufIn[0] <= 1'b0;
					end
			SNwd:	begin
						ErrBuf[WritePtr] <= ErrBufIn;
					    WritePtr <= WritePtr + 1;  
//					    $display("%g\t     EnrgBuf2 %d: storing number of words %d in location %d address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
					    Done <= 1'b1;
						CRCerr <= 1'b0;
					end
		endcase
	end
end

//Module to check the incoming CRC
CRCcheck CRCcheck_U(CRCnew,DataDly,Stop,Start,Reset,Clock,Address);

//Instantiate Block RAM memory for the data buffers
EvtBufRAM EvtBufRAM0(Clock, Clock, ena[0], enb[0], ena[0], Addw, Addr, DataWordIn, doa0, DataWordOut0);
EvtBufRAM EvtBufRAM1(Clock, Clock, ena[1], enb[1], ena[1], Addw, Addr, DataWordIn, doa1, DataWordOut1);
EvtBufRAM EvtBufRAM2(Clock, Clock, ena[2], enb[2], ena[2], Addw, Addr, DataWordIn, doa2, DataWordOut2);
EvtBufRAM EvtBufRAM3(Clock, Clock, ena[3], enb[3], ena[3], Addw, Addr, DataWordIn, doa3, DataWordOut3);
EvtBufRAM EvtBufRAM4(Clock, Clock, ena[4], enb[4], ena[4], Addw, Addr, DataWordIn, doa4, DataWordOut4);
EvtBufRAM EvtBufRAM5(Clock, Clock, ena[5], enb[5], ena[5], Addw, Addr, DataWordIn, doa5, DataWordOut5);
EvtBufRAM EvtBufRAM6(Clock, Clock, ena[6], enb[6], ena[6], Addw, Addr, DataWordIn, doa6, DataWordOut6);
EvtBufRAM EvtBufRAM7(Clock, Clock, ena[7], enb[7], ena[7], Addw, Addr, DataWordIn, doa7, DataWordOut7);
EvtBufRAM EvtBufRAM8(Clock, Clock, ena[8], enb[8], ena[8], Addw, Addr, DataWordIn, doa8, DataWordOut8);
EvtBufRAM EvtBufRAM9(Clock, Clock, ena[9], enb[9], ena[9], Addw, Addr, DataWordIn, doa9, DataWordOut9);
EvtBufRAM EvtBufRAMa(Clock, Clock, ena[10], enb[10], ena[10], Addw, Addr, DataWordIn, doaA, DataWordOutA);
EvtBufRAM EvtBufRAMb(Clock, Clock, ena[11], enb[11], ena[11], Addw, Addr, DataWordIn, doaB, DataWordOutB);
EvtBufRAM EvtBufRAMc(Clock, Clock, ena[12], enb[12], ena[12], Addw, Addr, DataWordIn, doaC, DataWordOutC);
EvtBufRAM EvtBufRAMd(Clock, Clock, ena[13], enb[13], ena[13], Addw, Addr, DataWordIn, doaD, DataWordOutD);
EvtBufRAM EvtBufRAMe(Clock, Clock, ena[14], enb[14], ena[14], Addw, Addr, DataWordIn, doaE, DataWordOutE);
EvtBufRAM EvtBufRAMf(Clock, Clock, ena[15], enb[15], ena[15], Addw, Addr, DataWordIn, doaF, DataWordOutF);

//always @ (posedge Clock) begin
//	$display("%g\t EnrgBuf2 %d: DataWordOut=%b %b %b %b",$time,Address,DataWordOut0,DataWordOut1,DataWordOut2,DataWordOut3);
//end

always @ (ReadPtr or DataWordOut0 or DataWordOut1 or DataWordOut2 or DataWordOut3 or DataWordOut4 or DataWordOut5
			or DataWordOut6 or DataWordOut7 or DataWordOut8 or DataWordOut9 or DataWordOutA or DataWordOutB
			or DataWordOutC or DataWordOutD or DataWordOutE or DataWordOutF) begin
  case (ReadPtr)
	4'h0: DataOut = DataWordOut0;
	4'h1: DataOut = DataWordOut1;
	4'h2: DataOut = DataWordOut2;
	4'h3: DataOut = DataWordOut3;
	4'h4: DataOut = DataWordOut4;
	4'h5: DataOut = DataWordOut5;
	4'h6: DataOut = DataWordOut6;
	4'h7: DataOut = DataWordOut7;
	4'h8: DataOut = DataWordOut8;
	4'h9: DataOut = DataWordOut9;
	4'ha: DataOut = DataWordOutA;
	4'hb: DataOut = DataWordOutB;
	4'hc: DataOut = DataWordOutC;
	4'hd: DataOut = DataWordOutD;
	4'he: DataOut = DataWordOutE;
	4'hf: DataOut = DataWordOutF;
  endcase	
end

// State machine to send the data out from one of the event buffers

parameter [2:0] WaitSd = 3'b001;   //Wait for a start signal
parameter [2:0] NextSd = 3'b010;   //Read the next data word from memory
parameter [2:0] DoneSd = 3'b100;   //Signal that the event is done
reg[2:0] StateSd, NextStateSd;
reg[7:0] NWrds;						//Number of words for the event being read

always @ (StateSd or Send or NFull or ReadPtr or NWrds or Addr)
begin
	case (StateSd) 
		WaitSd: begin
					if (Send && (NFull > 0)) begin
						enb = FourToSixteen(ReadPtr);
						NextStateSd = NextSd;
					end else begin
						NextStateSd = WaitSd;
						enb = 0;
					end
				end
		NextSd:	begin
					if (Addr==NWrds) NextStateSd = DoneSd;
					else NextStateSd = NextSd;
					enb = FourToSixteen(ReadPtr);
				end
		DoneSd:	begin
					NextStateSd = WaitSd;
					enb = 0;
				end
		default: begin NextStateSd = WaitSd; enb = 0; end
	endcase
end

always @ (posedge Clock)
begin
	if (Reset) begin
		StateSd <= WaitSd;
		ReadPtr <= 0;
		ErrBufOut <= 0;
	end else begin
		StateSd <= NextStateSd;
		if (SendErrBuf) begin
			ErrBufOut <= ErrBuf[ReadPtr];
			$display("%g\t EnrgBuf2 %d, Sending error buffer %b",$time,Address,ErrBuf[ReadPtr]);
		end
		case (StateSd)
			WaitSd:	begin
						if (Send && (NFull > 0)) begin
//							ErrBufOut <= ErrBuf[ReadPtr];
							NWrds <= NWords[ReadPtr];
							Addr <= Addr + 1;
//							$display("%g\t EnrgBuf2 %d, State WaitSd:  NWords=%d, ReadPtr=%d, Addr=%d",$time,Address,NWords[ReadPtr],ReadPtr,Addr);
						end else Addr <= 0;
						AllDone <= 1'b0;
					end
			NextSd:	begin
						Addr <= Addr + 1;
//						$display("%g\t EnrgBuf2 %d, State NextSd:  Addr=%d, ReadPtr=%d, NWrds=%d",$time,Address,Addr,ReadPtr,NWrds);
					end
			DoneSd:	begin
						AllDone <= 1'b1;
//						$display("%g\t EnrgBuf2 %d, setting the AllDone flag.  Addr=%d, ReadPtr=%d, WritePtr=%d, NWrds=%d",$time,Address,Addr,ReadPtr,WritePtr,NWrds);
						ReadPtr <= ReadPtr + 1;
					end
		endcase
	end
end

function [15:0] FourToSixteen;
input [3:0] a;
case (a)
	4'h0: FourToSixteen = 16'b0000000000000001;
	4'h1: FourToSixteen = 16'b0000000000000010;
	4'h2: FourToSixteen = 16'b0000000000000100;
	4'h3: FourToSixteen = 16'b0000000000001000;
	4'h4: FourToSixteen = 16'b0000000000010000;
	4'h5: FourToSixteen = 16'b0000000000100000;
	4'h6: FourToSixteen = 16'b0000000001000000;
	4'h7: FourToSixteen = 16'b0000000010000000;
	4'h8: FourToSixteen = 16'b0000000100000000;
	4'h9: FourToSixteen = 16'b0000001000000000;
	4'ha: FourToSixteen = 16'b0000010000000000;
	4'hb: FourToSixteen = 16'b0000100000000000;
	4'hc: FourToSixteen = 16'b0001000000000000;
	4'hd: FourToSixteen = 16'b0010000000000000;
	4'he: FourToSixteen = 16'b0100000000000000;
	4'hf: FourToSixteen = 16'b1000000000000000;
endcase
endfunction

endmodule
