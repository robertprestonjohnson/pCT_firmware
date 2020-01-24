//Module to buffer the mergeddata stream from a tracker board
//and prepare it for insertion into the event stream.
//One state machine monitors the data stream coming from one
//tracker front-end board and writes the data into RAM in 12-bit
//words.  There is a FIFO pipeline of 16 events.  Every time
//a Send signal is received, the next event is output one 12-bit
//word per clock cycle, thus achieving a factor of 12 speedup of
//output rate versus input rate.
//There are 12 instances of this module, plus two of a similar
//module for the energy detector.  All 14 receive data in parallel,
//but only one at a time outputs event data, so that events are built
//and sent into a buffer of 12-bit words.
//R. Johnson
//Modified 7/9/2013 to put the FPGA address into the layer header
//Modified 8/8/2013 to initialize ErrSgnl and make CRCerr one clock cycle in length
//Modified 8/11/2013 to buffer error information
//Modified 8/14/2013 so that the CRC checking works for empty as well as full events
//Modified 4/14/2014 to avoid an infinite loop if the number of clusters is zero
module EvtBuf12(ErrBufOut, CRCerr, Bufree, BufOcc, NFull, ErrSgnl, BufRdy, DataOut, SendHold, AddErr, Clock, Reset, Data, Address, Send, SendErrBuf, Trigger, BufMngrType);
input BufMngrType;
input Trigger;			//Pulsed each time a trigger is accepted
input Clock;
input Reset; 
input Data;       		//Incoming serial data stream.
input Send;             //Send the next event
input SendErrBuf;		//Send error info for next event
input [3:0] Address;    //Expected address of the FPGA board sending data.
output Bufree;			//True if a front-end-board buffer is available
output AddErr;  		//If set, an FPGA address error was encountered.
output SendHold;		//Request to disable sending events from tracker boards to the event builder
output [11:0] DataOut;  //Output stream of 12-bit words
output BufRdy;			//Logic high if an output buffer is ready for reading
output ErrSgnl;			//One-clock pulse when a chip DAQ error is encountered
output [4:0] NFull;		//Number of buffers full, out of 16, for monitoring
output CRCerr;			//1-clock pulse each time a CRC error is caught
output [7:0] ErrBufOut;	//Buffered error information to write out with event
output [3:0] BufOcc;	//Current number of front-end-board buffers full

reg AddErr,ErrSgnl,CRCerr;
reg [11:0] DataOut;

reg [7:0] ErrBuf[0:15]; //Buffer error information here
reg [7:0] ErrBufIn, ErrBufOut;
reg [15:0] evtCnt, evtOut, trgIn; 		//Event counters for debugging only
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

//State machine for monitoring the data stream
//Enumeration of states for the data monitor machine
parameter [10:0] Idle = 11'b00000000001;   //Wait for start bit
parameter [10:0] StBt = 11'b00000000010;   //Shift out the 6-bit start code
parameter [10:0] Hdrr = 11'b00000000100;   //Shift out header
parameter [10:0] Regs = 11'b00000001000;   //Print register data
parameter [10:0] ChHd = 11'b00000010000;   //Shift out chip header
parameter [10:0] Clus = 11'b00000100000;   //Shift out clusters
parameter [10:0] EOCL = 11'b00001000000;   //End of cluster
parameter [10:0] CRC1 = 11'b00010000000;   //Shift out first bit of CRC
parameter [10:0] GCRC = 11'b00100000000;   //Shift out the 6-bit CRC
parameter [10:0] EOEV = 11'b01000000000;   //End of event
parameter [10:0] SNwd = 11'b10000000000;   //Write the number of words into the first word of memory

reg [10:0] State, NextState;

reg [3:0] CntE, CntB, CntF;
reg [4:0] CntH;
reg [4:0] CntC;
reg [7:0] CntN;
reg [5:0] RegWrd;
reg [5:0] StWrd;
reg [11:0] Header, Chip, Cluster;
reg [5:0] CRC;
wire [5:0] CRCnew;
reg [15:0] ena;   //Write enables
reg [15:0] enb;   //Read enables
reg PacketType, DataDly, Stop, Start;

always @ (State or Address or Data or CntH or CntE or CntC or CntF or CntB or CntN or WritePtr or Header or PacketType or Chip or StWrd)
begin
  case(State)
    Idle: begin
			ena = 0; 
			Stop = 1'b0;
			if (Data) NextState = StBt;
			else NextState = Idle;
		  end
	StBt: begin
			ena = 0;
			Stop = 1'b0;
			if (CntH==4) begin
				NextState = Hdrr;
			end else NextState = StBt;
		  end
    Hdrr: begin
			ena = 0;
			Stop = 1'b0;
			if (StWrd[3:0] != Address) NextState = Idle;
			else if (PacketType == 1) begin
				if (CntH==10) NextState = Regs;
				else NextState = Hdrr;
			end else begin
				if (CntH==16) NextState = ChHd;  
				else NextState = Hdrr;
			end
		  end
	Regs: begin
			ena = 0;
			Stop = 1'b0;
			if (CntN==0) Stop = 1'b1;
			else Stop = 1'b0;
			if (CntN==66) 
				NextState = Idle;
			else 
				NextState = Regs;
		  end
    ChHd: begin
	        if (CntE == 0) begin
				ena = FourToSixteen(WritePtr);
			end else begin
				ena = 0;
			end
			if (Header[3:0] != 0) begin
				if (CntE==11) 
					NextState = Clus;
				else 
					NextState = ChHd;
				Stop = 1'b0;
			end else begin
				NextState = GCRC;
				Stop = 1'b1;
			end
		  end
    Clus: begin
			Stop = 1'b0;
	        if (CntB == 0) begin
				ena = FourToSixteen(WritePtr);   //Write the chip header or a cluster to RAM on the next clock
			end else begin
				ena = 0;
			end
			if (CntB == 10)
				NextState = EOCL;
			else 
				NextState = Clus;
		  end
	EOCL: begin
			ena = 0;
			Stop = 1'b0;
			if (CntC+1 == Chip[9:6] || Chip[9:6] == 0) begin
				if (CntF == Header[3:0]) begin
					NextState = CRC1;
				end else begin
					NextState = ChHd;
				end
			end else begin
				NextState = Clus;
			end
		  end
	CRC1: begin
			NextState = GCRC;
			Stop = 1'b1;
			ena = 0;
		  end
	GCRC: begin
			ena = 0;
			if (CntH==4) begin
				NextState = EOEV;
			end else begin
				NextState = GCRC;
			end
			Stop = 1'b0;
		  end
	EOEV: begin
		    if (Header[3:0] != 0) begin
			  ena = FourToSixteen(WritePtr);  //Write out the last cluster
			end else begin
			  ena = 0;
			end
			NextState = SNwd;
			Stop = 1'b0;
		  end
	SNwd: begin
			ena = FourToSixteen(WritePtr);	//Write the number of words
			NextState = Idle;
			Stop = 1'b0;
		  end
    default: begin NextState= Idle; ena = 0; Stop = 1'b0; end
  endcase
end

reg Bufree;
always @ (BufMngrType or BufOcc) begin
	if (BufMngrType) Bufree = (BufOcc < 8);
	else Bufree = (BufOcc < 4);
end

always @ (posedge Clock)
begin
  if (Reset) begin
    $display("%g\t EvtBuf12 %d:  reseting.  State=%b,  NextState=%b,  Idle=%b",$time,Address,State,NextState,Idle);
    State <= Idle;
	NFull <= 0;
	AddErr <= 1'b0;
	WritePtr <= 0;
	Start <= 1'b0;
	CRCerr <= 1'b0;
	evtCnt <= 0;
	BufOcc <= 0;
	trgIn <= 0;
  end else begin
    State <= NextState;
	// if (State == GCRC || NextState == GCRC) begin
		// DataDly <= 1'b0;
	// end else begin
		DataDly <= Data;
	//end
	if (Trigger) trgIn <= trgIn + 1;
	if (Trigger & !Done) begin
		BufOcc <= BufOcc + 1;
		$display("%g\t EvtBuf12 %d: incrementing buffer occupancy to %d",$time,Address,BufOcc+1);
	end
	if (Done & !Trigger) begin
		$display("%g\t EvtBuf12 %d: decrementing buffer occupancy to %d",$time,Address,BufOcc-1);
		BufOcc <= BufOcc - 1;
	end
	if (Done & Trigger) begin
		$display("%g\t EvtBuf12 %d: simultaneous Done and Trigger.  Keeping buffer occupancy at %d",$time,Address,BufOcc);
	end
	if (AllDone && !Done && NFull != 0) begin
		NFull <= NFull - 1;
		$display("%g\t EvtBuf12 %d:  decrementing NFull=%d, BufRdy=%b, events in %d, events out %d",$time,Address,NFull,BufRdy,evtCnt,evtOut);  
	end
	if (Done && !AllDone && NFull != 4'hf) begin
		NFull <= NFull + 1;
		$display("%g\t EvtBuf12 %d:  incrementing NFull=%d, BufRdy=%b, events in %d, events out %d",$time,Address,NFull,BufRdy,evtCnt,evtOut);  
	end
	// if (State != Idle && Address==4'd0) begin
		// $display("%g\t EvtBuf12 %d in: State=%b, Data=%b, DataDly=%b, NFull=%d, WritePtr=%d, Done=%b, Stop=%b, CRC=%b, events in=%d",$time,Address,State,Data,DataDly,NFull,WritePtr,Done,Stop,CRC,evtCnt);
		// $display("%g\t      EvtBuf12 %d in: CntC=%d Chip[9:6]=%d CntF=%d Header[3:0]=%d",$time,Address,CntC,Chip[9:6],CntF,Header[3:0]);
	// end
	case(State)
      Idle: begin
			  ErrSgnl <= 1'b0;
			  Done <= 1'b0;
              CntH <= 0;
			  CntE <= 0;
			  CntF <= 0;
			  CntN <= 0;
			  Addw <= 1;     //Address of the first word in the buffer (the layer header)
			  if (Data) begin
			    StWrd[0] <= 1'b1;
				Start <= 1'b1;
				$display("%g\t EvtBuf12 %d, Start signal occurance, event=%d is incrementing.  Events out=%d",$time,Address,evtCnt,evtOut);
				evtCnt <= evtCnt + 1;
			  end
            end
	  StBt: begin
			  CntH <= CntH + 1;
			  if (CntH == 0) PacketType <= Data;
			  StWrd <= {StWrd[4:0],Data};
			  Start <= 1'b0;
	        end
      Hdrr: begin
			  if (StWrd[3:0] != Address) begin
				AddErr <= 1'b1; 
				ErrBufIn[1] <= 1'b1;
				$display("%g\t EvtBuf12 %d, Address mismatch, StWrd[3:0]=%d.  Ignoring start sequence for event %d",$time,Address,StWrd[3:0],evtCnt);
			  end else ErrBufIn[1] <= 1'b0;
			  ErrBufIn <= 0;
              Header <= {Header[10:0], Data};
			  DataWordIn <= {Address,DataWordIn[6:0],Data};  //Shift in the layer header
              RegWrd <= {RegWrd[4:0], Data};
              CntH <= CntH + 1;
            end
	  Regs: begin
			  CntN <= CntN + 1;
              RegWrd <= {RegWrd[4:0], Data};
              if (CntN==0) begin
                $display("%g\t EvtBuf12 %d, New register readback.  Header=%b",$time,Address,Header[5:0]);
				case(Header[3:1])
				  3'b001: $display("             Calibration DAC readback");
				  3'b010: $display("             Threshold DAC readback");
				  3'b011: $display("             Configuration register readback");
				  3'b100: $display("             Data mask register readback");
				  3'b101: $display("             Trigger mask register readback");
				  3'b110: $display("             Calibration mask register readback");
				endcase
              end else begin
                if (((CntN)%6)==0) $display("%g\t EvtBuf12 %d, Register word= h%h  b%b",$time,Address,RegWrd,RegWrd);
			  end
            end
      ChHd: begin
	          if (CntE == 0) begin
				ErrBufIn[7:3] <= Header[9:5];
			    // if (CntF == 0) begin
			      // $display("%g\t EvtBuf12 %d, Layer header = %b, FPGA=%h, Event=%d, Tag=%b, Err=%b, # Chips=%2d",$time,Address,Header,StWrd[3:0],Header[11:7],Header[6:5],Header[4],Header[3:0]);
				  // $display("%g\t EvtBuf12 %d: storing layer header %b in location %d, address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
				// end else begin
				  // $display("%g\t     EvtBuf12 %d, Cluster %2d=%b, %2d %2d",$time,Address,CntC,Cluster,Cluster[11:6],Cluster[5:0]);
				  // $display("%g\t     EvtBuf12 %d: storing cluster %b in location %d, address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
				// end
				if (Header[3:0] != 0) Addw <= Addw + 1;
				CntF <= CntF + 1;
				CntC <= 0;
			    CntB <= 0;
			  end
              Chip <= {Chip[10:0], Data};
			  DataWordIn <= {DataWordIn[10:0], Data};  //Shift in a chip header
              CntE <= CntE + 1;
			  CntH <= 0;
			  CRC <= {CRC[4:0],Data};
            end
      Clus: begin
	          Cluster <= {Cluster[10:0],Data};
			  DataWordIn <= {DataWordIn[10:0], Data};  //Shift in a cluster
	          if (CntB == 0) begin
				ErrSgnl <= Chip[5];
				ErrBufIn[2] <= Chip[5];
                if (CntC == 0) begin
//				  $display("%g\t   EvtBuf12 %d, Chip header = %b, Overflow=%b, # Clusts=%2d, Chip=%2d, Chip Err=%b, Prty Err=%b",$time,Address,Chip,Chip[11],Chip[10:6],Chip[3:0],Chip[5],Chip[4]);
//				  $display("%g\t   EvtBuf12 %d: storing chip header %b in location %d, address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
				  if (Chip[10] != 0) begin
					AddErr <= 1'b1;
					$display("%g\t    EvtBuf12 %d: incorrect bit in the chip header.  Bit 10 should be zero.",$time,Address);
				  end
				  if (Chip[9:6] == 0) begin
					AddErr <= 1'b1;
					$display("%g\t    EvtBuf12 %d: number of clusters is zero.  Chip header=%b",$time,Address,Chip);
				  end
				end else begin
//				  $display("%g\t     EvtBuf12 %d, Cluster %2d=%b, %2d %2d",$time,Address,CntC,Cluster,Cluster[11:6],Cluster[5:0]);
//				  $display("%g\t     EvtBuf12 %d: storing cluster %b in location %d, address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
				end
				Addw <= Addw + 1;
		      end else ErrSgnl <= 1'b0;
			  CntB <= CntB + 1;
			end
	  EOCL: begin
	          CntB <= 0;
			  CntE <= 0;
			  CntC <= CntC + 1;
			  Cluster <= {Cluster[10:0],Data};
			  DataWordIn <= {DataWordIn[10:0], Data};
			  ErrSgnl <= 1'b0;
	        end
	  CRC1: begin
			  CRC <= {CRC[4:0],Data};
			end
	  GCRC: begin
			  CntH <= CntH + 1;
			  CRC <= {CRC[4:0],Data};
	        end
      EOEV: begin
	          // if (Header[3:0] != 0) begin
	            // $display("%g\t     EvtBuf12 %d, Cluster %2d=%b, %2d %2d",$time,Address,CntC,Cluster,Cluster[11:6],Cluster[5:0]);
				// $display("%g\t     EvtBuf12 %d: storing cluster %b in location %d, address %d, write-enable=%b",$time,Address,DataWordIn,WritePtr,Addw,ena);
			  // end
			  NWords[WritePtr] <= Addw;
			  DataWordIn <= Addw;
			  Addw <= 0;
//			  $display("%g\t EvtBuf12 %d:  incrementing the write pointer",$time,Address);
			  if (CRC != CRCnew) begin
				CRCerr <= 1'b1;
				ErrBufIn[0] <= 1'b1;
				$display("%g\t EvtBuf12 %d:  CRC mismatch.  CRC=%b, CRCnew=%b.",$time,Address,CRC,CRCnew);
			  end else ErrBufIn[0] <= 1'b0;
			end
	  SNwd:	begin
			  ErrBuf[WritePtr] <= ErrBufIn;
			  WritePtr <= WritePtr + 1;  
			  $display("%g\t     EvtBuf12 %d event %d in done, events out %d, triggers=%d: storing number of words %d in location %d address %d, write-enable=%b, NFull=%d BufOcc=%d",$time,Address,evtCnt,evtOut,trgIn,DataWordIn,WritePtr,Addw,ena,NFull,BufOcc);
			  Done <= 1'b1;
			  CRCerr <= 1'b0;
			end
     endcase
  end
end

//Module to check the incoming CRC
CRCcheck CRCcheck_U(CRCnew,DataDly,Stop,Start,Reset,Clock,Address);

//Instantiate Block RAM memory for the cluster buffers
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
//	$display("%g\t EvtBuf12 %d: DataWordOut=%b %b %b %b",$time,Address,DataWordOut0,DataWordOut1,DataWordOut2,DataWordOut3);
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
		evtOut <= 0;
	end else begin
		StateSd <= NextStateSd;
		if (SendErrBuf) begin
			ErrBufOut <= ErrBuf[ReadPtr];
//			$display("%g\t EvtBuf12 %d, Sending error buffer %b, StateSd=%b, events in=%d, events out=%d",$time,Address,ErrBuf[ReadPtr],StateSd,evtCnt,evtOut);
		end
		case (StateSd)
			WaitSd:	begin
						if (Send && (NFull > 0)) begin
							NWrds <= NWords[ReadPtr];
							Addr <= Addr + 1;
							evtOut <= evtOut + 1;
//							$display("%g\t EvtBuf12 %d events in=%d, events out=%d is incrementing; State WaitSd:  NWords=%d, ReadPtr=%d, Addr=%d, NFull=%d",$time,Address,evtCnt,evtOut,NWords[ReadPtr],ReadPtr,Addr,NFull);
						end else Addr <= 0;
						AllDone <= 1'b0;
					end
			NextSd:	begin
						Addr <= Addr + 1;
//						$display("%g\t EvtBuf12 %d, State NextSd:  Addr=%d, ReadPtr=%d, NWrds=%d",$time,Address,Addr,ReadPtr,NWrds);
					end
			DoneSd:	begin
						AllDone <= 1'b1;
						$display("%g\t EvtBuf12 %d, events in=%d, event going out=%d, triggers=%d.  Addr=%d, ReadPtr=%d, WritePtr=%d, NWrds=%d, NFull=%d, BufOcc=%d",$time,Address,evtCnt,evtOut,trgIn,Addr,ReadPtr,WritePtr,NWrds,NFull,BufOcc);
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


